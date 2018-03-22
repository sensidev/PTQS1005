#include "PTQS1005.h"


PTQS1005::PTQS1005(Serial &uart) {
    is_debug_mode = PTQS1005_DEBUG_MODE;
    _uart = &uart;
    _uart->baud(9600);
    _uart->format(8, Serial::None, 1);
}

void PTQS1005::uart_new_data_handler() {
    if (_uart->readable()) {
        raw_sensor_data[idx++] = (char) _uart->getc();
    }
}

void PTQS1005::init() {
    _uart->attach(this, &PTQS1005::uart_new_data_handler, _uart->RxIrq);
}

void PTQS1005::read_sensors_data() {
    idx = 0;

    debug_if(is_debug_mode, "Ask for sensor data \r\n");

    if (_uart->writeable()) {
        for (uint8_t i = 0; i < PTQS1005_CMD_BYTES_NO; i++) {
            _uart->putc(cmd[i]);
        }
    }
}

bool PTQS1005::is_raw_sensor_data_ready() {
    bool received_all_bytes = idx == PTQS1005_OUTPUT_BYTES_NO;

    debug_if(is_debug_mode && received_all_bytes, "Received all %d bytes.\r\n", PTQS1005_OUTPUT_BYTES_NO);

    return received_all_bytes && is_data_integral();
}

bool PTQS1005::is_data_integral() {
    uint16_t check_code = 0;

    uint8_t check_code_high_byte_idx = PTQS1005_OUTPUT_BYTES_NO - PTQS1005_CHECK_CODE_BYTES_NO;
    uint8_t check_code_low_byte_idx = PTQS1005_OUTPUT_BYTES_NO - PTQS1005_CHECK_CODE_BYTES_NO + 1;

    check_code |= (raw_sensor_data[check_code_high_byte_idx] & 0xFF) << 8;
    check_code |= (raw_sensor_data[check_code_low_byte_idx] & 0xFF);

    debug_if(is_debug_mode, "Received check code 0x%04X\r\n", check_code);

    return check_code == compute_check_code();
}

uint16_t PTQS1005::compute_check_code() {
    uint16_t check_code = 0;

    for (uint8_t i = 0; i < PTQS1005_OUTPUT_BYTES_NO - PTQS1005_CHECK_CODE_BYTES_NO; i++) {
        check_code += (uint16_t) raw_sensor_data[i];
    }

    debug_if(is_debug_mode, "Computed check code: 0x%04X\r\n", check_code);

    return check_code;
}

char *PTQS1005::get_raw_sensor_data_read() {
    if (is_debug_mode) {
        for (uint8_t i = 0; i < idx; i++) {
            debug("%02X ", raw_sensor_data[i]);
        }

        debug("\r\n");
    }
    return raw_sensor_data;
}


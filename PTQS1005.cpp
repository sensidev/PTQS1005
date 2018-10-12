#include "PTQS1005.h"


PTQS1005::PTQS1005(Serial &uart) {
    is_debug_mode = PTQS1005_DEBUG_MODE;
    _uart = &uart;
    _uart->baud(9600);
    _uart->format(8, Serial::None, 1);
}

void PTQS1005::uart_receive_new_data_handler() {
    if (_uart->readable()) {
        raw_sensor_data[sensor_data_idx++] = (char) _uart->getc();
    }
}

void PTQS1005::init() {
    _uart->attach(this, &PTQS1005::uart_receive_new_data_handler, _uart->RxIrq);
}

void PTQS1005::read_sensors_data() {
    int read_attempts = PTQS1005_READ_ATTEMPTS;

    sensor_data_idx = 0;

    debug_if(is_debug_mode, "Ask for sensor data \r\n");

    if (_uart->writeable()) {
        for (uint8_t i = 0; i < PTQS1005_CMD_BYTES_NO; i++) {
            _uart->putc(cmd[i]);
        }
    }

    while (!is_raw_sensor_data_ready() && read_attempts > 0) {
        wait_ms(PTQS1005_READ_DELAY_MS);
        read_attempts--;
    }

    if (0 == read_attempts) {
        debug_if(is_debug_mode, "Sensor data not available.\r\n");
    }
}

bool PTQS1005::is_raw_sensor_data_ready() {
    bool received_all_bytes = sensor_data_idx == PTQS1005_OUTPUT_BYTES_NO;

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
        for (uint8_t i = 0; i < sensor_data_idx; i++) {
            debug("%02X ", raw_sensor_data[i]);
        }

        debug("\r\n");
    }
    return raw_sensor_data;
}

uint16_t PTQS1005::get_unit16_for_index(uint8_t start) {
    uint16_t r = 0;

    r |= (raw_sensor_data[start] & 0xFF) << 8;
    r |= (raw_sensor_data[start + 1] & 0xFF);

    return r;
}

uint16_t PTQS1005::get_raw_pm25() {
    return get_unit16_for_index(PTQS1005_PM25_BYTE_IDX);
}

uint16_t PTQS1005::get_raw_tvoc() {
    return get_unit16_for_index(PTQS1005_TVOC_BYTE_IDX);
}

uint16_t PTQS1005::get_raw_hcho() {
    return get_unit16_for_index(PTQS1005_HCHO_BYTE_IDX);
}

uint16_t PTQS1005::get_raw_co2() {
    return get_unit16_for_index(PTQS1005_CO2_BYTE_IDX);
}

uint16_t PTQS1005::get_raw_temperature() {
    return get_unit16_for_index(PTQS1005_TEMPERATURE_BYTE_IDX);
}

uint16_t PTQS1005::get_raw_humidity() {
    return get_unit16_for_index(PTQS1005_HUMIDITY_BYTE_IDX);
}

uint16_t PTQS1005::get_divider_for(uint8_t eq_byte_index) {
    uint8_t eq = (uint8_t) raw_sensor_data[eq_byte_index];

    if (0 == eq) {
        return 10;
    } else if (1 == eq) {
        return 100;
    } else if (2 == eq) {
        return 1000;
    }

    debug_if(is_debug_mode, "Invalid 'equivalent' value %d. Proceed with default divider i.e. 10. \r\n", eq);

    return 10;
}

float PTQS1005::get_pm25() {
    return (float) get_raw_pm25();
}

float PTQS1005::get_tvoc() {
    return get_raw_tvoc() / get_divider_for(PTQS1005_TVOC_EQUIVALENCE_BYTE_IDX);
}

float PTQS1005::get_hcho() {
    return get_raw_hcho() / get_divider_for(PTQS1005_HCHO_EQUIVALENCE_BYTE_IDX);
}

float PTQS1005::get_co2() {
    return (float) get_raw_co2();
}

float PTQS1005::get_temperature() {
    return (float) get_raw_temperature() / 10;
}

float PTQS1005::get_humidity() {
    return (float) get_raw_humidity() / 10;
}

void PTQS1005::print_debug_info() {
    if (is_raw_sensor_data_ready()) {
        debug_if(is_debug_mode,"PTQS1005 PM2.5: %.2f ug/m3 \r\n", sensor.get_pm25());
        debug_if(is_debug_mode,"PTQS1005 TVOC: %.2f ppm \r\n", sensor.get_tvoc());
        debug_if(is_debug_mode,"PTQS1005 HCHO: %.2f ppm \r\n", sensor.get_hcho());
        debug_if(is_debug_mode,"PTQS1005 CO2: %.2f ppm \r\n", sensor.get_co2());
        debug_if(is_debug_mode,"PTQS1005 Temperature: %.2f C \r\n", sensor.get_temperature());
        debug_if(is_debug_mode,"PTQS1005 Humidity: %.2f %% \r\n", sensor.get_humidity());
    } else {
        debug_if(is_debug_mode, "Raw sensor data not ready!\r\n");
    }
}
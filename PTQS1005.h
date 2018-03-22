#ifndef PTQS1005_H
#define PTQS1005_H

#include "mbed.h"
#include "mbed_debug.h"

#define PTQS1005_DEBUG_MODE 1
#define PTQS1005_CMD_BYTES_NO 7
#define PTQS1005_OUTPUT_BYTES_NO 24
#define PTQS1005_CHECK_CODE_BYTES_NO 2

class PTQS1005 {
public:
    PTQS1005(Serial &uart);

    void init();

    bool is_raw_sensor_data_ready();
    void read_sensors_data();
    char* get_raw_sensor_data_read();

private:
    void uart_new_data_handler();
    uint16_t compute_check_code();
    bool is_data_integral();

    Serial *_uart;
    char cmd[PTQS1005_CMD_BYTES_NO] = {0x42, 0x4D, 0xAB, 0x00, 0x00, 0x01, 0x3A};
    char raw_sensor_data[PTQS1005_OUTPUT_BYTES_NO];
    uint8_t idx = 0;
    bool is_debug_mode;
};


#endif

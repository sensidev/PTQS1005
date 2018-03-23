#ifndef PTQS1005_H
#define PTQS1005_H

#include "mbed.h"
#include "mbed_debug.h"

#define PTQS1005_DEBUG_MODE 1
#define PTQS1005_CMD_BYTES_NO 7
#define PTQS1005_OUTPUT_BYTES_NO 24
#define PTQS1005_CHECK_CODE_BYTES_NO 2

#define PTQS1005_PM25_BYTE_IDX 4

#define PTQS1005_TVOC_BYTE_IDX 6
#define PTQS1005_TVOC_EQUIVALENCE_BYTE_IDX 8

#define PTQS1005_HCHO_BYTE_IDX 9
#define PTQS1005_HCHO_EQUIVALENCE_BYTE_IDX 11

#define PTQS1005_CO2_BYTE_IDX 12

#define PTQS1005_TEMPERATURE_BYTE_IDX 14

#define PTQS1005_HUMIDITY_BYTE_IDX 16

#define PTQS1005_READ_ATTEMPTS 3
#define PTQS1005_READ_DELAY_MS 50

class PTQS1005 {
public:
    PTQS1005(Serial &uart);

    void init();

    void read_sensors_data();

    bool is_raw_sensor_data_ready();

    uint16_t get_raw_pm25();

    uint16_t get_raw_tvoc();

    uint16_t get_raw_hcho();

    uint16_t get_raw_co2();

    uint16_t get_raw_temperature();

    uint16_t get_raw_humidity();

    float get_pm25();

    float get_tvoc();

    float get_hcho();

    float get_co2();

    float get_temperature();

    float get_humidity();

    char *get_raw_sensor_data_read();

private:
    void uart_receive_new_data_handler();

    uint16_t compute_check_code();

    bool is_data_integral();

    uint16_t get_divider_for(uint8_t eq_byte_index);

    uint16_t get_unit16_for_index(uint8_t start);

    Serial *_uart;
    char cmd[PTQS1005_CMD_BYTES_NO] = {0x42, 0x4D, 0xAB, 0x00, 0x00, 0x01, 0x3A};
    char raw_sensor_data[PTQS1005_OUTPUT_BYTES_NO];
    uint8_t sensor_data_idx = 0;
    bool is_debug_mode;
};


#endif

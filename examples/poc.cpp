#include "mbed.h"
#include "PTQS1005.h"

Serial uart(UART_TX, UART_RX);

Serial pc(USBTX, USBRX);

PTQS1005 sensor(uart);


int main() {
    pc.baud(115200);

    sensor.init();

    while (true) {
        pc.printf("-------------------- \r\n");

        sensor.read_sensors_data();

        if (sensor.is_raw_sensor_data_ready()) {
            pc.printf("PTQS1005 PM2.5: %.2f ug/m3 \r\n", sensor.get_pm25());
            pc.printf("PTQS1005 TVOC: %.2f ppm \r\n", sensor.get_tvoc());
            pc.printf("PTQS1005 HCHO: %.2f ppm \r\n", sensor.get_hcho());
            pc.printf("PTQS1005 CO2: %.2f ppm \r\n", sensor.get_co2());
            pc.printf("PTQS1005 Temperature: %.2f C \r\n", sensor.get_temperature());
            pc.printf("PTQS1005 Humidity: %.2f %% \r\n", sensor.get_humidity());
        }

        sensor.get_raw_sensor_data_read();

        wait(2);
    }
}
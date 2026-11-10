/* Writes a synthetic sensor-test session with the firmware's own SensorTestRecord layout, so the
 * Python reader (telemetry_viewer.parse_sensor_session) is checked against the C struct.
 * 10 s: IMU 400 Hz, barometer 50 Hz, ADC 10 Hz, status 1 Hz (humidity failing "now" only). */
#include <stdio.h>
#include <string.h>

#include "sensor_test.h"

static FILE *out;
static unsigned char seq;

static void put(SensorTestRecord *r, unsigned type, unsigned t_us)
{
    r->t_us = t_us;
    r->session = 7;
    r->type = (unsigned char)type;
    r->seq = seq++;
    r->marker = 0xA5;
    fwrite(r, 1, sizeof(*r), out);
}

int main(int argc, char **argv)
{
    if (argc != 2 || !(out = fopen(argv[1], "wb"))) return 1;
    unsigned char header[512] = {0};
    const unsigned session = 7, record_size = 32;
    memcpy(header, "ETVCLOG2", 8);
    memcpy(header + 8, &session, 4);
    memcpy(header + 12, &record_size, 4);
    strcpy((char *)header + 16, "EasyTVC mode=sensortest imu_hz=400");
    fwrite(header, 1, sizeof(header), out);

    SensorTestRecord r;
    for (unsigned i = 0; i < 4000; ++i) {
        const unsigned t = 1000000u + i * 2500u;
        memset(&r, 0, sizeof(r));
        r.u.imu.accel[0] = (short)(i % 2);
        r.u.imu.accel[2] = (short)(1365 + i % 3);
        r.u.imu.gyro[1] = -16;
        r.u.imu.accel_temp = 2650;
        r.u.imu.read_us = 40;
        put(&r, SENSOR_REC_IMU, t);
        if (i % 8 == 0) {
            memset(&r, 0, sizeof(r));
            r.u.baro.pressure_pa = 101325;
            r.u.baro.temperature_cc = 2412;
            r.u.baro.humidity_q10 = 45 * 1024;
            r.u.baro.adc_p = 300000;
            r.u.baro.adc_t = 500000;
            r.u.baro.adc_h = 30000;
            put(&r, SENSOR_REC_BARO, t + 1);
        }
        if (i % 40 == 0) {
            memset(&r, 0, sizeof(r));
            r.u.adc.vdda_mv = 3301;
            r.u.adc.batt_pin_mv = 1234;
            r.u.adc.mcu_temp_cc = 3100;
            r.u.adc.max_loop_us = 900;
            put(&r, SENSOR_REC_ADC, t + 2);
        }
        if (i % 400 == 399) {
            memset(&r, 0, sizeof(r));
            r.u.status.checks = SENSOR_CHECK_ALL & ~SENSOR_CHECK_HUMIDITY;
            r.u.status.boot_checks = SENSOR_CHECK_ALL;
            r.u.status.accel_id = 0x1E;
            r.u.status.gyro_id = 0x0F;
            r.u.status.baro_id = 0x60;
            r.u.status.imu_samples = i;
            put(&r, SENSOR_REC_STATUS, t + 3);
        }
    }
    const unsigned char erased[64] = {0};   /* end of session: no marker */
    fwrite(erased, 1, sizeof(erased), out);
    fclose(out);
    return 0;
}

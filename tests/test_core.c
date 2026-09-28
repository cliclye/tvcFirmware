#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "attitude.h"
#include "bme280.h"
#include "bmi088.h"
#include "easytvc_safety.h"
#include "flight_fsm.h"
#include "logger.h"
#include "nav.h"
#include "pid.h"
#include "pyro_seq.h"
#include "telemetry.h"
#include "tvc.h"

static EasyTVCFlightConfig test_config(void)
{
    return (EasyTVCFlightConfig){
        .launch_accel_g = 2.0f,
        .launch_confirm_ms = 50U,
        .minimum_boost_ms = 200U,
        .burnout_axial_accel_g = 0.5f,
        .minimum_apogee_ms = 800U,
        .failsafe_apogee_ms = 4000U,
        .main_deploy_altitude_m = 250.0f,
        .landed_speed_m_s = 0.5f,
        .landed_confirm_ms = 500U,
        .tvc_enable_delay_ms = 100U,
    };
}

static EasyTVCFlightInput healthy_input(uint32_t now_ms)
{
    return (EasyTVCFlightInput){
        .now_ms = now_ms,
        .sensors_healthy = true,
        .physical_arm_qualified = true,
        .software_arm_request = true,
        .acceleration_magnitude_g = 1.0f,
        .axial_acceleration_g = 2.0f,
        .altitude_m = 500.0f,
        .vertical_speed_m_s = 50.0f,
    };
}

static void test_flight_machine(void)
{
    EasyTVCFlightMachine machine;
    const EasyTVCFlightConfig config = test_config();
    EasyTVC_FlightInit(&machine, &config, 0U);

    EasyTVCFlightInput input = healthy_input(0U);
    input.physical_arm_qualified = false;
    assert(EasyTVC_FlightStep(&machine, &input).state == EASYTVC_FLIGHT_IDLE);

    input = healthy_input(1U);
    assert(EasyTVC_FlightStep(&machine, &input).state == EASYTVC_FLIGHT_ARMED);

    input = healthy_input(10U);
    input.acceleration_magnitude_g = 3.0f;
    assert(EasyTVC_FlightStep(&machine, &input).state == EASYTVC_FLIGHT_ARMED);
    input.now_ms = 61U;
    EasyTVCFlightOutput output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_BOOST);
    assert(!output.tvc_enabled);

    input.now_ms = 161U;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.tvc_enabled);

    input.now_ms = 300U;
    input.axial_acceleration_g = 0.2f;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_COAST);

    input.now_ms = 900U;
    input.vertical_speed_m_s = -1.0f;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_APOGEE);
    assert(output.drogue_deploy_requested);
    assert(!output.main_deploy_requested);

    input.now_ms = 901U;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_DESCENT);
    assert(!output.drogue_deploy_requested);

    input.now_ms = 902U;
    input.altitude_m = 249.0f;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.main_deploy_requested);
    assert(!output.drogue_deploy_requested);

    input.now_ms = 1402U;
    input.vertical_speed_m_s = 0.1f;
    input.altitude_m = 1.0f;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_DESCENT);
    input.now_ms = 1902U;
    output = EasyTVC_FlightStep(&machine, &input);
    assert(output.state == EASYTVC_FLIGHT_LANDED);
}

static void test_fault_on_unhealthy_sensors(void)
{
    EasyTVCFlightMachine machine;
    const EasyTVCFlightConfig config = test_config();
    EasyTVC_FlightInit(&machine, &config, 0U);
    EasyTVCFlightInput input = healthy_input(1U);
    assert(EasyTVC_FlightStep(&machine, &input).state == EASYTVC_FLIGHT_ARMED);
    input.now_ms = 2U;
    input.sensors_healthy = false;
    assert(EasyTVC_FlightStep(&machine, &input).state == EASYTVC_FLIGHT_FAULT);
}

static void test_pid(void)
{
    const EasyTVCPidConfig config = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integrator_limit = 1.0f,
        .output_limit = 5.0f,
        .output_rate_limit = 10.0f,
    };
    EasyTVCPid pid;
    EasyTVC_PidReset(&pid);
    assert(fabsf(EasyTVC_PidUpdate(&pid, &config, 10.0f, 0.0f, 0.01f) - 0.1f) < 0.0001f);
    assert(fabsf(EasyTVC_PidUpdate(&pid, &config, 10.0f, 0.0f, 0.01f) - 0.2f) < 0.0001f);
}

static void test_attitude(void)
{
    EasyTVCAttitudeFilter filter;
    EasyTVC_AttitudeInit(&filter, 1.0f, 0.0f);
    const float gyro[3] = {0.0f, 0.0f, 1.0f};
    const float accel[3] = {0.0f, 0.0f, 9.80665f};
    for (int index = 0; index < 100; ++index) {
        EasyTVC_AttitudeUpdate(&filter, gyro, accel, false, 0.01f);
    }
    const EasyTVCEulerAngles angles = EasyTVC_AttitudeEuler(&filter);
    assert(fabsf(angles.yaw_rad - 1.0f) < 0.02f);
    assert(fabsf(filter.orientation.w * filter.orientation.w +
                 filter.orientation.x * filter.orientation.x +
                 filter.orientation.y * filter.orientation.y +
                 filter.orientation.z * filter.orientation.z - 1.0f) < 0.001f);
}

static void test_logger(void)
{
    EasyTVCLogRecord storage[2];
    EasyTVCLogRing ring;
    EasyTVC_LogRingInit(&ring, storage, 2U);
    for (uint32_t timestamp = 1U; timestamp <= 3U; ++timestamp) {
        const EasyTVCLogRecord record = {.timestamp_ms = timestamp};
        EasyTVC_LogRingAppend(&ring, &record);
    }
    EasyTVCLogRecord record;
    assert(ring.count == 2U && ring.overflowed);
    assert(EasyTVC_LogRingGet(&ring, 0U, &record) && record.timestamp_ms == 2U);
    assert(EasyTVC_LogRingGet(&ring, 1U, &record) && record.timestamp_ms == 3U);
}

static void test_pyro_compile_lock(void)
{
    EasyTVCPyroSequencer sequencer;
    const EasyTVCPyroConfig config = {.pulse_ms = 500U};
    EasyTVC_PyroSequencerInit(&sequencer);

    assert(!EasyTVC_PyroPolicyAllows(EASYTVC_FLIGHT_APOGEE,
                                     EASYTVC_PYRO_EVENT_DROGUE,
                                     true));
    assert(!EasyTVC_PyroPolicyAllows(EASYTVC_FLIGHT_DESCENT,
                                     EASYTVC_PYRO_EVENT_MAIN,
                                     true));

    const EasyTVCPyroCommand command =
        EasyTVC_PyroSequencerStep(&sequencer,
                                  &config,
                                  EASYTVC_FLIGHT_APOGEE,
                                  true,
                                  true,
                                  true,
                                  10U);
    assert(!command.request_drogue_output);
    assert(!command.request_main_output);
}

static int fake_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    const uint8_t *table = (const uint8_t *)bus->context;
    if (length == 0U) {
        return 0;
    }
    data[0] = table[reg];
    return 0;
}

static int fake_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    (void)bus;
    (void)reg;
    (void)data;
    (void)length;
    return 0;
}

static void test_sensor_identity_fakes(void)
{
    uint8_t accel_regs[1] = {EASYTVC_BMI088_ACCEL_CHIP_ID};
    uint8_t gyro_regs[1] = {EASYTVC_BMI088_GYRO_CHIP_ID};
    uint8_t baro_regs[0xD1];
    memset(baro_regs, 0, sizeof(baro_regs));
    baro_regs[EASYTVC_BME280_CHIP_ID_REG] = EASYTVC_BME280_CHIP_ID;

    EasyTVCBus accel = {.context = accel_regs, .write = fake_write, .read = fake_read};
    EasyTVCBus gyro = {.context = gyro_regs, .write = fake_write, .read = fake_read};
    EasyTVCBus baro = {.context = baro_regs, .write = fake_write, .read = fake_read};

    EasyTVCBmi088 imu;
    EasyTVCBme280 bme;
    EasyTVC_Bmi088Init(&imu, &accel, &gyro);
    EasyTVC_Bme280Init(&bme, &baro);
    assert(EasyTVC_Bmi088Probe(&imu));
    assert(EasyTVC_Bme280Probe(&bme));

    accel_regs[0] = 0x00U;
    assert(!EasyTVC_Bmi088Probe(&imu));
}

static void test_tvc_holds_center_when_disabled(void)
{
    EasyTVCTvc tvc;
    const EasyTVCTvcConfig config = {
        .pitch = {.kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .integrator_limit = 1.0f,
                  .output_limit = 1.0f, .output_rate_limit = 10.0f},
        .yaw = {.kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .integrator_limit = 1.0f,
                .output_limit = 1.0f, .output_rate_limit = 10.0f},
        .servo_center = 0.5f,
        .servo_throw = 0.25f,
    };
    EasyTVC_TvcInit(&tvc);
    const EasyTVCTvcOutput output = EasyTVC_TvcUpdate(&tvc, &config, 0.4f, -0.3f, false, 0.01f);
    assert(fabsf(output.pitch_command) < 0.0001f);
    assert(fabsf(output.yaw_command) < 0.0001f);
    assert(fabsf(output.servo_normalized[0] - 0.5f) < 0.0001f);
}

static void test_telemetry_and_app(void)
{
    EasyTVCApp app;
    EasyTVC_AppInit(&app, 0U);

    EasyTVCAppInput input = {
        .gyro_rad_s = {0.0f, 0.0f, 0.0f},
        .accel_m_s2 = {0.0f, 0.0f, 9.80665f},
        .baro_altitude_m = 0.0f,
        .sensors_healthy = true,
        .physical_arm_qualified = true,
        .software_arm_request = true,
        .accel_is_gravity_reference = true,
        .now_ms = 0U,
        .dt_s = 0.01f,
    };

    EasyTVCAppOutput output = EasyTVC_AppStep(&app, &input);
    assert(output.flight.state == EASYTVC_FLIGHT_ARMED);
    assert(output.pyro.request_drogue_output == false);
    assert(strchr(output.telemetry, ',') != 0);
    assert(strstr(output.telemetry, ",1,") != 0);
}

int main(void)
{
    test_flight_machine();
    test_fault_on_unhealthy_sensors();
    test_pid();
    test_attitude();
    test_logger();
    test_pyro_compile_lock();
    test_sensor_identity_fakes();
    test_tvc_holds_center_when_disabled();
    test_telemetry_and_app();
    return 0;
}

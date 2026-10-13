cmake_minimum_required(VERSION 3.22)

project(EasyTVCFirmware LANGUAGES C)

set(EASYTVC_CORE_SOURCES
    Core/Src/app.c
    Core/Src/attitude.c
    Core/Src/bme280.c
    Core/Src/bmi088.c
    Core/Src/bus.c
    Core/Src/easytvc_safety.c
    Core/Src/flight_fsm.c
    Core/Src/logger.c
    Core/Src/nav.c
    Core/Src/pid.c
    Core/Src/pyro_seq.c
    Core/Src/telemetry.c
    Core/Src/tvc.c
    Core/Src/settings.c
    Core/Src/controller.c
    Core/Src/protocol.c
    Core/Src/runtime.c
    Core/Src/pad_arm.c
)

# STM32F405 register-level drivers. These touch memory-mapped peripherals and inline
# Cortex-M asm, so they are built for the MCU only and never for the host tests.
set(EASYTVC_HW_SOURCES
    Core/Src/stm32f4xx.c
    Core/Src/gpio_init.c
    Core/Src/spi_driver.c
    Core/Src/i2c_driver.c
    Core/Src/pwm_driver.c
    Core/Src/uart_driver.c
    Core/Src/hardware_bus.c
    Core/Src/sd_raw.c
    Core/Src/data_logger.c
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    # Compile the entire flight core for the real CPU as well as host tests.
    # This archive is NOT an operational board image: hardware adapters are still required.
    add_library(easytvc_core STATIC ${EASYTVC_CORE_SOURCES})
    target_include_directories(easytvc_core PUBLIC Core/Inc)
    target_compile_features(easytvc_core PUBLIC c_std_11)
    target_compile_options(easytvc_core PRIVATE -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        -mfpu=fpv4-sp-d16 -ffreestanding -ffunction-sections -fdata-sections -Wall -Wextra -Werror)
    add_library(easytvc_hw STATIC ${EASYTVC_HW_SOURCES})
    target_include_directories(easytvc_hw PUBLIC Core/Inc)
    target_compile_features(easytvc_hw PUBLIC c_std_11)
    target_compile_options(easytvc_hw PRIVATE -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        -mfpu=fpv4-sp-d16 -ffreestanding -ffunction-sections -fdata-sections -Wall -Wextra -Werror)
    target_link_libraries(easytvc_hw PUBLIC easytvc_core)

    set(EASYTVC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/ld/stm32f405rg.ld)

    # Two images from the same source: the bench image never arms and lets the gimbal follow the
    # board by hand; the flight image arms on the pad and runs TVC during boost.
    function(easytvc_image name bench_mode)
        add_executable(${name}
            Core/Src/main.c
            Core/Src/startup_stm32f405rg.c
            Core/Src/easytvc_safety.c
        )
        target_include_directories(${name} PRIVATE Core/Inc)
        target_compile_features(${name} PRIVATE c_std_11)
        target_compile_definitions(${name} PRIVATE EASYTVC_BENCH_MODE=${bench_mode})
        target_compile_options(${name} PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -mfloat-abi=hard
            -mfpu=fpv4-sp-d16
            -ffreestanding
            -fdata-sections
            -ffunction-sections
            -fno-common
            -Wall
            -Wextra
            -Werror
        )
        target_link_libraries(${name} PRIVATE easytvc_hw easytvc_core m)
        target_link_options(${name} PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -mfloat-abi=hard
            -mfpu=fpv4-sp-d16
            -nostartfiles
            --specs=nano.specs
            --specs=nosys.specs
            -Wl,--gc-sections
            -Wl,-T,${EASYTVC_LINKER_SCRIPT}
            -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/${name}.map
        )
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${name}> ${CMAKE_CURRENT_BINARY_DIR}/${name}.hex
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> ${CMAKE_CURRENT_BINARY_DIR}/${name}.bin
            COMMENT "Writing ${name}.hex and ${name}.bin"
        )
    endfunction()

    easytvc_image(easytvc_bench 1)
    easytvc_image(easytvc_flight 0)
else()
    add_library(easytvc_core STATIC ${EASYTVC_CORE_SOURCES})
    target_include_directories(easytvc_core PUBLIC Core/Inc)
    target_compile_features(easytvc_core PUBLIC c_std_11)
    target_compile_options(easytvc_core PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(easytvc_core PUBLIC m)
    set_target_properties(easytvc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

    add_library(easytvc_studio SHARED Core/Src/studio_sim.c Core/Src/flight_sim3d.c)
    target_link_libraries(easytvc_studio PRIVATE easytvc_core)
    target_compile_options(easytvc_studio PRIVATE -Wall -Wextra -Werror)

    add_executable(easytvc_test_core tests/test_core.c)
    target_link_libraries(easytvc_test_core PRIVATE easytvc_core)
    target_compile_options(easytvc_test_core PRIVATE -UNDEBUG)

    add_executable(easytvc_test_reliability tests/test_reliability.c)
    target_link_libraries(easytvc_test_reliability PRIVATE easytvc_core)
    target_compile_options(easytvc_test_reliability PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    add_executable(easytvc_test_drivers tests/test_drivers.c)
    target_link_libraries(easytvc_test_drivers PRIVATE easytvc_core)
    target_compile_options(easytvc_test_drivers PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    # The SD logger is hardware-independent apart from sd_raw, which the test replaces.
    add_executable(easytvc_test_logger tests/test_logger.c Core/Src/data_logger.c)
    target_include_directories(easytvc_test_logger PRIVATE Core/Inc)
    target_compile_options(easytvc_test_logger PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    add_executable(easytvc_test_flight tests/test_flight.c)
    target_link_libraries(easytvc_test_flight PRIVATE easytvc_core)
    target_compile_options(easytvc_test_flight PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    add_executable(easytvc_test_sim3d tests/test_sim3d.c Core/Src/flight_sim3d.c)
    target_link_libraries(easytvc_test_sim3d PRIVATE easytvc_core)
    target_compile_options(easytvc_test_sim3d PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    add_executable(easytvc_sim tests/sim_flight.c)
    target_link_libraries(easytvc_sim PRIVATE easytvc_core)

    add_executable(easytvc_protocol_fixture tests/protocol_console.c)
    target_link_libraries(easytvc_protocol_fixture PRIVATE easytvc_core)

    enable_testing()
    add_test(NAME easytvc_test_core COMMAND easytvc_test_core)
    add_test(NAME easytvc_test_reliability COMMAND easytvc_test_reliability)
    add_test(NAME easytvc_test_drivers COMMAND easytvc_test_drivers)
    add_test(NAME easytvc_test_logger COMMAND easytvc_test_logger)
    add_test(NAME easytvc_test_flight COMMAND easytvc_test_flight)
    add_test(NAME easytvc_test_sim3d COMMAND easytvc_test_sim3d)
endif()

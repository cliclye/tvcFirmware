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
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    # Compile the entire flight core for the real CPU as well as host tests.
    # This archive is NOT an operational board image: hardware adapters are still required.
    add_library(easytvc_core STATIC ${EASYTVC_CORE_SOURCES})
    target_include_directories(easytvc_core PUBLIC Core/Inc)
    target_compile_features(easytvc_core PUBLIC c_std_11)
    target_compile_options(easytvc_core PRIVATE -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        -mfpu=fpv4-sp-d16 -ffreestanding -ffunction-sections -fdata-sections -Wall -Wextra -Werror)
    set(EASYTVC_TARGET easytvc_safe_blank)
    set(EASYTVC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/ld/stm32f405rg.ld)

    add_executable(${EASYTVC_TARGET}
        Core/Src/easytvc_safety.c
        Core/Src/main.c
        Core/Src/startup_stm32f405rg.c
    )

    target_include_directories(${EASYTVC_TARGET} PRIVATE Core/Inc)
    target_compile_features(${EASYTVC_TARGET} PRIVATE c_std_11)
    target_compile_options(${EASYTVC_TARGET} PRIVATE
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
    target_link_options(${EASYTVC_TARGET} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -nostartfiles
        -nostdlib
        -Wl,--gc-sections
        -Wl,-T,${EASYTVC_LINKER_SCRIPT}
        -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/${EASYTVC_TARGET}.map
    )
    add_custom_command(TARGET ${EASYTVC_TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
                $<TARGET_FILE:${EASYTVC_TARGET}>
                ${CMAKE_CURRENT_BINARY_DIR}/${EASYTVC_TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
                $<TARGET_FILE:${EASYTVC_TARGET}>
                ${CMAKE_CURRENT_BINARY_DIR}/${EASYTVC_TARGET}.bin
        COMMENT "Writing Intel HEX and raw binary artifacts"
    )
else()
    add_library(easytvc_core STATIC ${EASYTVC_CORE_SOURCES})
    target_include_directories(easytvc_core PUBLIC Core/Inc)
    target_compile_features(easytvc_core PUBLIC c_std_11)
    target_compile_options(easytvc_core PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(easytvc_core PUBLIC m)
    set_target_properties(easytvc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

    add_library(easytvc_studio SHARED Core/Src/studio_sim.c)
    target_link_libraries(easytvc_studio PRIVATE easytvc_core)
    target_compile_options(easytvc_studio PRIVATE -Wall -Wextra -Werror)

    add_executable(easytvc_test_core tests/test_core.c)
    target_link_libraries(easytvc_test_core PRIVATE easytvc_core)
    target_compile_options(easytvc_test_core PRIVATE -UNDEBUG)

    add_executable(easytvc_test_reliability tests/test_reliability.c)
    target_link_libraries(easytvc_test_reliability PRIVATE easytvc_core)
    target_compile_options(easytvc_test_reliability PRIVATE -Wall -Wextra -Werror -UNDEBUG)

    add_executable(easytvc_sim tests/sim_flight.c)
    target_link_libraries(easytvc_sim PRIVATE easytvc_core)

    add_executable(easytvc_protocol_fixture tests/protocol_console.c)
    target_link_libraries(easytvc_protocol_fixture PRIVATE easytvc_core)

    enable_testing()
    add_test(NAME easytvc_test_core COMMAND easytvc_test_core)
    add_test(NAME easytvc_test_reliability COMMAND easytvc_test_reliability)
endif()

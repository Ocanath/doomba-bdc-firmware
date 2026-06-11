add_library(motor_controller STATIC
    motor.cpp
	trig_fixed.c
)

target_include_directories(motor_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(motor_controller PUBLIC
    dartt_serial
    dartt_audio
)

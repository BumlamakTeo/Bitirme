idf_component_register(SRCS "mcpwm_servo_control_example_main.c"
                            "servo_thread.c"
                            "serial_thread.c"
                       PRIV_REQUIRES esp_driver_mcpwm driver
                       INCLUDE_DIRS ".")

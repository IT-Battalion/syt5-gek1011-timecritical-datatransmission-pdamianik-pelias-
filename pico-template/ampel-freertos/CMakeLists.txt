add_executable(ampel-freertos
        main.c
        )

# pull in common dependencies
target_link_libraries(ampel-freertos pico_stdlib hardware_spi pico_multicore FreeRTOS)

# create map/bin/hex file etc.
pico_add_extra_outputs(ampel-freertos)

# add url via pico_set_program_url
example_auto_set_url(ampel-freertos)


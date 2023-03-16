add_executable(ampel-freertos-master
        main.c
        )

# pull in common dependencies
target_link_libraries(ampel-freertos-master pico_stdlib hardware_spi hardware_dma pico_multicore FreeRTOS)

# create map/bin/hex file etc.
pico_add_extra_outputs(ampel-freertos-master)

# add url via pico_set_program_url
example_auto_set_url(ampel-freertos-master)


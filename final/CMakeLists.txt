add_executable(final)

# must match with pio filename and executable name from above
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(final ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(final PRIVATE main.c vga_graphics.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(final pico_stdlib pico_bootsel_via_double_reset pico_multicore hardware_pwm hardware_dma hardware_irq hardware_adc hardware_pio hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(final)
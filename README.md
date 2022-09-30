# stm32f4xx-dma-i2c-example
Example for NUCLEO-F411 board with display SSD1306 and temperature probe LM75B

This example uses a i2c bus with SCL and SDA on PB8 and PB9. The i2c bus have a monochrome display SSD1306 and temperature probe LM75B.

## Description

`DMAI2cInterface` was crated to connect display using DMA. This structure is supplied to display driver in crate `ssd1306`.

Temperature probe uses raw I2C bus directly in task `get_temp`

There are interrupts handlers `i2c_dma_tx_it` and `i2c_dma_rx_it` for corresponding DMA Stream interrupts.

A `draw` task just displays current temperature from sensor to display.

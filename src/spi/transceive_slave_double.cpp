#include <device.h>
#include <drivers/spi.h>
#include <stdio.h>

int spi_master_transceive(const struct device *spi, struct spi_config *spi_cfg,
                          double *tx_data, double *rx_data) {

  struct spi_buf tx_bufs = {.buf = tx_data, .len = 4};
  struct spi_buf_set tx = {.buffers = &tx_bufs};

  struct spi_buf rx_bufs = {.buf = rx_data, .len = 4};
  struct spi_buf_set rx = {.buffers = &rx_bufs};

  tx.count = 1;
  rx.count = 1;

  return spi_transceive(spi, spi_cfg, &tx, &rx);
}

void main() {
  // Data to be exchanged
  double tx_data = 3.655;
  double rx_data = 5.133;

  // Initialize SPI bus number 3
  const struct device *spi = device_get_binding("SPI_1");

  // Chip select pin
  // const spi_cs_control cs_ctrl = {
  // 	.gpio_dev = device_get_binding("GPIOB"),
  // 	.delay = 0,
  // 	.gpio_pin = 6,
  // 	.gpio_dt_flags = GPIO_ACTIVE_LOW,
  // };

  // SPI COnfiguration
  struct spi_config spi_cfg {
    .frequency = 3000000, .operation = SPI_WORD_SET(16) | SPI_OP_MODE_SLAVE
    // , .cs = &cs_ctrl
  };

  while (1) {

    spi_master_transceive(spi, &spi_cfg, &tx_data, &rx_data);
    printf("Sent: %f\n", tx_data);
    printf("Received: %f\n\n", rx_data);
  }
}

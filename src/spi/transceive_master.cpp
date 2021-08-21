#include <device.h>
#include <drivers/spi.h>
#include <stdio.h>

int spi_master_transceive(const struct device *spi, struct spi_config *spi_cfg,
                          uint16_t *tx_data, uint16_t *rx_data) {

  struct spi_buf tx_bufs = {.buf = tx_data, .len = 2};
  struct spi_buf_set tx = {.buffers = &tx_bufs};

  struct spi_buf rx_bufs = {.buf = rx_data, .len = 2};
  struct spi_buf_set rx = {.buffers = &rx_bufs};

  tx.count = 1;
  rx.count = 1;

  return spi_transceive(spi, spi_cfg, &tx, &rx);
}

void main() {
  // Data to be exchanged
  uint16_t tx_data = 6;
  uint16_t rx_data = 2;

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
    .frequency = 8000000,
    .operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER
    // , .cs = &cs_ctrl
  };

  while (1) {

    spi_master_transceive(spi, &spi_cfg, &tx_data, &rx_data);
    printf("Sent: %d\n", tx_data);
    printf("Received: %d\n\n", rx_data);
    k_sleep(K_TIMEOUT_ABS_MS(1));
  }
}

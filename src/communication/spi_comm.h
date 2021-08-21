#pragma once

#include <device.h>
#include <drivers/spi.h>

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
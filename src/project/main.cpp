#include "quad_2d.h"
#include "spi_comm.h"
#include <math.h>
#include <stdio.h>

int main() {
  Quad2D quad;

  // Initialize SPI bus
  const struct device *spi = device_get_binding("SPI_1");

  // Set 8 Mhz frequency and 3Mhz transfer
  struct spi_config spi_cfg {
    .frequency = 3000000,
    .operation = SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER
  };

  float altitude_target = 5;
  float thrust_command = 0.0;

  // Altitude PID Gains
  float k_p__z = 6.5;
  float k_i__z = 0;
  float k_d__z = 2;

  // Translation PID Gains
  float k_p__x = 6.5;
  float k_i__x = 0;
  float k_d__x = 2;

  // Euler integration timestep
  constexpr static float dt = 0.01;
  constexpr static float euler_steps = 400;

  // feedforward thrust = - g
  float ff_thrust = 9.81;

  double tx_data = 4;
  double rx_data = 0;

  for (int i = 0; i < euler_steps; i++) {

    // // Get system state
    quad.sensor_read();

    // // Send system state
    tx_data = quad.z_mes();
    spi_master_transceive(spi, &spi_cfg, &tx_data, &rx_data);
    printf("Sent: %f\n", tx_data);

    // Compute control input
    float altitude_error = altitude_target - quad.z_mes();

    // Apply control input and compute the change
    quad.dynamics(thrust_command, 0);
    // quad.euler_step(dt);

    k_sleep(K_TIMEOUT_ABS_MS(1));
  }
}
#include "quad_2d.h"
#include "spi_comm.h"
#include <math.h>
#include <stdio.h>

int main() {
  Quad2D quad;

  // Initialize SPI bus
  const struct device *spi = device_get_binding("SPI_1");

  // Freqeuncy of altotude control loop in Hz
  constexpr static int altitude_rate = 200;
  constexpr static int altitude_dt = 1 / altitude_rate;

  // Set 8 Mhz frequency and 3Mhz transfer
  struct spi_config spi_cfg {
    .frequency = 3000000, .operation = SPI_WORD_SET(16) | SPI_OP_MODE_SLAVE
  };

  // Euler integration timestep
  constexpr static float dt = 0.01;
  constexpr static int euler_steps = 500;

  double z_measured = 4;
  double thrust_command = 0;

  int counter = 0;

  for (;;) {

    if (counter < euler_steps) {

      // Get system state
      quad.sensor_read();

      // Send system state
      z_measured = quad.z_mes();

      // printf("Sent: %f\n", z_measured);
      spi_master_transceive(spi, &spi_cfg, &z_measured, &thrust_command);
      printf("%f\n", z_measured);

      // Apply control input and compute the change
      quad.dynamics(thrust_command, 0.0);
      quad.euler_step(dt);

      // Sleep between cycles
      k_sleep(K_TIMEOUT_ABS_MS(dt));

      // Increment counter variable
      counter++;
    }

    else
      k_cpu_idle();
  }
}
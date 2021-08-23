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
  constexpr static int euler_steps = 100;

  // feedforward thrust = - g
  float ff_thrust = 9.81;

  double tx_data = 4;
  double rx_data = 0;

  int counter = 0;

  for (;;) {

    if (counter < euler_steps) {

      // Get system state
      quad.sensor_read();

      // Send system state
      tx_data = quad.z_mes();

      // printf("Sent: %f\n", tx_data);
      spi_master_transceive(spi, &spi_cfg, &tx_data, &rx_data);
      printf("%f\n", tx_data);

      // Compute control input
      float altitude_error = altitude_target - quad.z_mes();

      // Apply control input and compute the change
      quad.dynamics(thrust_command, 0.0);
      quad.euler_step(dt);

      // Sleep between cycles
      k_sleep(K_TIMEOUT_ABS_MS(altitude_dt));

      // Increment counter variable
      counter++;
    }

    else
      k_cpu_idle();
  }
}
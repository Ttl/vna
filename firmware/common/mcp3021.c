#include "mcp3021.h"

uint16_t mcp3021_read_result()
{
    uint16_t result;
	i2c_tx_start(I2C1_BASE);
	i2c_tx_byte(I2C1_BASE, (MCP3021_ADDR << 1) | I2C_READ);

    result = i2c1_rx_byte(1) << 8;
    result |= i2c1_rx_byte(0);

	i2c_stop(I2C1_BASE);
	return result;
}

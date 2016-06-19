#ifndef _MCP3021_H_
#define __MCP3021_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <libopencm3/lpc43xx/i2c.h>
#include <stdint.h>
#include "i2c_lpc.h"

#define MCP3021_ADDR 0x4d

uint16_t mcp3021_read_result();

#endif

/*
 * Copyright 2012-2016 Benjamin Vernoux <bvernoux@airspy.com>
 *
 * This file is part of AirSpy.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __M0S_BIN_H
#define __M0S_BIN_H

#ifdef __cplusplus
extern "C"
{
#endif

extern uint8_t m0s_bin[];
extern uint32_t m0s_bin_size;
extern uint32_t cm0s_exec_baseaddr; /* defined in linker script */

#ifdef __cplusplus
}
#endif

#endif /* __M0S_BIN_H */

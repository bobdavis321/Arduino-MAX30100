/*
Arduino-MAX30102 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAX30102_REGISTERS_H
#define MAX30102_REGISTERS_H

#define MAX30102_I2C_ADDRESS                    0x57

// Interrupt status register (RO)
#define MAX30102_REG_INTERRUPT_STATUS           0x00
#define MAX30102_IS_PWR_RDY                     (1 << 0)
#define MAX30102_IS_SPO2_RDY                    (1 << 4)
#define MAX30102_IS_HR_RDY                      (1 << 5)
#define MAX30102_IS_TEMP_RDY                    (1 << 6)
#define MAX30102_IS_A_FULL                      (1 << 7)

// Interrupt enable register
#define MAX30102_REG_INTERRUPT_ENABLE           0x02
#define MAX30102_IE_ENB_SPO2_RDY                (1 << 4)
#define MAX30102_IE_ENB_HR_RDY                  (1 << 5)
#define MAX30102_IE_ENB_TEMP_RDY                (1 << 6)
#define MAX30102_IE_ENB_A_FULL                  (1 << 7)

// FIFO control and data registers
#define MAX30102_REG_FIFO_WRITE_POINTER         0x04
#define MAX30102_REG_FIFO_OVERFLOW_COUNTER      0x05
#define MAX30102_REG_FIFO_READ_POINTER          0x06
#define MAX30102_REG_FIFO_DATA                  0x07  // Burst read does not autoincrement addr

// Mode Configuration register
#define MAX30102_REG_MODE_CONFIGURATION         0x09
#define MAX30102_MC_TEMP_EN                     (1 << 3)
#define MAX30102_MC_RESET                       (1 << 6)
#define MAX30102_MC_SHDN                        (1 << 7)
typedef enum Mode {
    MAX30102_MODE_HRONLY    = 0x02,
    MAX30102_MODE_SPO2_HR   = 0x03
} Mode;

// SpO2 Configuration register
// Check tables 8 and 9, p19 of the MAX30102 datasheet to see the permissible
// combinations of sampling rates and pulse widths
#define MAX30102_REG_SPO2_CONFIGURATION         0x0A
#define MAX30102_SPC_SPO2_HI_RES_EN             (1 << 6)
typedef enum SamplingRate {
    MAX30102_SAMPRATE_50HZ      = 0x00,
	MAX30102_SAMPRATE_100HZ     = 0x01,
	MAX30102_SAMPRATE_167HZ     = 0x02,
	MAX30102_SAMPRATE_200HZ     = 0x03,
	MAX30102_SAMPRATE_400HZ     = 0x04,
	MAX30102_SAMPRATE_600HZ     = 0x05,
	MAX30102_SAMPRATE_800HZ     = 0x06,
	MAX30102_SAMPRATE_1000HZ    = 0x07
} SamplingRate;

typedef enum LEDPulseWidth {
    MAX30102_SPC_PW_200US_13BITS    = 0x00,
    MAX30102_SPC_PW_400US_14BITS    = 0x01,
    MAX30102_SPC_PW_800US_15BITS    = 0x02,
    MAX30102_SPC_PW_1600US_16BITS   = 0x03
} LEDPulseWidth;

// LED Configuration register
#define MAX30102_REG_LED_CONFIGURATION          0x0D
#define MAX30102_REG_LED_CONFIGURATION2         0x0C
typedef enum LEDCurrent {
	MAX30102_LED_CURR_0MA      = 0x00,
	MAX30102_LED_CURR_4_4MA    = 0x10,
	MAX30102_LED_CURR_7_6MA    = 0x20,
	MAX30102_LED_CURR_11MA     = 0x30,
	MAX30102_LED_CURR_14_2MA   = 0x40,
	MAX30102_LED_CURR_17_4MA   = 0x50,
	MAX30102_LED_CURR_20_8MA   = 0x60,
	MAX30102_LED_CURR_24MA     = 0x70,
	MAX30102_LED_CURR_27_1MA   = 0x80,
	MAX30102_LED_CURR_30_6MA   = 0x90,
	MAX30102_LED_CURR_33_8MA   = 0xa0,
	MAX30102_LED_CURR_37MA     = 0xb0,
	MAX30102_LED_CURR_40_2MA   = 0xc0,
	MAX30102_LED_CURR_43_6MA   = 0xd0,
	MAX30102_LED_CURR_46_8MA   = 0xe0,
	MAX30102_LED_CURR_50MA     = 0xf0
} LEDCurrent;

// Temperature integer part register
#define MAX30102_REG_TEMPERATURE_DATA_INT       0x1F
// Temperature fractional part register
#define MAX30102_REG_TEMPERATURE_DATA_FRAC      0x20

// Revision ID register (RO)
#define MAX30102_REG_REVISION_ID                0xfe
// Part ID register
#define MAX30102_REG_PART_ID                    0xff

#define MAX30102_FIFO_DEPTH                     0x10

#endif

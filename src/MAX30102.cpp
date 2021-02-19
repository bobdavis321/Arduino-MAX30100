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

#include <Wire.h>

#include "MAX30102.h"

MAX30102::MAX30102()
{
}

bool MAX30102::begin()
{
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    if (getPartId() != EXPECTED_PART_ID) {
        return false;
    }

    setMode(DEFAULT_MODE);
    setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    setSamplingRate(DEFAULT_SAMPLING_RATE);
    setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    setHighresModeEnabled(true);

    return true;
}

void MAX30102::setMode(Mode mode)
{
    writeRegister(MAX30102_REG_MODE_CONFIGURATION, mode);
}

void MAX30102::setLedsPulseWidth(LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30102_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30102::setSamplingRate(SamplingRate samplingRate)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30102_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30102::setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    writeRegister(MAX30102_REG_LED_CONFIGURATION, redLedCurrent);
    writeRegister(MAX30102_REG_LED_CONFIGURATION2, irLedCurrent);
}

void MAX30102::setHighresModeEnabled(bool enabled)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    if (enabled) {
        writeRegister(MAX30102_REG_SPO2_CONFIGURATION, previous | MAX30102_SPC_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30102_REG_SPO2_CONFIGURATION, previous & ~MAX30102_SPC_SPO2_HI_RES_EN);
    }
}

void MAX30102::update()
{
    readFifoData();
}

bool MAX30102::getRawValues(uint16_t *ir, uint16_t *red)
{
    if (!readoutsBuffer.isEmpty()) {
        SensorReadout readout = readoutsBuffer.pop();

        *ir = readout.ir;
        *red = readout.red;

        return true;
    } else {
        return false;
    }
}

void MAX30102::resetFifo()
{
    writeRegister(MAX30102_REG_FIFO_WRITE_POINTER, 0);
    writeRegister(MAX30102_REG_FIFO_READ_POINTER, 0);
    writeRegister(MAX30102_REG_FIFO_OVERFLOW_COUNTER, 0);
}

uint8_t MAX30102::readRegister(uint8_t address)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom(MAX30102_I2C_ADDRESS, 1);

    return Wire.read();
}

void MAX30102::writeRegister(uint8_t address, uint8_t data)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

void MAX30102::burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(baseAddress);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MAX30102_I2C_ADDRESS, length);

    uint8_t idx = 0;
    while (Wire.available()) {
        buffer[idx++] = Wire.read();
    }
}

void MAX30102::readFifoData()
{
    uint8_t buffer[MAX30102_FIFO_DEPTH*4];
    uint8_t toRead;

    toRead = (readRegister(MAX30102_REG_FIFO_WRITE_POINTER) - readRegister(MAX30102_REG_FIFO_READ_POINTER)) & (MAX30102_FIFO_DEPTH-1);

    if (toRead) {
        burstRead(MAX30102_REG_FIFO_DATA, buffer, 4 * toRead);

        for (uint8_t i=0 ; i < toRead ; ++i) {
            // Warning: the values are always left-aligned
            readoutsBuffer.push({
                    .ir=(uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]),
                    .red=(uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3])});
        }
    }
}

void MAX30102::startTemperatureSampling()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30102_MC_TEMP_EN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

bool MAX30102::isTemperatureReady()
{
    return !(readRegister(MAX30102_REG_MODE_CONFIGURATION) & MAX30102_MC_TEMP_EN);
}

float MAX30102::retrieveTemperature()
{
    int8_t tempInteger = readRegister(MAX30102_REG_TEMPERATURE_DATA_INT);
    float tempFrac = readRegister(MAX30102_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}

void MAX30102::shutdown()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30102_MC_SHDN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30102::resume()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30102_MC_SHDN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

uint8_t MAX30102::getPartId()
{
    return readRegister(0xff);
}

/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* Modified by Shalaj Lawania 2014
Modified further by Andrea Corrado 2015
    Modifications: Added calibration(), get_values(), readX(), readY(), readZ(), readXoffset(), readYoffset(), readZoffset(), get_data() methods */

#include "accelerometer/fxos8700q.h"
#define UINT14_MAX        16383


FXOS8700Q_acc::FXOS8700Q_acc(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    // activate the peripheral
    uint8_t data[2] = {FXOS8700Q_CTRL_REG1, 0x00};
    m_i2c.frequency(400000);
    writeRegs(data, 2);
    data[0] = FXOS8700Q_M_CTRL_REG1;
    data[1] = 0x1F;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_M_CTRL_REG2;
    data[1] = 0x20;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_XYZ_DATA_CFG;
    data[1] = 0x00;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_CTRL_REG1;
    data[1] = 0x18;//0x1D;
    writeRegs(data, 2);

    sumX = 0;
    sumY = 0;
    sumZ = 0;
    calibration_values = 0;
    xoffset = 0;
    yoffset = 0;
    zoffset = 0;
    c1 = 0;
    d1 = 0;
    t1.start();
    accelerometer_motion=0;
}

FXOS8700Q_acc::~FXOS8700Q_acc() { }

void FXOS8700Q_acc::enable(void) {
    uint8_t data[2];
    readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
    data[1] |= 0x01;
    data[0] = FXOS8700Q_CTRL_REG1;
    writeRegs(data, 2);
}

void FXOS8700Q_acc::disable(void) {
    uint8_t data[2];
    readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
    data[1] &= 0xFE;
    data[0] = FXOS8700Q_CTRL_REG1;
    writeRegs(data, 2);
}



uint32_t FXOS8700Q_acc::whoAmI() {
    uint8_t who_am_i = 0;
    readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
    return (uint32_t) who_am_i;
}

uint32_t FXOS8700Q_acc::dataReady(void) {
    uint8_t stat = 0;
    readRegs(FXOS8700Q_STATUS, &stat, 1);
    return (uint32_t) stat;
}

uint32_t FXOS8700Q_acc::sampleRate(uint32_t f) {
    return(50); // for now sample rate is fixed at 50Hz
}

void FXOS8700Q_acc::getX(float * x) {
    *x = (float(getAccAxis(FXOS8700Q_OUT_X_MSB))/4096.0f);
}

void FXOS8700Q_acc::getY(float * y) {
    *y = (float(getAccAxis(FXOS8700Q_OUT_Y_MSB))/4096.0f);
}

void FXOS8700Q_acc::getZ(float * z) {
    *z = (float(getAccAxis(FXOS8700Q_OUT_Z_MSB))/4096.0f);
}

void FXOS8700Q_acc::getX(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_OUT_X_MSB);
}

void FXOS8700Q_acc::getY(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_OUT_Y_MSB);
}

void FXOS8700Q_acc::getZ(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_OUT_Z_MSB);
}

void FXOS8700Q_acc::calibration() {

    if (calibration_values < 20 && c1 == 0) {
        printf("\n\r Calibrating... ");
        get_data(&Xarray[calibration_values], &Yarray[calibration_values], &Zarray[calibration_values]);
        //pc.printf("\n\r For j = %d, X = %1.4f, Y = %1.4f, Z = %1.4f", j, Xarray[j], Yarray[j], Zarray[j]);
        calibration_values++;
    }

    if (calibration_values >= 20 && c1 == 0) {
        for (int i = 0; i < 20; i++) {
            //printf("\n\r For j = %d, X = %1.4f, Y = %1.4f, Z = %1.4f", i, Xarray[i], Yarray[i], Zarray[i]);
            sumX = sumX + Xarray[i];
            sumY = sumY + Yarray[i];
            sumZ = sumZ + Zarray[i];
        }
        xoffset = sumX/20;
        yoffset = sumY/20;
        zoffset = sumZ/20;
        //printf("\n\r Offset: X = %1.4f, Y = %1.4f, Z = %1.4f", xoffset, yoffset, zoffset);
        c1 = 1;
    }
}

int FXOS8700Q_acc::get_values()
{
    calibration();
    if (calibration_values == 20) {
        //printf("\n\r Offset: X = %1.4f, Y = %1.4f, Z = %1.4f", xoffset, yoffset, zoffset);
        get_data(&X, &Y, &Z);
        X = X - xoffset;
        Y = Y - yoffset;
        Z = Z - zoffset;
        sumX = 0;
        sumY = 0;
        sumZ = 0;

        if (((Z > 1) || (Z < -1) || (X > 1) || (X < -1) || (Y > 1.2) || (Y < -1.2)) && (t1.read_ms() > 2000)) {
            //printf("\n\r Detected: Hammer Accelerometer: X=%1.4f Y=%1.4f Z=%1.4f  \n", X, Y, Z);
            t1.stop();
            //printf("\n\r Time: %d", t1.read_ms());
            t1.reset();
            accelerometer_motion=1;
            t1.start();
        } else if (t1.read() > 10) {
            //pc.printf("\n\r Hammer Accelerometer: X=%1.4f Y=%1.4f Z=%1.4f  \n", X, Y, Z);
            accelerometer_motion=0;
        }
    }
    return accelerometer_motion;
}

float FXOS8700Q_acc::readXoffset() {
    return xoffset;
    }

float FXOS8700Q_acc::readYoffset() {
    return yoffset;
    }

float FXOS8700Q_acc::readZoffset(){
    return zoffset;
    }

float FXOS8700Q_acc::readX(){
    return X;
    }

float FXOS8700Q_acc::readY(){
    return Y;
    }

float FXOS8700Q_acc::readZ(){
    return Z;
    }

void FXOS8700Q_acc::get_data(float *x, float *y, float *z)
{
    getX(x);
    getY(y);
    getZ(z);
    *x = *x * 100;
    *y = *y * 100;
    *z = *z * 100;
}

void FXOS8700Q_acc::getAxis(MotionSensorDataUnits &data) {
    int16_t acc, t[3];
    uint8_t res[6];
   readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    t[0] = acc;
    acc = (res[2] << 6) | (res[3] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    t[1] = acc;
    acc = (res[4] << 6) | (res[5] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    t[2] = acc;
    data.x = ((float) t[0]) / 4096.0f;
    data.y = ((float) t[1]) / 4096.0f;
    data.z = ((float) t[2]) / 4096.0f;
}


void FXOS8700Q_acc::getAxis(MotionSensorDataCounts &data) {
    int16_t acc;
    uint8_t res[6];
    readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    data.x = acc;
    acc = (res[2] << 6) | (res[3] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    data.y = acc;
    acc = (res[4] << 6) | (res[5] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
    data.z = acc;
}

void FXOS8700Q_acc::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}

void FXOS8700Q_acc::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}


int16_t FXOS8700Q_acc::getAccAxis(uint8_t addr) {
    int16_t acc;
    uint8_t res[2];
    readRegs(addr, res, 2);

    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;

    return acc;
}



FXOS8700Q_mag::FXOS8700Q_mag(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    // activate the peripheral
    uint8_t data[2] = {FXOS8700Q_CTRL_REG1, 0x00};
    m_i2c.frequency(400000);
    writeRegs(data, 2);
    data[0] = FXOS8700Q_M_CTRL_REG1;
    data[1] = 0x1F;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_M_CTRL_REG2;
    data[1] = 0x20;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_XYZ_DATA_CFG;
    data[1] = 0x00;
    writeRegs(data, 2);
    data[0] = FXOS8700Q_CTRL_REG1;
    data[1] = 0x18;//0x1D;
    writeRegs(data, 2);
}

FXOS8700Q_mag::~FXOS8700Q_mag() { }

void FXOS8700Q_mag::enable(void) {
    uint8_t data[2];
    readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
    data[1] |= 0x01;
    data[0] = FXOS8700Q_CTRL_REG1;
    writeRegs(data, 2);
}

void FXOS8700Q_mag::disable(void) {
    uint8_t data[2];
    readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
    data[1] &= 0xFE;
    data[0] = FXOS8700Q_CTRL_REG1;
    writeRegs(data, 2);
}



uint32_t FXOS8700Q_mag::whoAmI() {
    uint8_t who_am_i = 0;
    readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
    return (uint32_t) who_am_i;
}

uint32_t FXOS8700Q_mag::dataReady(void) {
    uint8_t stat = 0;
    readRegs(FXOS8700Q_STATUS, &stat, 1);
    return (uint32_t) stat;
}

uint32_t FXOS8700Q_mag::sampleRate(uint32_t f) {
    return(50); // for now sample rate is fixed at 50Hz
}

void FXOS8700Q_mag::getX(float * x) {
    *x = (float(getAccAxis(FXOS8700Q_M_OUT_X_MSB)) * 0.1f);
}

void FXOS8700Q_mag::getY(float * y) {
    *y = (float(getAccAxis(FXOS8700Q_M_OUT_Y_MSB)) * 0.1f);
}

void FXOS8700Q_mag::getZ(float * z) {
    *z = (float(getAccAxis(FXOS8700Q_M_OUT_Z_MSB)) * 0.1f);
}

void FXOS8700Q_mag::getX(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_M_OUT_X_MSB);
}

void FXOS8700Q_mag::getY(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_M_OUT_Y_MSB);
}

void FXOS8700Q_mag::getZ(int16_t * d) {
    *d = getAccAxis(FXOS8700Q_M_OUT_Z_MSB);
}


void FXOS8700Q_mag::getAxis(MotionSensorDataUnits &data) {
    int16_t t[3];
    uint8_t res[6];
   readRegs(FXOS8700Q_M_OUT_X_MSB, res, 6);

    t[0] = (res[0] << 8) | res[1];
    t[1] = (res[2] << 8) | res[3];
    t[2] = (res[4] << 8) | res[5];

    data.x = ((float) t[0]) * 0.1f;
    data.y = ((float) t[1]) * 0.1f;
    data.z = ((float) t[2]) * 0.1f;
}


void FXOS8700Q_mag::getAxis(MotionSensorDataCounts &data) {
    int16_t acc;
    uint8_t res[6];
    readRegs(FXOS8700Q_M_OUT_X_MSB, res, 6);

    data.x = (res[0] << 8) | res[1];
    data.y = (res[2] << 8) | res[3];
    data.z = (res[4] << 8) | res[5];
}

void FXOS8700Q_mag::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}

void FXOS8700Q_mag::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}


int16_t FXOS8700Q_mag::getAccAxis(uint8_t addr) {
    int16_t acc;
    uint8_t res[2];
    readRegs(addr, res, 2);

    acc = (res[0] << 8) | res[1];

    return acc;
}

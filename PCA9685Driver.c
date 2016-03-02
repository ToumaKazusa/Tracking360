#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include "PCA9685Driver.h"
#include <math.h>

#define DEBUGON 1

typedef enum
{
    tilt_ch = 0,
    pan_ch = 1,
} Channels;

// 1ms == -90
// 1.5 ms == 0
// 2ms == +90
// 20 ms == period
// 1 ms = x/20ms
// 1.5 ms = x / 20
// (4096 * 1) / 20 = 204
// (4096 * 1.5) / 20  = 307
// (4096 * 2) / 20    = 409 
#define SG90FRQ       50
#define SG90PRD       20
#define SG90PERIOD0   1
#define SG90PERIOD90  1.5
#define SG90PERIOD180 2
#define SG90RNGMOTION 180

#define PCACTRLREF 4096.0


void MotorDescriptior_Init(struct MotorDescriptiorType* self)
{
    self->period_min = SG90PERIOD0;
    self->period_mid = SG90PERIOD90;
    self->period_max = SG90PERIOD180;
    self->PWMFrq     = SG90FRQ;
    self->PWMPrd     = SG90PRD;
    self->RangeOfMotion = SG90RNGMOTION;
}

void PCA9685_SoftwareReset(struct PCA9685DriverType* driver)
{ 
    driver->errono = wiringPiI2CWrite(driver->fd, 0x06);
    if(driver->errono == -1)
    {
        printf("failed at %d in %s\n", __LINE__, __FUNCTION__);
    }
}

int PCA9685_SetAllPMW(struct PCA9685DriverType* driver, int on, int off)
{
    driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_ON_L, on & 0xFF);
    if(driver->errono == -1)
    {
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
        return driver->errono;
    }

    driver->errono =  wiringPiI2CWriteReg8(driver->fd, LED0_ON_H , on >> 8);
    if(driver->errono == -1)
    {
        printf("failed at %d\n", __LINE__);
        return driver->errono;
    }

    driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_OFF_L, off & 0xFF);
    if(driver->errono == -1)
    {
        printf("failed at %d\n", __LINE__);
        return driver->errono;
    }

    driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_OFF_H, off >> 8);
    if(driver->errono == -1)
    {
        printf("failed at %d\n", __LINE__);
        return driver->errono;
    }

    if(driver->bDebug)
        printf("succeed %s\n", __FUNCTION__);
    return driver->errono;
}

void PCA9685_Init(struct PCA9685DriverType* driver)
{
    int mode1 = 0;
    driver->deviceId = 0x40;
    driver->bDebug = DEBUGON;
    driver->errono = 0;
    driver->controlRefinements = PCACTRLREF;

    //driver->errono = wiringPiSetup();
    //if(driver->errono == -1)
    //    printf("wiringPi Setup failed\n");
    driver->fd = wiringPiI2CSetup(driver->deviceId);

    PCA9685_SetAllPMW(driver, 0, 0);

    driver->errono = wiringPiI2CWriteReg8(driver->fd, MODE2, OUTDRV);
    if(driver->errono == -1)
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
    wiringPiI2CWriteReg8(driver->fd, MODE1, ALLCALL);
    usleep(5000);

    mode1 = wiringPiI2CReadReg8(driver->fd, MODE1);
    if(DEBUGON)
        printf("mode1 read == %d at %d in %s\n", mode1, __LINE__, __FUNCTION__);
    if(mode1 == -1)
        printf("read failed %s\n", __FUNCTION__);
    else
    {
        mode1 = mode1 & ~SLEEP;
        wiringPiI2CWriteReg8(driver->fd, MODE1, mode1);
        usleep(5000);
    }
}

int PC9865_SetPWMFreq(struct PCA9685DriverType* driver, int frq) 
{
    float prescaleval = 25000000.0;
    prescaleval /= driver->controlRefinements;
    prescaleval /= frq;
    prescaleval -= 1.0;

    driver->errono = 0;

    if (driver->bDebug)
    {
        printf("Setting PWM frequency to %d Hz\n", frq);
        printf("Estimated pre-scale: %f\n", prescaleval);
    }

    {
    double prescale = floor(prescaleval + 0.5);
    int oldmode, newmode;
        if (driver->bDebug)
            printf("Final pre-scale: %f\n", prescale);
        oldmode = wiringPiI2CReadReg8(driver->fd, MODE1);
        newmode = (oldmode & 0x7F) | 0x10;
        if((driver->errono = wiringPiI2CWriteReg8(driver->fd, MODE1, newmode)) == -1)
        {
            printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
            return driver->errono;
        }

        if((driver->errono = wiringPiI2CWriteReg8(driver->fd,PRESCALE, (int) floor(prescale))) == -1)
        {
            printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
            return driver->errono;
        }

        if((driver->errono = wiringPiI2CWriteReg8(driver->fd, MODE1, oldmode)) == -1)
        {
            printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
            return driver->errono;
        }

        usleep(5000);

        if((driver->errono = wiringPiI2CWriteReg8(driver->fd, MODE1, oldmode | 0x80)) == -1)
        {
            printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
            return driver->errono;
        }
    } 

    return driver->errono;
}
int PC9865_SetPWM(struct PCA9685DriverType* driver, int channel, int on, int off) 
{
    driver->errono = 0;
    if((driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_ON_L+4*channel, on & 0xFF)) == -1)
    {
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
        return driver->errono;
    }
    if((driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_ON_H+4*channel, on >> 8)) == -1) 
    {
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
        return driver->errono;
    }
    if((driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_OFF_L+4*channel, off & 0xFF)) == -1)  
    {
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
        return driver->errono;
    }
    if((driver->errono = wiringPiI2CWriteReg8(driver->fd, LED0_OFF_H+4*channel, off >> 8)) == -1)  
    {
        printf("i2c write failed at %d in %s\n", __LINE__, __FUNCTION__);
        return driver->errono;
    }
    return driver->errono;
}

int PCA9685_ComputeTicks(struct PCA9685DriverType* driver, int pulseWidth, int period)
{
    // (4096 * 1) / 20 = 204
    return (int) ((float) (driver->controlRefinements * pulseWidth) / period);
}

void Track360MtrCtrl_Init(struct Track360MotorControlType* self)
{
    MotorDescriptior_Init(&self->motor);
    PCA9685_Init(&self->pca9685);
    PC9865_SetPWMFreq(&self->pca9685, self->motor.PWMFrq);
    self->bDebug = DEBUGON;
    self->MinPWTck = PCA9685_ComputeTicks(&self->pca9685, self->motor.period_min, self->motor.PWMPrd);

    if(self->bDebug)
        printf("MinPWtc = %d\n", self->MinPWTck);
    self->PWTckPerDeg = PCA9685_ComputeTicks(&self->pca9685, self->motor.period_max, self->motor.PWMPrd) - self->MinPWTck;
    self->PWTckPerDeg /= self->motor.RangeOfMotion; 

    if(self->bDebug)
        printf("PWTckPerDeg= %f\n", self->PWTckPerDeg);
    Track360MtrCtrl_Tilt(self, self->motor.RangeOfMotion / 2);
    Track360MtrCtrl_Pan(self, self->motor.RangeOfMotion / 2);
}

int Track360MtrCtrl_DegToTck(struct Track360MotorControlType* self, int degree)
{
    int tick = (int) (self->PWTckPerDeg* degree) + self->MinPWTck;
    if(self->bDebug)
        printf("value written = %d\n", tick);
    return tick;
}

void Track360MtrCtrl_Tilt(struct Track360MotorControlType* self, int degree)
{
    PC9865_SetPWM(&self->pca9685, tilt_ch, 0, 
            Track360MtrCtrl_DegToTck(self, degree));
    self->curPosTilt = degree;
}

void Track360MtrCtrl_Pan(struct Track360MotorControlType* self, int degree)
{
    PC9865_SetPWM(&self->pca9685, pan_ch, 0, 
            Track360MtrCtrl_DegToTck(self, degree));
    self->curPosPan = degree;
}

void Track360MtrCtrl_TiltUpFrmCurPos(struct Track360MotorControlType* self, int degree, VDirection direction)
{
    if(direction == up)
        self->curPosTilt -= degree;
    else
        self->curPosTilt += degree;

    if(self->curPosTilt < 0)
        self->curPosTilt = 0;

    if(self->curPosTilt > self->motor.RangeOfMotion)
        self->curPosTilt = self->motor.RangeOfMotion;
    Track360MtrCtrl_Tilt(self, self->curPosTilt);
}

void Track360MtrCtrl_PanFrmCurPos(struct Track360MotorControlType* self, int degree, HDirection direction)
{
    if(direction == left)
        self->curPosPan -= degree;
    else
        self->curPosPan += degree;

    if(self->curPosPan < 0)
        self->curPosPan = 0;

    if(self->curPosPan > self->motor.RangeOfMotion)
        self->curPosPan = self->motor.RangeOfMotion;
    Track360MtrCtrl_Pan(self, self->curPosPan);
}

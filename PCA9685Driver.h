#ifdef __cplusplus
extern "C"{
#endif
#ifndef PCA9685_H
#define PCA9685_H
typedef enum
{
    MODE1              = 0x00,
    MODE2              = 0x01,
    SUBADR1            = 0x02,
    SUBADR2            = 0x03,
    SUBADR3            = 0x04,
    PRESCALE           = 0xFE,
    LED0_ON_L          = 0x06,
    LED0_ON_H          = 0x07,
    LED0_OFF_L         = 0x08,
    LED0_OFF_H         = 0x09,
    ALL_LED_ON_L       = 0xFA,
    ALL_LED_ON_H       = 0xFB,
    ALL_LED_OFF_L      = 0xFC,
    ALL_LED_OFF_H      = 0xFD,
} Registers;

typedef enum
{
    RESTART            = 0x80,
    SLEEP              = 0x10,
    ALLCALL            = 0x01,
    INVRT              = 0x10,
    OUTDRV             = 0x04,
} Commands;

typedef enum
{
    left,
    right,
} HDirection;

typedef enum
{
    up,
    down,
} VDirection;

struct PCA9685DriverType
{
    int deviceId;
    int bDebug;
    int errono;
    int fd;
    float controlRefinements;
};

struct MotorDescriptiorType
{
    float period_min;
    float period_mid;
    float period_max;
    float PWMFrq;
    int   PWMPrd;
    int   RangeOfMotion;
};

struct Track360MotorControlType
{
    struct MotorDescriptiorType motor;
    struct PCA9685DriverType pca9685;
    int MinPWTck;
    float PWTckPerDeg;
    int bDebug;
    int curPosTilt;
    int curPosPan;
};

void Track360MtrCtrl_Init(struct Track360MotorControlType* self);

void Track360MtrCtrl_Tilt(struct Track360MotorControlType* self, int degree);

void Track360MtrCtrl_Pan(struct Track360MotorControlType* self, int degree);

void Track360MtrCtrl_TiltFrmCurPos(struct Track360MotorControlType* self, int degree, VDirection direction);

void Track360MtrCtrl_PanFrmCurPos(struct Track360MotorControlType* self, int degree, HDirection direction);
#endif
#ifdef __cplusplus
}
#endif

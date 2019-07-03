
#ifndef _LEDS_PCA968X_H
#define _LEDS_PCA968X_H


/*  ---   PCA9685 DEFINITION   ---  */

#define PCA9685_OSC_INTERNAL                25000000    // 25MHz

#define PCA9685_REG_NUMBER                  76

#define PCA9685_REG_MODE1                   0x00
#define PCA9685_REG_MODE2                   0x01

#define PCA9685_REG_SUBADR1                 0x02
#define PCA9685_REG_SUBADR2                 0x03
#define PCA9685_REG_SUBADR3                 0x04

#define PCA9685_REG_ALLCALLADR              0x05

#define PCA9685_REG_LED0_ON_L               0x06
#define PCA9685_REG_LDE0_ON_H               0x07
#define PCA9685_REG_LED0_OFF_L              0x08
#define PCA9685_REG_LDE0_OFF_H              0x09

#define PCA9685_REG_LED1_ON_L               0x0A
#define PCA9685_REG_LDE1_ON_H               0x0B
#define PCA9685_REG_LED1_OFF_L              0x0C
#define PCA9685_REG_LDE1_OFF_H              0x0D

#define PCA9685_REG_LED2_ON_L               0x0E
#define PCA9685_REG_LDE2_ON_H               0x0F
#define PCA9685_REG_LED2_OFF_L              0x10
#define PCA9685_REG_LDE2_OFF_H              0x11

#define PCA9685_REG_LED3_ON_L               0x12
#define PCA9685_REG_LDE3_ON_H               0x13
#define PCA9685_REG_LED3_OFF_L              0x14
#define PCA9685_REG_LDE3_OFF_H              0x15

#define PCA9685_REG_LED4_ON_L               0x16
#define PCA9685_REG_LDE4_ON_H               0x17
#define PCA9685_REG_LED4_OFF_L              0x18
#define PCA9685_REG_LDE4_OFF_H              0x19

#define PCA9685_REG_LED5_ON_L               0x1A
#define PCA9685_REG_LDE5_ON_H               0x1B
#define PCA9685_REG_LED5_OFF_L              0x2C
#define PCA9685_REG_LDE5_OFF_H              0x1D

#define PCA9685_REG_LED6_ON_L               0x1E
#define PCA9685_REG_LDE6_ON_H               0x1F
#define PCA9685_REG_LED6_OFF_L              0x20
#define PCA9685_REG_LDE6_OFF_H              0x21

#define PCA9685_REG_LED7_ON_L               0x22
#define PCA9685_REG_LDE7_ON_H               0x23
#define PCA9685_REG_LED7_OFF_L              0x24
#define PCA9685_REG_LDE7_OFF_H              0x25

#define PCA9685_REG_LED8_ON_L               0x26
#define PCA9685_REG_LDE8_ON_H               0x27
#define PCA9685_REG_LED8_OFF_L              0x28
#define PCA9685_REG_LDE8_OFF_H              0x29

#define PCA9685_REG_LED9_ON_L               0x2A
#define PCA9685_REG_LDE9_ON_H               0x2B
#define PCA9685_REG_LED9_OFF_L              0x2C
#define PCA9685_REG_LDE9_OFF_H              0x2D

#define PCA9685_REG_LED10_ON_L              0x2E
#define PCA9685_REG_LDE10_ON_H              0x2F
#define PCA9685_REG_LED10_OFF_L             0x30
#define PCA9685_REG_LDE10_OFF_H             0x31

#define PCA9685_REG_LED11_ON_L              0x32
#define PCA9685_REG_LDE11_ON_H              0x33
#define PCA9685_REG_LED11_OFF_L             0x34
#define PCA9685_REG_LDE11_OFF_H             0x35

#define PCA9685_REG_LED12_ON_L              0x36
#define PCA9685_REG_LDE12_ON_H              0x37
#define PCA9685_REG_LED12_OFF_L             0x38
#define PCA9685_REG_LDE12_OFF_H             0x39

#define PCA9685_REG_LED13_ON_L              0x3A
#define PCA9685_REG_LDE13_ON_H              0x3B
#define PCA9685_REG_LED13_OFF_L             0x3C
#define PCA9685_REG_LDE13_OFF_H             0x3D

#define PCA9685_REG_LED14_ON_L              0x3E
#define PCA9685_REG_LDE14_ON_H              0x3F
#define PCA9685_REG_LED14_OFF_L             0x40
#define PCA9685_REG_LDE14_OFF_H             0x41

#define PCA9685_REG_LED15_ON_L              0x42
#define PCA9685_REG_LDE15_ON_H              0x43
#define PCA9685_REG_LED15_OFF_L             0x44
#define PCA9685_REG_LDE15_OFF_H             0x45

#define PCA9685_REG_ALL_LED_ON_L            0xFA
#define PCA9685_REG_ALL_LED_ON_H            0xFB
#define PCA9685_REG_ALL_LED_OFF_L           0xFC
#define PCA9685_REG_ALL_LED_OFF_H           0xFD

#define PCA9685_REG_PRE_SCALE               0xFE
#define PCA9685_REG_TEST_MODE               0xFF




#endif      /*  _LEDS_PCA968X_H  */


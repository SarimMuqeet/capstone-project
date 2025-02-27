//#ifndef PCA9685_H
//#define PCA9685_H
//
//#ifdef __cplusplus
// extern "C" {
//#endif
//
//
//#include <stdbool.h>
//
////needed for flag checking and uints
//#include "stm32h7xx_hal.h"
//
//#ifndef PCA9685_I2C_TIMEOUT
//#define PCA9685_I2C_TIMEOUT 1
//#endif
//
//#define PCA9865_I2C_DEFAULT_DEVICE_ADDRESS 0x80
//
///**
// * Structure defining a handle describing a PCA9685 device.
// */
//typedef struct {
//
//	/**
//	 * The handle to the I2C bus for the device.
//	 */
//	I2C_HandleTypeDef *i2c_handle;
//
//	/**
//	 * The I2C device address.
//	 * @see{PCA9865_I2C_DEFAULT_DEVICE_ADDRESS}
//	 */
//	uint16_t device_address;
//
//	/**
//	 * Set to true to drive inverted.
//	 */
//	bool inverted;
//
//} pca9685_handle_t;
//
///**
// * Initializes a PCA9685 device by resetting registers to known values, setting a PWM frequency of 1000Hz, turning
// * all channels off and waking it up.
// * @param handle Handle to a PCA9685 device.
// * @return True on success, false otherwise.
// */
//bool pca9685_init(pca9685_handle_t *handle);
//
///**
// * Tests if a PCA9685 is sleeping.
// * @param handle Handle to a PCA9685 device.
// * @param sleeping Set to the sleeping state of the device.
// * @return True on success, false otherwise.
// */
//bool pca9685_is_sleeping(pca9685_handle_t *handle, bool *sleeping);
//
///**
// * Puts a PCA9685 device into sleep mode.
// * @param handle Handle to a PCA9685 device.
// * @return True on success, false otherwise.
// */
//bool pca9685_sleep(pca9685_handle_t *handle);
//
///**
// * Wakes a PCA9685 device up from sleep mode.
// * @param handle Handle to a PCA9685 device.
// * @return True on success, false otherwise.
// */
//bool pca9685_wakeup(pca9685_handle_t *handle);
//
///**
// * Sets the PWM frequency of a PCA9685 device for all channels.
// * Asserts that the given frequency is between 24 and 1526 Hertz.
// * @param handle Handle to a PCA9685 device.
// * @param frequency PWM frequency to set in Hertz.
// * @return True on success, false otherwise.
// */
//bool pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency);
//
///**
// * Sets the PWM on and off times for a channel of a PCA9685 device.
// * Asserts that the given channel is between 0 and 15.
// * Asserts that the on and off times are between 0 and 4096.
// * @param handle Handle to a PCA9685 device.
// * @param channel Channel to set the times for.
// * @param on_time PWM on time of the channel.
// * @param off_time PWM off time of the channel.
// * @return True on success, false otherwise.
// */
//bool pca9685_set_channel_pwm_times(pca9685_handle_t *handle, unsigned channel, unsigned on_time, unsigned off_time);
//
///**
// * Helper function to set the PWM duty cycle for a channel of a PCA9685 device. The duty cycle is either directly
// * converted to a 12-bit value used for the PWM timings, if logarithmic is set to false, or an 8-bit value which is then
// * transformed to a 12-bit value using a look up table for the PWM timings.
// * Asserts that the duty cycle is between 0 and 1.
// * @param handle Handle to a PCA9685 device.
// * @param channel Channel to set the duty cycle of.
// * @param duty_cycle Duty cycle to set.
// * @param logarithmic Set to true to apply logarithmic function.
// * @return True on success, false otherwise.
// */
//bool pca9685_set_channel_duty_cycle(pca9685_handle_t *handle, unsigned channel, float duty_cycle, bool logarithmic);
//
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* STM32H7xx_HAL_CONF_H */



/*Other Driver */
/*
 * pca9685.h
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/Servos_PWM_STM32_HAL
 */

#ifndef PCA9685_H_
#define PCA9685_H_

//
//	Enable Servo control mode
//
#define PCA9685_SERVO_MODE

#ifdef PCA9685_SERVO_MODE
//
//	Servo min and max values for TURINGY TG9e Servos
//
#define SERVO_MIN	110
#define SERVO_MAX	500
#define MIN_ANGLE	0.0
#define MAX_ANGLE	180.0
#endif

//
//	Adjustable address 0x80 - 0xFE
//
#define PCA9685_ADDRESS 0x80

//
//	Registers
//
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 		0x0
#define PCA9685_PRESCALE 	0xFE

#define PCA9685_LED0_ON_L 	0x6
#define PCA9685_LED0_ON_H	0x7
#define PCA9685_LED0_OFF_L	0x8
#define PCA9685_LED0_OFF_H 	0x9

#define PCA9685_ALLLED_ON_L 	0xFA
#define PCA9685_ALLLED_ON_H 	0xFB
#define PCA9685_ALLLED_OFF_L 	0xFC
#define PCA9685_ALLLED_OFF_H 	0xFD

#define PCA9685_MODE1_ALCALL_BIT	0
typedef enum
{
	PCA9685_MODE1_SUB1_BIT 	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;

PCA9685_STATUS PCA9685_SoftwareReset(void);
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);

#ifndef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency);
#endif

PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);
#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, float Angle);
#endif

PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c);

#endif /* PCA9685_H_ */

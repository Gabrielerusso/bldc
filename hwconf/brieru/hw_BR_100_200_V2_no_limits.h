/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */
#ifndef HW_BR_100_200_no_limits_H_ 
#define HW_BR_100_200_no_limits_H_ 

#define HW_NAME					"BRIESC_100_200_V2_no_limits"


// HW properties
#define HW_HAS_3_SHUNTS
#define INVERTED_SHUNT_POLARITY
//#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_PHASE_FILTERS
//#define HW_HAS_CURRENT_FILTER

// Macros
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				7


#define LED_GREEN_ON()			false
#define LED_GREEN_OFF()			false
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define PHASE_FILTER_GPIO		GPIOC
#define PHASE_FILTER_PIN		9

#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

#define AUX_GPIO				GPIOC
#define AUX_PIN					12
#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)

#define AUX2_GPIO				GPIOB
#define AUX2_PIN				4
#define AUX2_ON()				palSetPad(AUX2_GPIO, AUX2_PIN)
#define AUX2_OFF()				palClearPad(AUX2_GPIO, AUX2_PIN)

#ifdef HW_HAS_CURRENT_FILTER
#define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
#define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)
#endif

// Shutdown pin
#define HW_SHUTDOWN_GPIO		GPIOD
#define HW_SHUTDOWN_PIN			2
#define HW_SHUTDOWN_IN_GPIO		GPIOB
#define HW_SHUTDOWN_IN_PIN		3
#define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN); \
                                chThdSleepMilliseconds(100)
#define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN); \
                                chThdSleepMilliseconds(10000);
#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button();

#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
								HW_SHUTDOWN_HOLD_ON()

// Internal RC osc
//#define HW_USE_INTERNAL_RC

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1       -> SENS3
 * 1  (2):	IN1		SENS2       
 * 2  (3):	IN2		SENS3       -> SENS1
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2    
 * 8  (3):	IN3		TEMP_MOS    -> UNUSED
 * 9  (1):	IN14	TEMP_MOTOR  -> TEMP_MOS
 * 10 (2):	IN15	ADC_EXT3    -> TEMP_MOS2
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2  -> TEMP_MOTOR
 * 16 (2):  IN9		TEMP_MOS_3  -> UNUSED
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			2
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			0
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			10
#define ADC_IND_TEMP_MOS		9   //8
#define ADC_IND_TEMP_MOS_2		10  //15
//#define ADC_IND_TEMP_MOS_3		10  //16
#define ADC_IND_TEMP_MOTOR		15  //9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.30
#endif
#ifndef VIN_R1
#define VIN_R1					64000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.0005/4)
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		briesc_get_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side

#define NTC_TEMP_MOTOR(beta)	    (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()			    (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3435.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			    (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3435.0) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11


// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// PAS sensor
//#define HW_PAS1_PORT HW_ICU_GPIO
//#define HW_PAS1_PIN HW_ICU_PIN
//#define HW_PAS2_PORT HW_ICU_GPIO
//#define HW_PAS2_PIN HW_ICU_PIN

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		1000.0

// Default motor setting overrides
// Default setting overrides
#define MCCONF_L_MIN_VOLTAGE			36.0	// Minimum input voltage
#define MCCONF_L_MAX_VOLTAGE			85.0	// Maximum input voltage
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#define MCCONF_FOC_F_ZV					30000.0
#define MCCONF_L_MAX_ABS_CURRENT		300.0	// The maximum absolute current above which a fault is generated
#define MCCONF_FOC_SAMPLE_V0_V7			FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL	// Run control loop
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#define MCCONF_L_IN_CURRENT_MIN			-10.0	// Input current limit in Amperes (Lower)
#define MCCONF_FOC_CURRENT_FILTER_CONST	0.8		// Filter constant for the filtered currents
#define MCCONF_FOC_TEMP_COMP			false	// Motor temperature compensation
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP	25.0	// Motor temperature compensation base temperature
#define MCCONF_M_NTC_MOTOR_BETA			3435.0 // Beta value for motor termistor
#define MCCONF_L_LIM_TEMP_FET_START     70      // MOSFET Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_FET_END       85      // MOSFET Temp Cutoff End
#define MCCONF_FOC_SAT_COMP_MODE		SAT_COMP_DISABLED		// Stator saturation compensation mode
#define MCCONF_FOC_CC_DECOUPLING		FOC_CC_DECOUPLING_DISABLED // Current controller decoupling
#define MCCONF_FOC_OBSERVER_TYPE		FOC_OBSERVER_MXLEMMING // Position observer type for FOC
#define MCCONF_FOC_CURRENT_SAMPLE_MODE	FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS
#define MCCONF_FOC_DT_US				0.1 // Microseconds for dead time compensation
#define MCCONF_M_HALL_EXTRA_SAMPLES		2 // Extra samples for filtering when reading hall sensors
#define MCCONF_SI_MOTOR_POLES			30 // Motor pole count
#define MCCONF_SI_GEAR_RATIO			1 // Gear ratio
#define MCCONF_SI_WHEEL_DIAMETER		0.270 // Wheel Diameter
#define MCCONF_BMS_TYPE					BMS_TYPE_NONE
#define MCCONF_MAX_CURRENT_UNBALANCE		300	    // [Amp] More than this unbalance trips the fault (likely a sensor disconnected)
#define MCCONF_MAX_CURRENT_UNBALANCE_RATE	0.5		// Fault if more than 50% of the time the motor is unbalanced

// APP OVERRIDE
#define APPCONF_SHUTDOWN_MODE				SHUTDOWN_MODE_OFF_AFTER_10S
#define APPCONF_CAN_STATUS_RATE_1			100
#define APPCONF_CAN_STATUS_RATE_2			10
#define APPCONF_ADC_HYST					0.05
#define APPCONF_ADC_VOLTAGE_MIN				0.2
#define APPCONF_ADC_VOLTAGE_MAX				3.4
#define APPCONF_ADC_TC_MAX_DIFF				4000.0
#define APPCONF_ADC_UPDATE_RATE_HZ			100
#define APPCONF_PPM_THROTTLE_EXP			-0.25
#define APPCONF_PPM_THROTTLE_EXP_BRAKE		-0.25
#define APPCONF_PPM_THROTTLE_EXP_MODE		THR_EXP_POLY

// Setting limits
#define HW_LIM_CURRENT			-500.0, 500.0
#define HW_LIM_CURRENT_IN		-300.0, 300.0
#define HW_LIM_CURRENT_ABS		0.0, 650.0
#define HW_LIM_VIN				25.0, 100.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.96
#define HW_LIM_TEMP_FET			-30.0, 90.0

// HW-specific functions
float briesc_get_temp(void);
bool hw_sample_shutdown_button(void);

#endif /* HW_BR_100_200_H_ */

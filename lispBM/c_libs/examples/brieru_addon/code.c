/*
	Copyright 2022 Gabriele Russo gabryr96@gmail.com

	This file was made for the BRIESC but could be used on all compatible 
	platforms.

	This file is made for the VESC firmware, made by Benjamin Vedder and 
	released under the GNU GPL.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "st_types.h"
#include "vesc_c_if.h"

HEADER

//  PIN DEFINITION
//	PA13 SWDIO 
//	PA14 SWCLK
//	PC12 AUX1
//	PB06 PPM
#define	REMOTE_CH1_GPIO		GPIOA
#define	REMOTE_CH1_PIN		13
#define	REMOTE_CH2_GPIO		GPIOA
#define	REMOTE_CH2_PIN		14
#define BUZZER_GPIO 		GPIOC
#define BUZZER_PIN			12
#define BRAKE_LIGTH_GPIO	GPIOB
#define BRAKE_LIGTH_PIN		6


static uint8_t limited_mode = 0; //flag for limited mode, if equal to 1 means
// that esc will be in limited mode and go up to 25km/h 500W max
static uint8_t 	alarm = 0;
static uint8_t 	adc1_n = 0;
static uint8_t 	adc2_n = 0;
static uint8_t	emergency_brake_flag = 0;
static uint8_t 	brake_ligth_status = 0;
static uint8_t 	power_mode = 0;
static float	adc1_decoded = 0.0;
static float	adc1_raw = 0.0;
static float	adc2_decoded = 0.0;
static float	adc2_raw = 0.0;
static uint8_t	cruise_ = 0;

/**
 * @brief activate buzzer placed on AUX1 output for a given time period and
 * number of times
 * 
 * @param b_t1 buzzer beep duration in ms
 * @param b_t2 buzzer pause duration in ms
 * @return ** void 
 */
static void activate_buzzer(uint32_t b_t1, uint32_t b_t2){
	static uint32_t buzzer_t = 0;
	//current time in ms
	uint32_t cur_time = (VESC_IF->system_time()*1000);

	if((cur_time - buzzer_t) > (b_t1 + b_t2) ){
		VESC_IF->set_pad(BUZZER_GPIO, BUZZER_PIN);
		VESC_IF->sleep_ms(b_t1);
		VESC_IF->clear_pad(BUZZER_GPIO, BUZZER_PIN);
		buzzer_t = cur_time; //reset local time counter
	}
	return ;
}

/**
 * @brief 
 * 
 * @param args 
 * @param argn 
 * @return lbm_value 0=nothing done; 1=limited mode activated; 2=alarm active;
 * 3=alarm ddeactivated
 */
static lbm_value rf_receiver(lbm_value *args, lbm_uint argn){
	int ch1_read = VESC_IF->io_read(VESC_PIN_SWCLK);
	int ch2_read = VESC_IF->io_read(VESC_PIN_SWDIO);
	static uint8_t ch1_status = 0;
	static uint8_t ch2_status = 0;
	uint32_t retval = 0;
	if(ch1_read > 0){
		if(ch1_status == 0){
			ch1_status = 1;
			if(limited_mode == 0){
				limited_mode = 1;
				//configure mode
				activate_buzzer(50,0);
				retval = 1;
			}
		}
	} else {
		ch1_status = 0;
	}

	if(ch2_read > 0){
		if(ch2_status == 0){
			ch2_status = 1;
			if(alarm == 0){
				alarm = 1; //activate alarm
				retval = 2;
				activate_buzzer(100,0);
			} else {
				alarm = 0; //deactivate alarm
				activate_buzzer(200,0);
				retval = 3;
			}
		}
	} else {
		ch2_status = 0;
	}
	return VESC_IF->lbm_enc_i(retval);
}

static lbm_value adc2_taps(lbm_value *args, lbm_uint argn){
	static uint8_t adc2_flag = 0;
	static uint32_t adc2_time = 0;
	uint32_t cur_time = (VESC_IF->system_time()*1000);
	float speed = VESC_IF->mc_get_speed();
	adc2_decoded = VESC_IF->lbm_dec_as_float(args[0]);
	adc2_raw = VESC_IF->io_read_analog(VESC_PIN_ADC2);
	uint8_t retval = 0;
	if( adc2_decoded > 0.8){
		if(adc2_flag == 0){
			if(adc2_n > 0 && (cur_time-adc2_time)>800){
				adc2_n = 0; //reset if period is more than 800ms
			}
			adc2_n++;
			adc2_time = cur_time;
			adc2_flag = 1;
			activate_buzzer(5,0);
		}
	} else {
		adc2_flag = 0;
	}
	//change driving mode only when under 5km/h
	if(adc2_n > 0 && (cur_time - adc2_time) > 5000 && speed<5){
		switch (adc2_n){
		case 3:
			//config mode ECO
			retval = 1;
			break;
		case 4:
			//config mode DRIVE
			retval = 2;
			break;
		case 5:
			//config mode SPORT
			retval = 3;
			break;
		case 6:
			//config mode TRACK
			retval = 4;
			break;
		default:
			break;
		}
		//sound feedback
		for(int i=0; i<adc2_n; i++){
			activate_buzzer(10,0);
			VESC_IF->sleep_ms(300);
		}
		adc2_n = 0;	//reset "tap" counter
		power_mode = retval;
	}
	return VESC_IF->lbm_enc_u(retval);
}

/**
 * @brief 
 * 
 * @param args 
 * @param argn 
 * @return lbm_value 
 */
static lbm_value brake_ligth(lbm_value *args, lbm_uint argn){
	static uint32_t brake_ligth_time = 0;
	uint32_t cur_time = (VESC_IF->system_time()*1000);
	if(adc1_decoded > 0.05){
		if(emergency_brake_flag > 0){ //hard brake -> brake ligth blink
			if((cur_time-brake_ligth_time) > 200){
				if(brake_ligth_status > 0){
					brake_ligth_status = 0;
					VESC_IF->clear_pad(BRAKE_LIGTH_GPIO, BRAKE_LIGTH_PIN);
				} else {
					brake_ligth_status = 1;
					VESC_IF->set_pad(BRAKE_LIGTH_GPIO, BRAKE_LIGTH_PIN);
					activate_buzzer(20,200);
				}
			} else if(brake_ligth_status == 0){
				brake_ligth_status = 1;
				VESC_IF->set_pad(BRAKE_LIGTH_GPIO, BRAKE_LIGTH_PIN);
			}
		}
	} else {
		if(emergency_brake_flag > 0){
			emergency_brake_flag = 0; //reset emergency brake flag
		}
		if(brake_ligth_status > 0){
			brake_ligth_status = 0;
			VESC_IF->clear_pad(BRAKE_LIGTH_GPIO, BRAKE_LIGTH_PIN);
		}
	}
	return VESC_IF->lbm_enc_i(1);
}

static lbm_value cruise(lbm_value *args, lbm_uint argn){
	static uint8_t adc1_flag = 0;
	static uint32_t adc1_time = 0;
	uint32_t max_speed = VESC_IF->lbm_dec_as_u32(args[1]);
	adc1_decoded = VESC_IF->lbm_dec_as_float(args[0]);
	adc1_raw = VESC_IF->io_read_analog(VESC_PIN_ADC1);
	uint32_t cur_time = (VESC_IF->system_time()*1000);
	float speed = VESC_IF->mc_get_speed();
	int retval = 0;
	//enabling cruise
	if(adc1_decoded > 0.9){
		if(adc1_flag == 0){
			if(adc1_n > 0 && (cur_time-adc1_time)>800){
				adc1_n = 0; //reset if period is more than 800ms
			}
			adc1_n++;
			adc1_time = cur_time;
			adc1_flag = 1;
		}
	} else {
		adc1_flag = 0;
		if(adc1_n == 2){
			if((cur_time - adc1_time) > 1500){
				adc1_n = 0;
				cruise_ = 1;
				//activate cruise
				retval = 1;
				activate_buzzer(20, 0);
			}
		} else if(adc1_n > 2){
			adc1_n = 0;
		}
	}
	//disabling cruise
	if( ((adc1_decoded > 0.05 && (cur_time - adc1_time) > 2000) 
			|| adc2_decoded > 0.05 || speed > max_speed) && cruise_ > 0){
		cruise_ = 0;
		//disable cruise
		retval = -1;
		activate_buzzer(20,0);
	}
	return VESC_IF->lbm_enc_i(retval);
}

INIT_FUN(lib_info *info) {
	INIT_START
	(void)info;
	
	VESC_IF->lbm_add_extension("check_remote", rf_receiver);
	VESC_IF->lbm_add_extension("cruise_control", cruise);
	VESC_IF->lbm_add_extension("brake_ligth", brake_ligth);
	VESC_IF->lbm_add_extension("brake_taps", adc2_taps);
	VESC_IF->set_pad_mode(BUZZER_GPIO, BUZZER_PIN, PAL_STM32_MODE_OUTPUT);
	VESC_IF->set_pad_mode(BRAKE_LIGTH_GPIO, BRAKE_LIGTH_PIN, PAL_STM32_MODE_OUTPUT);
	VESC_IF->sleep_ms(5000);
	VESC_IF->io_set_mode(VESC_PIN_SWCLK,VESC_PIN_MODE_INPUT_NOPULL);
	VESC_IF->io_set_mode(VESC_PIN_SWDIO,VESC_PIN_MODE_INPUT_NOPULL);
	return true;
}


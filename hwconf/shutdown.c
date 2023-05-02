/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "shutdown.h"
#include "app.h"
#include "conf_general.h"
#include "mc_interface.h"

#ifdef USE_LISPBM
#include "lispif.h"
#endif

#ifdef HW_SHUTDOWN_HOLD_ON

// Private variables
static volatile float m_inactivity_time = 0.0;
static THD_WORKING_AREA(shutdown_thread_wa, 128);
static volatile bool m_init_done = false;
static volatile uint64_t odometer_old = mc_interface_get_odometer();

// Private functions
static THD_FUNCTION(shutdown_thread, arg);

void shutdown_init(void) {
	chThdCreateStatic(shutdown_thread_wa, sizeof(shutdown_thread_wa), NORMALPRIO, shutdown_thread, NULL);
	m_init_done = true;
}

void shutdown_reset_timer(void) {
	m_inactivity_time = 0.0;
}

bool shutdown_button_pressed(void) {
	return false;
}

float shutdown_get_inactivity_time(void) {
	return m_inactivity_time;
}

void shutdown_set_sampling_disabled(bool disabled) {
}

static bool do_shutdown(void) {
	//save settings only if we have done at least 1km to save writing cycles
	if((mc_interface_get_odometer()-odometer_old) >= 1000) {
		conf_general_store_backup_data();
		odometer_old = mc_interface_get_odometer();
	}
#ifdef USE_LISPBM
	lispif_process_shutdown();
#endif
	chThdSleepMilliseconds(100);
	DISABLE_GATE();
	HW_SHUTDOWN_HOLD_OFF();
	return true;
}

static THD_FUNCTION(shutdown_thread, arg) {
	(void)arg;

	chRegSetThreadName("Shutdown");

	systime_t last_iteration_time = chVTGetSystemTimeX();
	odometer_old = mc_interface_get_odometer();
	
	for(;;) {
		float dt = (float)chVTTimeElapsedSinceX(last_iteration_time) / (float)CH_CFG_ST_FREQUENCY;
		last_iteration_time = chVTGetSystemTimeX();

		const app_configuration *conf = app_get_configuration();

		if(conf->shutdown_mode == SHUTDOWN_MODE_ALWAYS_ON) {
			m_inactivity_time += dt;
			HW_SHUTDOWN_HOLD_ON();
			// Without a shutdown switch use inactivity timer to estimate
			// when device is stopped. Check also distance between store
			// to prevent excessive flash write cycles.
			if (m_inactivity_time >= SHUTDOWN_SAVE_BACKUPDATA_TIMEOUT) {
				shutdown_reset_timer();
				// If at least 1km was done then we can store data 
				if((mc_interface_get_odometer()-odometer_old) >= 1000) {
					conf_general_store_backup_data();
					odometer_old = mc_interface_get_odometer();
				}
			}
		}

		if (conf->shutdown_mode >= SHUTDOWN_MODE_OFF_AFTER_10S) {
			m_inactivity_time += dt;

			float shutdown_timeout = 0.0;
			switch (conf->shutdown_mode) {
			case SHUTDOWN_MODE_OFF_AFTER_10S: shutdown_timeout = 10.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_1M: shutdown_timeout = 60.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_5M: shutdown_timeout = 60.0 * 5.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_10M: shutdown_timeout = 60.0 * 10.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_30M: shutdown_timeout = 60.0 * 30.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_1H: shutdown_timeout = 60.0 * 60.0; break;
			case SHUTDOWN_MODE_OFF_AFTER_5H: shutdown_timeout = 60.0 * 60.0 * 5.0; break;
			default: break;
			}

			if (m_inactivity_time >= shutdown_timeout) {
				do_shutdown();
				chThdSleepMilliseconds(500);
				shutdown_reset_timer();
				HW_SHUTDOWN_HOLD_ON();
				//reset timer if esc didn't shut down
				//example if a switch is connected instead of a button
			}
		}

		chThdSleepMilliseconds(10);
	}
}

#else // HARDWARE WITHOUT POWER SWITCH 
// just saving backup data, no actual shutdown

// Private variables
static volatile float m_inactivity_time = 0.0;
static THD_WORKING_AREA(shutdown_thread_wa, 128);

// Private functions
static THD_FUNCTION(shutdown_thread, arg);

void shutdown_init(void) {
	chThdCreateStatic(shutdown_thread_wa, sizeof(shutdown_thread_wa), LOWPRIO, shutdown_thread, NULL);
}

void shutdown_reset_timer(void) {
	m_inactivity_time = 0.0;
}

float shutdown_get_inactivity_time(void) {
	return m_inactivity_time;
}

static THD_FUNCTION(shutdown_thread, arg) {
	(void)arg;

	chRegSetThreadName("Shutdown");

	systime_t last_iteration_time = chVTGetSystemTimeX();
	uint64_t odometer_old = mc_interface_get_odometer();

	for(;;) {
		float dt = (float)chVTTimeElapsedSinceX(last_iteration_time) / (float)CH_CFG_ST_FREQUENCY;
		last_iteration_time = chVTGetSystemTimeX();

		const app_configuration *conf = app_get_configuration();

		//if set to always off don't store backup
		if(conf->shutdown_mode != SHUTDOWN_MODE_ALWAYS_OFF) {
			m_inactivity_time += dt;
			if (m_inactivity_time >= SHUTDOWN_SAVE_BACKUPDATA_TIMEOUT) {
				shutdown_reset_timer();
				// Without a shutdown switch use inactivity time to measure
				// when stopped. If timeout is passed and trip distance is
				// greater than 1km store it to prevent excessive flash write.
				// Example, i stop for 4 miutes after 3,4km and the firmware
				// stores parameters, if i stop for 20s or after 600m no.
				if((mc_interface_get_odometer()-odometer_old) >= 1000) {
					conf_general_store_backup_data();
					odometer_old = mc_interface_get_odometer();
				}
			}
		}

		chThdSleepMilliseconds(1000);
	}
}

#endif

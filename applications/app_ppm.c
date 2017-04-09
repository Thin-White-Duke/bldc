/*
	Copyright 2012-2015 Benjamin Vedder	benjamin@vedder.se

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

/*
 * app_ppm.c
 *
 *  Created on: 18 apr 2014
 *      Author: benjamin
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include <math.h>
//#include "commands.h"

// Only available if servo output is not active
#if !SERVO_OUT_ENABLE

// Settings
#define MAX_CAN_AGE			0.1
#define MIN_PULSES_WITHOUT_POWER	50
#define RPM_FILTER_SAMPLES		8

// Threads
static THD_FUNCTION(ppm_thread, arg);
static THD_WORKING_AREA(ppm_thread_wa, 1024);
static thread_t *ppm_tp;
virtual_timer_t vt;

// Private functions
static void servodec_func(void);

// Private variables
static volatile bool is_running = false;
static volatile float pid_rpm = 0;
//static volatile int mode_switch_pulses = 0;
static volatile ppm_config config;
static volatile throttle_config throt_config;
static volatile int pulses_without_power = 0;

//static volatile int count = 0;

static volatile float filter_buffer[RPM_FILTER_SAMPLES];
static volatile int filter_ptr = 0;
static volatile bool has_enough_pid_filter_data = false;
static volatile float direction_hyst = 0;

static float px[5];
static float py[5];
static float nx[5];
static float ny[5];

// Private functions
static void update(void *p);
#endif

float calculate_throttle_curve_ppm(float *x, float *y, float bezier_reduce_factor, float t) {
	
	float directSteps;
	if (t < x[1]){
	    directSteps = (y[1] / x[1] * t);
	} else if (t > x[3]) {
	    directSteps = ((y[4] - y[3]) / (x[4] - x[3]) * (t-x[3]) + y[3]);
	} else if (t > x[2]) {
	    directSteps = ((y[3] - y[2]) / (x[3] - x[2]) * (t-x[2]) + y[2]);
	} else if (t > x[1]) {
	    directSteps = ((y[2] - y[1]) / (x[2] - x[1]) * (t-x[1]) + y[1]);
	} else { // (throttle == x[1])
	    directSteps = y[1];
	};

	float f[5];
	for (int i = 0; i < 5; i++) f[i] = y[i];

	for (int j = 1; j < 5; j++ )
		for (int i = 4; i >= j; i--)
			f[i] = ( (t - x[i-j]) * f[i] - (t - x[i]) * f[i-1]) / (x[i] - x[i-j]);

	float spline = f[4] - ((f[4] - directSteps) * bezier_reduce_factor);

	// safety when stupid values are entered for x and y
	if (spline > 1.0) return 1.0;
	if (spline < 0.0) return 0.0;

	return spline;
}

void app_ppm_configure(ppm_config *conf, throttle_config *throttle_conf) {
#if !SERVO_OUT_ENABLE
	config = *conf;
	throt_config = *throttle_conf;
	pulses_without_power = 0;
	
	has_enough_pid_filter_data = false;
	filter_ptr = 0;
	mc_interface_set_cruise_control_status(CRUISE_CONTROL_INACTIVE);

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_center, config.pulse_end, config.median_filter);
	}
	
	px[0] = 0.0;
	px[1] = throt_config.x1_throttle;
	px[2] = throt_config.x2_throttle;
	px[3] = throt_config.x3_throttle;
	px[4] = 1.0;
	py[0] = 0.0;
	py[1] = throt_config.y1_throttle;
	py[2] = throt_config.y2_throttle;
	py[3] = throt_config.y3_throttle;
	py[4] = 1.0;
	
	nx[0] = 0.0;
	nx[1] = throt_config.x1_neg_throttle;
	nx[2] = throt_config.x2_neg_throttle;
	nx[3] = throt_config.x3_neg_throttle;
	nx[4] = 1.0;
	ny[0] = 0.0;
	ny[1] = throt_config.y1_neg_throttle;
	ny[2] = throt_config.y2_neg_throttle;
	ny[3] = throt_config.y3_neg_throttle;
	ny[4] = 1.0;
	
	direction_hyst = config.max_erpm_for_dir * 0.20;
#else
	(void)conf;
#endif
}

void app_ppm_start(void) {
#if !SERVO_OUT_ENABLE
	chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa), NORMALPRIO, ppm_thread, NULL);

	chSysLock();
	chVTSetI(&vt, MS2ST(1), update, NULL);
	chSysUnlock();
#endif
}

#if !SERVO_OUT_ENABLE
static void servodec_func(void) {
	chSysLockFromISR();
	timeout_reset();
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static void update(void *p) {
	chSysLockFromISR();
	chVTSetI(&vt, MS2ST(2), update, p);
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(ppm_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PPM");
	ppm_tp = chThdGetSelfX();

	servodec_set_pulse_options(config.pulse_start, config.pulse_center, config.pulse_end, config.median_filter);
	servodec_init(servodec_func);
	
	is_running = true;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		if (timeout_has_timeout() || servodec_get_time_since_update() > timeout_get_timeout_msec() ||
				mc_interface_get_fault() != FAULT_CODE_NONE) {
			pulses_without_power = 0;
			continue;
		}

		float servo_val = servodec_get_servo(0);

		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT_NOREV:
		case PPM_CTRL_TYPE_DUTY_NOREV:
		case PPM_CTRL_TYPE_PID_NOREV:
			servo_val += 1.0;
			servo_val /= 2.0;
			
			utils_deadband(&servo_val, config.hyst, 1.0);
			
			servo_val = calculate_throttle_curve_ppm(px, py, throt_config.bezier_reduce_factor, servo_val);
			
			break;
		//case PPM_CTRL_TYPE_CURRENT:
		case PPM_CTRL_TYPE_WATT:
			utils_deadband(&servo_val, config.hyst, 1.0);

			if (throt_config.adjustable_throttle_enabled && servo_val != 0.0){
				if (servo_val > 0.0) {
					servo_val = calculate_throttle_curve_ppm(px, py, throt_config.bezier_reduce_factor, servo_val);
				} else if (config.max_erpm_for_dir_active == false) {
					servo_val = -calculate_throttle_curve_ppm(nx, ny, throt_config.bezier_neg_reduce_factor, -servo_val);
				}
			}
			break;
		default:
			
			utils_deadband(&servo_val, config.hyst, 1.0);
			
			if (throt_config.adjustable_throttle_enabled && servo_val != 0.0){
				if (servo_val > 0.0) {
					servo_val = calculate_throttle_curve_ppm(px, py, throt_config.bezier_reduce_factor, servo_val);
				} else {
					servo_val = -calculate_throttle_curve_ppm(nx, ny, throt_config.bezier_neg_reduce_factor, -servo_val);
				}
			}
			
			break;
		}

		float current = 0;
		//float desired_watt = 0;
		bool current_mode = false;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		bool send_duty = false;
		//bool send_watt = false;
		bool send_pid = false;
		
		// Find lowest RPM and cruise control
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		float mid_rpm = rpm_local;
		int motor_count = 1;
		ppm_cruise cruise_control_status = CRUISE_CONTROL_INACTIVE;
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {					
					// add to middle rpm count
					mid_rpm += msg->rpm;
					motor_count += 1;

					if (fabsf(msg->rpm) < fabsf(rpm_lowest)) {
						rpm_lowest = msg->rpm;
					}

					// if any controller (VESC) sends the cruise contol status
					if (msg->cruise_control_status != CRUISE_CONTROL_INACTIVE) {
						cruise_control_status = msg->cruise_control_status;
					}
				}
			}
		}
		
		// get middle rpm
		mid_rpm /= motor_count;
		
		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT:
			current_mode = true;
			if (servo_val >= 0.0) {
				// check of can bus send cruise control command
				if (cruise_control_status != CRUISE_CONTROL_INACTIVE && servo_val == 0.0) {
					// is rpm in range for cruise control
					if (rpm_lowest > mcconf->s_pid_min_erpm) {
						if (pid_rpm == 0) {
							pid_rpm = rpm_lowest;
						}
						current_mode = false;
						send_pid = true;
						mc_interface_set_pid_speed_with_cruise_status(rpm_local + pid_rpm - mid_rpm, cruise_control_status);
					} else {
						pid_rpm = 0;
						current = 0.0;
					}
				}else{
					current = servo_val * mcconf->l_current_max;
				}
			} else {
				current = servo_val * fabsf(mcconf->l_current_min);
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
			break;
		case PPM_CTRL_TYPE_CURRENT_NOREV:
		case PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE:
			current_mode = true;
			
			if (servo_val >= 0.0) {
				// check of can bus send cruise control command
				if (cruise_control_status != CRUISE_CONTROL_INACTIVE && servo_val == 0.0) {
					// is rpm in range for cruise control
					if (rpm_lowest > mcconf->s_pid_min_erpm) {
						if (pid_rpm == 0) {
							pid_rpm = rpm_lowest;
						}
						current_mode = false;
						send_pid = true;
						mc_interface_set_pid_speed_with_cruise_status(rpm_local + pid_rpm - mid_rpm, cruise_control_status);
					} else {
						pid_rpm = 0;
						current = 0.0;
					}
				}else{
					current = servo_val * mcconf->l_current_max;
				}
				
			} else {
				current = fabsf(servo_val * mcconf->l_current_min);
				current_mode_brake = true;
			}
			if (servo_val < 0.001) {
				pulses_without_power++;
			}
			break;
		case PPM_CTRL_TYPE_WATT:
			current_mode = true;
			
			if (config.max_erpm_for_dir_active) { // advanced backwards
				static bool force_brake = true;

				static int8_t did_idle_once = 0;

				// Hysteresis 20 % of actual RPM
				if (force_brake) {
					if (rpm_local < config.max_erpm_for_dir - direction_hyst) { // for 2500 it's 2000
						force_brake = false;
						did_idle_once = 0;
					}
				} else {	
					if (rpm_local > config.max_erpm_for_dir + direction_hyst) { // for 2500 it's 3000
						force_brake = true;
						did_idle_once = 0;
					}
				}
				
				if (servo_val >= 0.0) {
					if (servo_val == 0.0) {
						// if there was a idle in between then allow going backwards
						if (did_idle_once == 1 && !force_brake) {
							did_idle_once = 2;
						}
					}else{
						// accelerated forward or fast enough at least
						if (rpm_local > -config.max_erpm_for_dir){ // for 2500 it's -2500
							did_idle_once = 0;
						}
					}

					// check of can bus send cruise control command
					if (cruise_control_status != CRUISE_CONTROL_INACTIVE && servo_val == 0.0) {
						// is rpm in range for cruise control
						if (fabsf(rpm_lowest) > mcconf->s_pid_min_erpm) {
							if (pid_rpm == 0) {
								pid_rpm = mid_rpm;
							}
							current_mode = false;
							send_pid = true;
							
							mc_interface_set_pid_speed_with_cruise_status(rpm_local + pid_rpm - mid_rpm, cruise_control_status);
						} else {
							pid_rpm = 0;
							current = 0.0;
						}
					}else{
						if (mcconf->use_max_watt_limit) {
							current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
									servo_val * (mcconf->watts_max / GET_INPUT_VOLTAGE() / mc_interface_get_duty_cycle_for_watt_calculation()));
						} else {
							current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
									servo_val * mcconf->l_in_current_max / mc_interface_get_duty_cycle_for_watt_calculation());
						}
					}
				} else {

					// too fast
					if (force_brake){
						current_mode_brake = true;
					}else{
						// not too fast backwards
						if (rpm_local > -config.max_erpm_for_dir) { // for 2500 it's -2500
							// first time that we brake and we are not too fast
							if (did_idle_once != 2) {
								did_idle_once = 1;
								current_mode_brake = true;
							}
						// too fast backwards
						} else {
							// if brake was active already
							if (did_idle_once == 1) {
								current_mode_brake = true;
							} else {	
								// it's ok to go backwards now braking would be strange now
								did_idle_once = 2;
							}
						}
					}

					if (current_mode_brake) {
						// braking
						if (throt_config.adjustable_throttle_enabled) {
							// negative calculation
							servo_val = -calculate_throttle_curve_ppm(nx, ny, throt_config.bezier_neg_reduce_factor, -servo_val);
						}
						current = fabsf(servo_val * mcconf->l_current_min);
					}else {
						// reverse acceleration
						if (throt_config.adjustable_throttle_enabled) {
							// positive calculation
							servo_val = -calculate_throttle_curve_ppm(px, py, throt_config.bezier_reduce_factor, -servo_val);
						}

						if (mcconf->use_max_watt_limit) {
							current = -utils_smallest_of_2(fabsf(servo_val * mcconf->l_current_min), 
									fabsf(servo_val * (mcconf->watts_max / GET_INPUT_VOLTAGE() / mc_interface_get_duty_cycle_for_watt_calculation())));
						} else {
							current = -utils_smallest_of_2(fabsf(servo_val * mcconf->l_current_min), 
									fabsf(servo_val * mcconf->l_in_current_max / mc_interface_get_duty_cycle_for_watt_calculation()));
						}
					}
				}
			} else { // normal backwards
				if (servo_val >= 0.0) {
					// check of can bus send cruise control command
					if (cruise_control_status != CRUISE_CONTROL_INACTIVE && servo_val == 0.0) {
						// is rpm in range for cruise control
						if (fabsf(rpm_lowest) > mcconf->s_pid_min_erpm) {
							if (pid_rpm == 0) {
								pid_rpm = mid_rpm;
							}
							current_mode = false;
							send_pid = true;
							
							mc_interface_set_pid_speed_with_cruise_status(rpm_local + pid_rpm - mid_rpm, cruise_control_status);
						} else {
							pid_rpm = 0;
							current = 0.0;
						}
					}else{
						if (mcconf->use_max_watt_limit) {
							current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
									servo_val * (mcconf->watts_max / GET_INPUT_VOLTAGE() / mc_interface_get_duty_cycle_for_watt_calculation()));
						} else {
							current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
									servo_val * mcconf->l_in_current_max / mc_interface_get_duty_cycle_for_watt_calculation());
						}
					}
				} else {
					current = servo_val * fabsf(mcconf->l_current_min);
				}
			}

			if (servo_val < 0.001) {
				pulses_without_power++;
			}
			break;
		case PPM_CTRL_TYPE_WATT_NOREV_BRAKE:
			current_mode = true;
			
			if (servo_val >= 0.0) {
				// check of can bus send cruise control command
				if (cruise_control_status != CRUISE_CONTROL_INACTIVE && servo_val == 0.0) {
					// is rpm in range for cruise control
					if (rpm_lowest > mcconf->s_pid_min_erpm) {
						if (pid_rpm == 0) {
							pid_rpm = mid_rpm;
						}
						current_mode = false;
						send_pid = true;
						mc_interface_set_pid_speed_with_cruise_status(rpm_local + pid_rpm - mid_rpm, cruise_control_status);
						
					} else {
						pid_rpm = 0;
						current = 0.0;
					}
				}else{
					if (mcconf->use_max_watt_limit) {
						current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
								servo_val * (mcconf->watts_max / GET_INPUT_VOLTAGE() / mc_interface_get_duty_cycle_for_watt_calculation()));
					} else {
						current = utils_smallest_of_2(servo_val * mcconf->l_current_max, 
								servo_val * mcconf->l_in_current_max / mc_interface_get_duty_cycle_for_watt_calculation());
					}
				}
			} else {
				
				current_mode_brake = true;
				
				// brake as in current mode if not enabled. Calculating via Battery Voltage would brake far too hard
				current = fabsf(servo_val * mcconf->l_current_min);
			}
			if (servo_val < 0.001) {
				pulses_without_power++;
			}
			break;
		case PPM_CTRL_TYPE_PID_NOACCELERATION:
			current_mode = true;
			
			filter_buffer[filter_ptr++] = mid_rpm;
			if (filter_ptr >= RPM_FILTER_SAMPLES) {
				filter_ptr = 0;
				has_enough_pid_filter_data = true;
			}
			
			float rpm_filtered = 0.0;
			// only send when enough values are collected
			if (has_enough_pid_filter_data)	{
				for (int i = 0; i < RPM_FILTER_SAMPLES; i++) {
					rpm_filtered += filter_buffer[i];
				}
				rpm_filtered /= RPM_FILTER_SAMPLES;
			}
			
			if (servo_val >= 0.0) {
				// check if pid needs to be lowered
				if (servo_val > 0.0) {
					// needs to be set first ?
					if (pid_rpm == 0) {
						pid_rpm = rpm_filtered;
					}
					
					if(rpm_filtered > 1000){
						float diff = pid_rpm - rpm_filtered;
						if (diff > 1500) {
							pid_rpm = pid_rpm - 10;
						}else if(diff > 500 && rpm_filtered < 1500) {
							pid_rpm = pid_rpm - 10;
						}
					}else{
						pid_rpm = 0;
					}
					
					// if not too slow than set the pid
					if(pid_rpm > 0 && pid_rpm < config.pid_max_erpm) {
						current_mode = false;
						
						send_pid = true;
						mc_interface_set_pid_speed(rpm_local + pid_rpm - rpm_filtered);
						
						// overwrite mid_rpm
						mid_rpm = rpm_filtered;

					} else {
						current = 0.0;
					}
				}else{
					current = 0.0;
				}				
			} else {
				current = fabsf(servo_val * mcconf->l_current_min);
				current_mode_brake = true;
			}
			
			if (servo_val < 0.001) {
				pulses_without_power++;
			}
			break;
		case PPM_CTRL_TYPE_CRUISE_CONTROL_SECONDARY_CHANNEL:
			if (servo_val < -0.3 && config.cruise_left != CRUISE_CONTROL_INACTIVE) {
				mc_interface_set_cruise_control_status(config.cruise_left);
			} else if (servo_val > 0.3 && config.cruise_right != CRUISE_CONTROL_INACTIVE) {
				mc_interface_set_cruise_control_status(config.cruise_right);
			} else {
				mc_interface_set_cruise_control_status(CRUISE_CONTROL_INACTIVE);
			}
			// Run this loop at 500Hz
			chThdSleepMilliseconds(2);

			// Reset the timeout
			//timeout_reset();
			continue;
			break;
		case PPM_CTRL_TYPE_DUTY:
		case PPM_CTRL_TYPE_DUTY_NOREV:
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_duty(utils_map(servo_val, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;

		case PPM_CTRL_TYPE_PID:
		case PPM_CTRL_TYPE_PID_NOREV:
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_pid_speed(servo_val * config.pid_max_erpm);
				send_duty = true;
			}
			break;

		default:
			continue;
		}
		
		// switch the mode
		/*if (servo_val == -1.0){
			mode_switch_pulses++;			
		}else{
			if (mode_switch_pulses > 1000){
				//switch mode
				//mc_interface_change_speed_mode(); // ???
			}
			mode_switch_pulses = 0;
		}*/

		if (pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (pulses_without_power == pulses_without_power_before) {
				pulses_without_power = 0;
			}
			pulses_without_power_before = pulses_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());
			continue;
		}

		if (send_duty && config.multi_esc) {
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_duty(msg->id, duty);
				}
			}
		}

		if (send_pid && config.multi_esc) {
			
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_rpm(msg->id, msg->rpm + pid_rpm - mid_rpm, cruise_control_status);
				}
			}
		}

		if (current_mode) {

			pid_rpm = 0; // always reset in current, not that something stupid happens

			if (current_mode_brake) {
				mc_interface_set_brake_current(current);

				if (config.multi_esc) {
					// Send brake command to all ESCs seen recently on the CAN bus
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							if (config.ctrl_type == PPM_CTRL_TYPE_WATT_NOREV_BRAKE || config.ctrl_type == PPM_CTRL_TYPE_WATT) {
								comm_can_set_brake_servo(msg->id, fabsf(servo_val));
							} else {
								comm_can_set_current_brake(msg->id, current);
							}
						}
					}
				}
			} else {
				// Apply soft RPM limit
				bool use_min_current = false;
				if (rpm_lowest > config.rpm_lim_end) {
					if (servo_val > 0.0) {
						use_min_current = true;
						servo_val = 0.000001; // make sure min current ius used
					}
					if (current > 0.0) {
						current = mcconf->cc_min_current;	
					}

				} else if (rpm_lowest > config.rpm_lim_start) {
					if (servo_val > 0.0) {
						use_min_current = true;
						servo_val = utils_map(rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, servo_val, 0.000001);
					}
					if (current > 0.0) {
						current = utils_highest_of_2(utils_map(rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, current, 0.0), mcconf->cc_min_current);
					}

				} else if (rpm_lowest < -config.rpm_lim_end) {
					if (servo_val < 0.0) {
						use_min_current = true;
						servo_val = -0.000001; // make sure min current ius used
					}
					if (current < 0.0) {
						current = -mcconf->cc_min_current; // wrong in original
					}					

				} else if (rpm_lowest < -config.rpm_lim_start) {
					if (servo_val < 0.0) {
						use_min_current = true;
						servo_val = -utils_map(-rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, -servo_val, 0.000001);
					}
					if (current < 0.0) {
						current = utils_smallest_of_2(-utils_map(-rpm_lowest, config.rpm_lim_start, config.rpm_lim_end, -current, 0.0), -mcconf->cc_min_current);
					}
				}
				
				float current_out = current;
				float servo_val_out = servo_val;
				bool is_reverse = false;
				if (servo_val < 0.0) {
					is_reverse = true;
					current_out = -current_out;
					current = -current;
					servo_val_out = -servo_val_out;
					servo_val = -servo_val;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
				}

				// Traction control
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							if (config.tc) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
								if (diff > config.tc_offset) {
									current_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, current, 0.0);
									servo_val_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, servo_val, 0.0);
								} else {
									current_out = current;
									servo_val_out = servo_val;
								}
							}

							if (is_reverse) {
								if (config.ctrl_type == PPM_CTRL_TYPE_WATT_NOREV_BRAKE || (config.ctrl_type == PPM_CTRL_TYPE_WATT && config.max_erpm_for_dir_active)) { // if not active you want real brakes
									comm_can_set_servo(msg->id, -servo_val_out, use_min_current);
								} else {
									comm_can_set_current(msg->id, -current_out);
								}
							} else {
								if (config.ctrl_type == PPM_CTRL_TYPE_WATT_NOREV_BRAKE || config.ctrl_type == PPM_CTRL_TYPE_WATT) {
									comm_can_set_servo(msg->id, servo_val_out, use_min_current);
								} else {
									comm_can_set_current(msg->id, current_out);
								}
							}
						}
					}

					if (config.tc) {
						
						float diff = rpm_local - rpm_lowest;
						if (diff > config.tc_offset) {
							current_out = utils_map(diff - config.tc_offset, 0.0, config.tc_max_diff - config.tc_offset, current, 0.0);
						} else {
							current_out = current;
						}
					}
				}

				if (is_reverse) {
					mc_interface_set_current(-current_out);
				} else {
					mc_interface_set_current(current_out);
				}
			}
		}
	}
}
#endif

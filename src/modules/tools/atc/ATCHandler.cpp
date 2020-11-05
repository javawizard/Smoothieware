/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ATCHandler.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ATCHandler.h"
#include "SlowTicker.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"
#include "modules/robot/Conveyor.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "libs/utils.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "ATCHandlerPublicAccess.h"

#include "FileStream.h"
#include <math.h>

#define ATC_AXIS 4
#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

#define atc_checksum            	CHECKSUM("atc")
#define enable_checksum          	CHECKSUM("enable")
#define endstop_pin_checksum      	CHECKSUM("homing_endstop_pin")
#define debounce_ms_checksum      	CHECKSUM("homing_debounce_ms")
#define max_travel_mm_checksum    	CHECKSUM("homing_max_travel_mm")
#define homing_retract_mm_checksum  CHECKSUM("homing_retract_mm")
#define homing_rate_mm_s_checksum   CHECKSUM("homing_rate_mm_s")
#define action_mm_checksum      	CHECKSUM("action_mm")
#define action_rate_mm_s_checksum   CHECKSUM("action_rate_mm_s")

#define detector_switch_checksum    CHECKSUM("toolsensor")
#define detector_checksum           CHECKSUM("detector")
#define detect_pin_checksum			CHECKSUM("detect_pin")
#define detect_rate_mm_s_checksum	CHECKSUM("detect_rate_mm_s")
#define detect_travel_mm_checksum 	CHECKSUM("detect_travel_mm")

#define tool_number_checksum     	CHECKSUM("tool_number")
#define active_tool_checksum     	CHECKSUM("active_tool")

#define mx_mm_checksum     			CHECKSUM("mx_mm")
#define my_mm_checksum     			CHECKSUM("my_mm")
#define mz_mm_checksum     			CHECKSUM("mz_mm")
#define safe_z_checksum				CHECKSUM("safe_z_mm")
#define safe_z_offset_checksum		CHECKSUM("safe_z_offset_mm")
#define fast_z_rate_checksum		CHECKSUM("fast_z_rate_mm_m")
#define slow_z_rate_checksum		CHECKSUM("slow_z_rate_mm_m")

#define probe_checksum				CHECKSUM("probe")
#define fast_rate_mm_m_checksum		CHECKSUM("fast_rate_mm_m")
#define slow_rate_mm_m_checksum		CHECKSUM("slow_rate_mm_m")
#define retract_mm_checksum			CHECKSUM("retract_mm")
#define probe_height_mm_checksum	CHECKSUM("probe_height_mm")

ATCHandler::ATCHandler()
{
    atc_status = NONE;
    atc_home_info.clamp_status = UNHOMED;
    atc_home_info.triggered = false;
    detector_info.triggered = false;
    ref_tool_mz = 0.0;
    cur_tool_mz = 0.0;
    tool_offset = 0.0;
    last_pos[0] = 0.0;
    last_pos[1] = 0.0;
    last_pos[2] = 0.0;
}

void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::fill_drop_scripts(int old_tool) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[old_tool];
    // lift z axis to atc start position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
    // move x and y to active tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", current_tool->mx_mm, current_tool->my_mm);
	this->script_queue.push(buff);
	// move around to see if tool rack is empty
	this->script_queue.push("M492.2");
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", current_tool->mz_mm + safe_z_offset_mm, fast_z_rate);
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", current_tool->mz_mm, slow_z_rate);
	this->script_queue.push(buff);
	// loose tool
	this->script_queue.push("M490.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
	// move around to see if tool is dropped, halt if not
	this->script_queue.push("M492.1");
	// set new tool to -1
	this->script_queue.push("M493.2 T-1");
}

void ATCHandler::fill_pick_scripts(int new_tool) {
	char buff[100];
	struct atc_tool *current_tool = &atc_tools[new_tool];
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
	// move x and y to new tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", current_tool->mx_mm, current_tool->my_mm);
	this->script_queue.push(buff);
	// move around to see if tool rack is filled
	this->script_queue.push("M492.1");
	// loose tool
	this->script_queue.push("M490.2");
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", current_tool->mz_mm + safe_z_offset_mm, fast_z_rate);
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", current_tool->mz_mm, slow_z_rate);
	this->script_queue.push(buff);
	// clamp tool
	this->script_queue.push("M490.1");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
	// move around to see if tool rack is empty, halt if not
	this->script_queue.push("M492.2");
	// set new tool
	snprintf(buff, sizeof(buff), "M493.2 T%d", new_tool);
	this->script_queue.push(buff);

}

void ATCHandler::fill_cali_scripts() {
	char buff[100];
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
	// move x and y to calibrate position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", probe_mx_mm, probe_my_mm);
	this->script_queue.push(buff);
	// do calibrate with fast speed
	snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	this->script_queue.push(buff);
	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", probe_retract_mm);
	this->script_queue.push(buff);
	// do calibrate with slow speed
	snprintf(buff, sizeof(buff), "G38.6 Z%.3f F%.3f", -1 - probe_retract_mm, probe_slow_rate);
	this->script_queue.push(buff);
	// save new tool offset
	this->script_queue.push("M493.1");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z_mm);
	this->script_queue.push(buff);
}

void ATCHandler::fill_zprobe_scripts(bool goto_last_pos) {
	char buff[100];
	if (goto_last_pos) {
		// goto last pos
		snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f", this->last_pos[0], this->last_pos[1]);
		this->script_queue.push(buff);
	}
	// do calibrate with fast speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", probe_mz_mm, probe_fast_rate);
	this->script_queue.push(buff);
	// lift a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", probe_retract_mm);
	this->script_queue.push(buff);
	// do calibrate with slow speed
	snprintf(buff, sizeof(buff), "G38.2 Z%.3f F%.3f", -1 - probe_retract_mm, probe_slow_rate);
	this->script_queue.push(buff);
	// set z working coordinate
	snprintf(buff, sizeof(buff), "G10 L20 P0 Z%.3f", probe_height_mm);
	this->script_queue.push(buff);
	// retract z a bit
	snprintf(buff, sizeof(buff), "G91 G0 Z%.3f", probe_retract_mm);
	this->script_queue.push(buff);
}

void ATCHandler::fill_autolevel_scripts(float x_pos, float y_pos,
		float x_size, float y_size, int x_grids, int y_grids)
{
	char buff[100];

	// goto x and y path origin
	snprintf(buff, sizeof(buff), "G90 G0 X%.3f Y%.3f", x_pos, y_pos);
	this->script_queue.push(buff);

	// do auto leveling
	snprintf(buff, sizeof(buff), "G32R1X0Y0A%.3fB%.3fI%dJ%d", x_size, y_size, x_grids, y_grids);
	this->script_queue.push(buff);
}

void ATCHandler::on_module_loaded()
{

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_HALT);

    this->on_config_reload(this);

    THEKERNEL->slow_ticker->attach(1000, this, &ATCHandler::read_endstop);
    THEKERNEL->slow_ticker->attach(1000, this, &ATCHandler::read_detector);

    // load current tool from eeprom
    this->active_tool = THEKERNEL->eeprom_data->TOOL;

}

void ATCHandler::on_config_reload(void *argument)
{
	char buff[10];

	atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("nc" )->as_string())->as_input();
	atc_home_info.debounce_ms    = THEKERNEL->config->value(atc_checksum, debounce_ms_checksum)->by_default(1  )->as_number();
	atc_home_info.max_travel    = THEKERNEL->config->value(atc_checksum, max_travel_mm_checksum)->by_default(5  )->as_number();
	atc_home_info.retract    = THEKERNEL->config->value(atc_checksum, homing_retract_mm_checksum)->by_default(3  )->as_number();
	atc_home_info.action_dist    = THEKERNEL->config->value(atc_checksum, action_mm_checksum)->by_default(1  )->as_number();
	atc_home_info.homing_rate    = THEKERNEL->config->value(atc_checksum, homing_rate_mm_s_checksum)->by_default(1  )->as_number();
	atc_home_info.action_rate    = THEKERNEL->config->value(atc_checksum, action_rate_mm_s_checksum)->by_default(1  )->as_number();

	detector_info.detect_pin.from_string( THEKERNEL->config->value(atc_checksum, detector_checksum, detect_pin_checksum)->by_default("nc" )->as_string())->as_input();
	detector_info.detect_rate = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_rate_mm_s_checksum)->by_default(1  )->as_number();
	detector_info.detect_travel = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_travel_mm_checksum)->by_default(1  )->as_number();

	this->safe_z_mm = THEKERNEL->config->value(atc_checksum, safe_z_checksum)->by_default(-10)->as_number();
	this->safe_z_offset_mm = THEKERNEL->config->value(atc_checksum, safe_z_offset_checksum)->by_default(10)->as_number();
	this->fast_z_rate = THEKERNEL->config->value(atc_checksum, fast_z_rate_checksum)->by_default(500)->as_number();
	this->slow_z_rate = THEKERNEL->config->value(atc_checksum, slow_z_rate_checksum)->by_default(60)->as_number();
	this->active_tool = THEKERNEL->config->value(atc_checksum, active_tool_checksum)->by_default(0)->as_number();
	this->tool_number = THEKERNEL->config->value(atc_checksum, tool_number_checksum)->by_default(6)->as_number();

	probe_mx_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, mx_mm_checksum)->by_default(-10  )->as_number();
	probe_my_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, my_mm_checksum)->by_default(-10  )->as_number();
	probe_mz_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, mz_mm_checksum)->by_default(-10  )->as_number();
	probe_fast_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, fast_rate_mm_m_checksum)->by_default(300  )->as_number();
	probe_slow_rate = THEKERNEL->config->value(atc_checksum, probe_checksum, slow_rate_mm_m_checksum)->by_default(60   )->as_number();
	probe_retract_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, retract_mm_checksum)->by_default(2   )->as_number();
	probe_height_mm = THEKERNEL->config->value(atc_checksum, probe_checksum, probe_height_mm_checksum)->by_default(0   )->as_number();

	atc_tools.clear();
	for (int i = 0; i <=  tool_number; i ++) {
		struct atc_tool tool;
		tool.num = i;
	    // lift z axis to atc start position
		snprintf(buff, sizeof(buff), "tool%d", i);
		tool.mx_mm = THEKERNEL->config->value(atc_checksum, get_checksum(buff), mx_mm_checksum)->by_default(-10  )->as_number();
		tool.my_mm = THEKERNEL->config->value(atc_checksum, get_checksum(buff), my_mm_checksum)->by_default(-10  )->as_number();
		tool.mz_mm = THEKERNEL->config->value(atc_checksum, get_checksum(buff), mz_mm_checksum)->by_default(-10  )->as_number();
		atc_tools.push_back(tool);
	}
}

void ATCHandler::on_halt(void* argument)
{
    if(argument == nullptr ) {
        this->atc_status = NONE;
        this->atc_home_info.clamp_status = UNHOMED;
        this->clear_script_queue();
        this->set_inner_playing(false);
	}
}

// Called every millisecond in an ISR
uint32_t ATCHandler::read_endstop(uint32_t dummy)
{

	if(!atc_homing || atc_home_info.triggered) return 0;

    if(STEPPER[ATC_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if(atc_home_info.pin.get()) {
            if(debounce < atc_home_info.debounce_ms) {
                debounce++;
            } else {
            	STEPPER[ATC_AXIS]->stop_moving();
            	atc_home_info.triggered = true;
                debounce = 0;
            }

        } else {
            // The endstop was not hit yet
            debounce = 0;
        }
    }

    return 0;
}

// Called every millisecond in an ISR
uint32_t ATCHandler::read_detector(uint32_t dummy)
{

    if(!detecting || detector_info.triggered) return 0;

    if (detector_info.detect_pin.get()) {
    	detector_info.triggered = true;
    }

    return 0;
}

bool ATCHandler::laser_detect() {
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();
    // move around and check laser detector
    detector_info.triggered = false;
    // switch on detector
    bool switch_state = true;
    bool ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("ERROR: Failed switch on detector switch.\r\n");
        return false;
    }
    detecting = true;

	float delta[Y_AXIS + 1];
	for (size_t i = 0; i <= Y_AXIS; i++) delta[i] = 0;
	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT->delta_move(delta, detector_info.detect_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= 0 - detector_info.detect_travel;
	THEROBOT->delta_move(delta, detector_info.detect_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	delta[Y_AXIS]= detector_info.detect_travel / 2;
	THEROBOT->delta_move(delta, detector_info.detect_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;


	detecting = false;
	// switch off detector
	switch_state = false;
    ok = PublicData::set_value(switch_checksum, detector_switch_checksum, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("ERROR: Failed switch off detector switch.\r\n");
        return false;
    }

    // THEROBOT->reset_position_from_current_actuator_position();

    return detector_info.triggered;
}

void ATCHandler::home_clamp()
{
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    atc_home_info.triggered = false;
    atc_home_info.clamp_status = UNHOMED;
    debounce = 0;
    atc_homing = true;

    // home atc
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS]= atc_home_info.max_travel; // we go the max
	THEROBOT->delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return;

	atc_homing = false;

    if (!atc_home_info.triggered) {
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->set_halt_reason(ATC_HOME_FAIL);
        THEKERNEL->streams->printf("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    }

    if(atc_home_info.triggered) {
    	THEROBOT->reset_position_from_current_actuator_position();
    }

    // Move back
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[ATC_AXIS] = -atc_home_info.retract; // we go to retract position
	THEROBOT->delta_move(delta, atc_home_info.homing_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();

	atc_home_info.clamp_status = CLAMPED;

}

void ATCHandler::clamp_tool()
{
	if (atc_home_info.clamp_status == CLAMPED) {
		THEKERNEL->streams->printf("Already clamped!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp();
		// change clamp status
		atc_home_info.clamp_status = CLAMPED;
		return;
	}
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	// change clamp status
	atc_home_info.clamp_status = CLAMPED;
}

void ATCHandler::loose_tool()
{
	if (atc_home_info.clamp_status == LOOSED) {
		THEKERNEL->streams->printf("Already loosed!\n");
		return;
	}
	if (atc_home_info.clamp_status == UNHOMED) {
		home_clamp();
	}
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = -atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	// change clamp status
	atc_home_info.clamp_status = LOOSED;
}

void ATCHandler::set_tool_offset()
{
    float px, py, pz;
    uint8_t ps;
    std::tie(px, py, pz, ps) = THEROBOT->get_last_probe_position();
    if (ps == 1) {
        cur_tool_mz = THEROBOT->from_millimeters(pz);
        if (ref_tool_mz < 0) {
        	tool_offset = cur_tool_mz - ref_tool_mz;
        	const float offset[3] = {0.0, 0.0, tool_offset};
        	THEROBOT->setToolOffset(offset);
        }
    }

}

void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
    	// gcode->stream->printf("Has m: %d\r\n", gcode->m);
    	if (gcode->m == 6 && gcode->has_letter('T')) {
    		// gcode->stream->printf("Has t\r\n");
    		if (atc_status != NONE) {
    			gcode->stream->printf("ATC already begun\r\n");
    			return;
    		}
            int new_tool = gcode->get_value('T');
            if (new_tool > this->tool_number) {
            	gcode->stream->printf("T%d invalid tool\r\n", new_tool);
            } else {
            	if (new_tool != active_tool) {
                    // push old state
                    THEROBOT->push_state();
                    THEROBOT->get_axis_position(last_pos, 3);
                    set_inner_playing(true);
                    this->clear_script_queue();
                	if (this->active_tool < 0) {
                		gcode->stream->printf("Start picking new tool: T%d\r\n", new_tool);
                		// just pick up tool
                		atc_status = PICK;
                		this->fill_pick_scripts(new_tool);
                		this->fill_cali_scripts();
                	} else if (new_tool < 0) {
                		gcode->stream->printf("Start dropping current tool: T%d\r\n", this->active_tool);
                		// just drop tool
                		atc_status = DROP;
                		this->fill_drop_scripts(active_tool);
                	} else {
                		gcode->stream->printf("Start atc, old tool: T%d, new tool: T%d\r\n", this->active_tool, new_tool);
                		// full atc progress
                		atc_status = FULL;
                	    this->fill_drop_scripts(active_tool);
                	    this->fill_pick_scripts(new_tool);
                	    this->fill_cali_scripts();
                	}

            	}
            }
		} else if (gcode->m == 490)  {
			if (gcode->subcode == 0) {
				// home tool change
				home_clamp();
				gcode->stream->printf("clamp homed!\r\n");

			} else if (gcode->subcode == 1) {
				// clamp tool
				clamp_tool();
				gcode->stream->printf("tool clamped!\r\n");

			} else if (gcode->subcode == 2) {
				// loose tool
				loose_tool();
				gcode->stream->printf("tool loosed!\r\n");
			}
		} else if (gcode->m == 491) {
			// do calibrate
            THEROBOT->push_state();
            THEROBOT->get_axis_position(last_pos, 3);
            set_inner_playing(true);
            this->clear_script_queue();
            atc_status = CALI;
    	    this->fill_cali_scripts();
		} else if (gcode->m == 492) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// check true
				if (!laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_NO_TOOL);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 2) {
				// check false
				if (laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->set_halt_reason(ATC_HAS_TOOL);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			}
		} else if (gcode->m == 493) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// set tooll offset
				set_tool_offset();
			} else if (gcode->subcode == 2) {
				// set new tool
				if (gcode->has_letter('T')) {
		    		this->active_tool = gcode->get_value('T');
		    		// save current tool data to eeprom
		    		if (THEKERNEL->eeprom_data->TOOL != this->active_tool) {
		        	    THEKERNEL->eeprom_data->TOOL = this->active_tool;
		        	    THEKERNEL->write_eeprom_data();
		    		}

				} else {
					THEKERNEL->call_event(ON_HALT, nullptr);
					THEKERNEL->set_halt_reason(ATC_NO_TOOL);
					THEKERNEL->streams->printf("ERROR: No tool was set!\n");

				}
			}
		} else if (gcode->m == 494) {
			// Do Z Probe, change probe tool if needed
            THEROBOT->push_state();
			set_inner_playing(true);
            this->clear_script_queue();
        	if (active_tool == 0) {
        		gcode->stream->printf("Do Probe right away\r\n");
                atc_status = PROBE;
        	    this->fill_zprobe_scripts(false);
        	} else {
                // save current position
                THEROBOT->get_axis_position(last_pos, 3);
        		if (active_tool < 0) {
        			//
            		gcode->stream->printf("Picking Probe tool and do Probe\r\n");
            		atc_status = PROBE_PICK;
            		this->fill_pick_scripts(0);
            		this->fill_cali_scripts();
            		this->fill_zprobe_scripts(true);
        		} else {
            		gcode->stream->printf("Change to Probe tool, change back when done\r\n");
            		atc_status = PROBE_FULL;
            		int old_tool = active_tool;
            		// change to probe tool
            		this->fill_drop_scripts(old_tool);
            		this->fill_pick_scripts(0);
            		this->fill_cali_scripts();
            		// do z probe
            		this->fill_zprobe_scripts(true);
            		// change back to last tool
            		this->fill_drop_scripts(0);
            		this->fill_pick_scripts(old_tool);
            		this->fill_cali_scripts();
        		}
        	}
		} else if (gcode->m == 495) {
			// Do Auto Leveling, change probe tool if needed
			if (gcode->has_letter('X') && gcode->has_letter('Y') && gcode->has_letter('A') && gcode->has_letter('B') && gcode->has_letter('I') && gcode->has_letter('J')) {
				float x_pos = gcode->get_value('X');
				float y_pos = gcode->get_value('Y');
	    		float x_size = gcode->get_value('A');
	    		float y_size = gcode->get_value('B');
	    		int x_grids = gcode->get_value('I');
	    		int y_grids = gcode->get_value('J');
	            THEROBOT->push_state();
				set_inner_playing(true);
	            this->clear_script_queue();
	        	if (active_tool == 0) {
	        		gcode->stream->printf("Do Auto Leveling right away\r\n");
	                atc_status = AUTOLEVEL;
	        	    this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	        	} else {
	                // save current position
	                THEROBOT->get_axis_position(last_pos, 3);
	        		if (active_tool < 0) {
	        			//
	            		gcode->stream->printf("Picking Probe tool and do Auto Leveling\r\n");
	            		atc_status = AUTOLEVEL_PICK;
	            		this->fill_pick_scripts(0);
	            		this->fill_cali_scripts();
	            		this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	        		} else {
	            		gcode->stream->printf("Change to Probe tool, change back when done\r\n");
	            		atc_status = AUTOLEVEL_FULL;
	            		int old_tool = active_tool;
	            		// change to probe tool
	            		this->fill_drop_scripts(old_tool);
	            		this->fill_pick_scripts(0);
	            		this->fill_cali_scripts();
	            		// do auto leveling
	            		this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	            		// change back to last tool
	            		this->fill_drop_scripts(0);
	            		this->fill_pick_scripts(old_tool);
	            		this->fill_cali_scripts();
	        		}
	        	}
			} else {
				gcode->stream->printf("ALARM: Miss AutoLevel Parameter: X/Y/A/B/I/J\r\n");
			}
		} else if (gcode->m == 496) {
			// Do z probe, then do Auto Leveling, change probe tool if needed
			if (gcode->has_letter('X') && gcode->has_letter('Y') && gcode->has_letter('A') && gcode->has_letter('B') && gcode->has_letter('I') && gcode->has_letter('J')) {
				float x_pos = gcode->get_value('X');
				float y_pos = gcode->get_value('Y');
	    		float x_size = gcode->get_value('A');
	    		float y_size = gcode->get_value('B');
	    		int x_grids = gcode->get_value('I');
	    		int y_grids = gcode->get_value('J');
	            THEROBOT->push_state();
				set_inner_playing(true);
	            this->clear_script_queue();
	        	if (active_tool == 0) {
	        		gcode->stream->printf("Do Probe and Auto Leveling right away\r\n");
	                atc_status = PROBELEVEL;
	                this->fill_zprobe_scripts(false);
	        	    this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	        	} else {
	                // save current position
	                THEROBOT->get_axis_position(last_pos, 3);
	        		if (active_tool < 0) {
	        			//
	            		gcode->stream->printf("Picking Probe tool and do Probe and Auto Leveling\r\n");
	            		atc_status = PROBELEVEL_PICK;
	            		this->fill_pick_scripts(0);
	            		this->fill_cali_scripts();
	            		this->fill_zprobe_scripts(true);
	            		this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	        		} else {
	            		gcode->stream->printf("Change to Probe tool, change back when done\r\n");
	            		atc_status = AUTOLEVEL_FULL;
	            		int old_tool = active_tool;
	            		// change to probe tool
	            		this->fill_drop_scripts(old_tool);
	            		this->fill_pick_scripts(0);
	            		this->fill_cali_scripts();
	            		// do z probe
	            		this->fill_zprobe_scripts(true);
	            		this->fill_autolevel_scripts(x_pos, y_pos, x_size, y_size, x_grids, y_grids);
	            		// change back to last tool
	            		this->fill_drop_scripts(0);
	            		this->fill_pick_scripts(old_tool);
	            		this->fill_cali_scripts();
	        		}
	        	}
			} else {
				gcode->stream->printf("ALARM: Miss AutoLevel Parameter: X/Y/A/B/I/J\r\n");
			}
		} else if (gcode->m == 498) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEKERNEL->streams->printf("EEPRROM Data: TOOL:%d\n", THEKERNEL->eeprom_data->TOOL);
				THEKERNEL->streams->printf("EEPRROM Data: TLO:%1.3f\n", THEKERNEL->eeprom_data->TLO);
				THEKERNEL->streams->printf("EEPRROM Data: G54: %1.3f, %1.3f, %1.3f\n", THEKERNEL->eeprom_data->G54[0], THEKERNEL->eeprom_data->G54[1], THEKERNEL->eeprom_data->G54[2]);
				THEKERNEL->streams->printf("EEPRROM Data: G28: %1.3f, %1.3f, %1.3f\n", THEKERNEL->eeprom_data->G28[0], THEKERNEL->eeprom_data->G28[1], THEKERNEL->eeprom_data->G28[2]);
			} else if (gcode->subcode == 2) {
				// Show EEPROM DATA
				THEKERNEL->erase_eeprom_data();
			}
		} else if ( gcode->m == 499) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				THEKERNEL->streams->printf("tool:%d ref:%1.3f cur:%1.3f offset:%1.3f\n", active_tool, ref_tool_mz, cur_tool_mz, tool_offset);
			} else if (gcode->subcode == 2) {
				THEKERNEL->streams->printf("probe -- mx:%1.1f my:%1.1f mz:%1.1f\n", probe_mx_mm, probe_my_mm, probe_mz_mm);
				for (int i = 0; i <=  tool_number; i ++) {
					THEKERNEL->streams->printf("tool%d -- mx:%1.1f my:%1.1f mz:%1.1f\n", atc_tools[i].num, atc_tools[i].mx_mm, atc_tools[i].my_mm, atc_tools[i].mz_mm);
				}
			}
		}
    }
}

void ATCHandler::on_main_loop(void *argument)
{
    if (this->atc_status != NONE) {
        if(THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("Kernel is halted!....\r\n");
            return;
        }

        while (!this->script_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->script_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->script_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->script_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

		if (this->atc_status != PROBE && this->atc_status != AUTOLEVEL) {
	        // return to saved x and y position
	        rapid_move(last_pos[0], last_pos[1], NAN);

	        // return to saved z position
	        // rapid_move(NAN, NAN, last_pos[2]);
		}

        this->atc_status = NONE;

		set_inner_playing(false);

        // save to config file to persist data
		// TODO

        // pop old state
        THEROBOT->pop_state();

		// if we were printing from an M command from pronterface we need to send this back
		THEKERNEL->streams->printf("Done ATC\r\n");
    }
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ATCHandler::rapid_move(float x, float y, float z)
{
    #define CMDLEN 128
    char *cmd= new char[CMDLEN]; // use heap here to reduce stack usage

    strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system

    if(!isnan(x)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " X%1.3f", x);
    }
    if(!isnan(y)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Y%1.3f", y);
    }
    if(!isnan(z)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Z%1.3f", z);
    }

    // send as a command line as may have multiple G codes in it
    struct SerialMessage message;
    message.message = cmd;
    delete [] cmd;

    message.stream = &(StreamOutput::NullStream);
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_idle();

}

void ATCHandler::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(get_tool_status_checksum)) {
    	if (this->active_tool >= 0) {
            struct tool_status *t= static_cast<tool_status*>(pdr->get_data_ptr());
            t->active_tool = this->active_tool;
            t->ref_tool_mz = this->ref_tool_mz;
            t->cur_tool_mz = this->cur_tool_mz;
            t->tool_offset = this->tool_offset;
            pdr->set_taken();
    	}
    } else if (pdr->second_element_is(get_atc_pin_status_checksum)) {
        char *data = static_cast<char *>(pdr->get_data_ptr());
        // cover endstop
        data[0] = (char)this->atc_home_info.pin.get();
        data[1] = (char)this->detector_info.detect_pin.get();
        pdr->set_taken();
    }
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_ref_tool_mz_checksum)) {
        this->ref_tool_mz = cur_tool_mz;
        this->tool_offset = 0.0;
        pdr->set_taken();
    }
}

bool ATCHandler::get_inner_playing() const
{
    void *returned_data;

    bool ok = PublicData::get_value( player_checksum, inner_playing_checksum, &returned_data );
    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return b;
    }
    return false;
}

void ATCHandler::set_inner_playing(bool inner_playing)
{
	PublicData::set_value( player_checksum, inner_playing_checksum, &inner_playing );
}

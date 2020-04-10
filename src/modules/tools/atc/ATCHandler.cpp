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

ATCHandler::ATCHandler()
{
	new_tool = 0;
    atc_status = NONE;
    atc_home_info.homed = false;
    atc_home_info.triggered = false;
    detector_info.triggered = false;
}

void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::fill_drop_scripts() {
	char buff[100];
	struct atc_tool *current_tool = &act_tools[active_tool];
    // lift z axis to atc start position
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	this->script_queue.push(buff);
    // move x and y to active tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%f Y%f", current_tool->mx_mm, current_tool->my_mm);
	this->script_queue.push(buff);
	// move around to see if tool rack is empty
	this->script_queue.push("M492.2");
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%f F%f", current_tool->mz_mm + safe_z_offset_mm, fast_z_rate);
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%f F%f", current_tool->mz_mm, slow_z_rate);
	this->script_queue.push(buff);
	// loose tool
	this->script_queue.push("M490.2");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	this->script_queue.push(buff);
	// move around to see if tool is dropped, halt if not
	this->script_queue.push("M492.1");
}

void ATCHandler::fill_pick_scripts() {
	char buff[100];
	struct atc_tool *current_tool = &act_tools[new_tool];
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	// move x and y to new tool position
	snprintf(buff, sizeof(buff), "G53 G0 X%f Y%f", current_tool->mx_mm, current_tool->my_mm);
	this->script_queue.push(buff);
	// move around to see if tool rack is filled
	this->script_queue.push("M492.1");
	// loose tool
	this->script_queue.push("M490.2");
    // drop z axis to z position with fast speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%f F%f", current_tool->mz_mm + safe_z_offset_mm, fast_z_rate);
	this->script_queue.push(buff);
    // drop z axis with slow speed
	snprintf(buff, sizeof(buff), "G53 G1 Z%f F%f", current_tool->mz_mm, slow_z_rate);
	this->script_queue.push(buff);
	// clamp tool
	this->script_queue.push("M490.1");
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	// move around to see if tool rack is empty, halt if not
	this->script_queue.push("M492.2");
}

void ATCHandler::fill_cali_scripts() {
	char buff[100];
	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	// move x and y to calibrate position
	snprintf(buff, sizeof(buff), "G53 G0 X%f Y%f", probe_mx_mm, probe_my_mm);

	// do calibrate with fast speed
	this->script_queue.push("G91 G38.2 Z-100 F200");
	// lift a bit
	this->script_queue.push("G91 G38.2 Z-100 F200");
	// do calibrate with slow speed
	this->script_queue.push("G91 G38.2 Z-100 F200");
	// save new tool offset

	// lift z to safe position with fast speed
	snprintf(buff, sizeof(buff), "G53 G0 Z%f", this->safe_z_mm);
	this->script_queue.push(buff);

	// goto saved position
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
}

void ATCHandler::on_config_reload(void *argument)
{
	atc_home_info.pin.from_string( THEKERNEL->config->value(atc_checksum, endstop_pin_checksum)->by_default("nc" )->as_string())->as_input();
	atc_home_info.debounce_ms    = THEKERNEL->config->value(atc_checksum, debounce_ms_checksum)->by_default(1  )->as_number();
	atc_home_info.max_travel    = THEKERNEL->config->value(atc_checksum, max_travel_mm_checksum)->by_default(5  )->as_number();
	atc_home_info.retract    = THEKERNEL->config->value(atc_checksum, homing_retract_mm_checksum)->by_default(3  )->as_number();
	atc_home_info.action_dist    = THEKERNEL->config->value(atc_checksum, action_mm_checksum)->by_default(1  )->as_number();
	atc_home_info.homing_rate    = THEKERNEL->config->value(atc_checksum, homing_rate_mm_s_checksum)->by_default(1  )->as_number();
	atc_home_info.action_rate    = THEKERNEL->config->value(atc_checksum, action_rate_mm_s_checksum)->by_default(1  )->as_number();

	detector_info.detect_pin.from_string( THEKERNEL->config->value(atc_checksum, detector_checksum, detect_pin_checksum)->by_default("nc" )->as_string())->as_input();
	detector_info.detect_rate = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_rate_mm_s_checksum)->by_default(1  )->as_number();
	detector_info.detect_travel = THEKERNEL->config->value(atc_checksum, detector_checksum, detect_travel_mm_checksum)->by_default(3  )->as_number();

	this->safe_z_mm = THEKERNEL->config->value(atc_checksum, safe_z_checksum)->by_default(-10)->as_number();
	this->safe_z_offset_mm = THEKERNEL->config->value(atc_checksum, safe_z_offset_checksum)->by_default(10)->as_number();
	this->fast_z_rate = THEKERNEL->config->value(atc_checksum, fast_z_rate_checksum)->by_default(500)->as_number();
	this->slow_z_rate = THEKERNEL->config->value(atc_checksum, slow_z_rate_checksum)->by_default(60)->as_number();
	this->active_tool = THEKERNEL->config->value(atc_checksum, active_tool_checksum)->by_default(0)->as_number();
	this->tool_number = THEKERNEL->config->value(atc_checksum, tool_number_checksum)->by_default(6)->as_number();

	act_tools.clear();
	for (int i = 0; i <=  tool_number; i ++) {
		struct atc_tool tool;
		tool.num = i;
		tool.mx_mm = THEKERNEL->config->value(atc_checksum, get_checksum("tool" + i), mx_mm_checksum)->by_default(-10  )->as_number();
		tool.mx_mm = THEKERNEL->config->value(atc_checksum, get_checksum("tool" + i), my_mm_checksum)->by_default(-10  )->as_number();
		tool.mx_mm = THEKERNEL->config->value(atc_checksum, get_checksum("tool" + i), mz_mm_checksum)->by_default(-10  )->as_number();
		act_tools.push_back(tool);
	}
}

void ATCHandler::on_halt(void* argument)
{
    if(argument == nullptr && this->atc_status != NONE ) {
        this->atc_status = NONE;
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
    bool ok = PublicData::set_value(switch_checksum, detector_checksum, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("ERROR: Failed switch on detector switch.\r\n");
        return false;
    }
    detecting = true;

	float delta[Y_AXIS + 1];
	for (size_t i = 0; i <= Y_AXIS; i++) delta[i] = 0;
	delta[Y_AXIS]= detector_info.detect_travel; // we go the max
	THEROBOT->delta_move(delta, detector_info.detect_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
	if(THEKERNEL->is_halted()) return false;

	detecting = false;
	// switch off detector
	switch_state = false;
    ok = PublicData::set_value(switch_checksum, detector_checksum, state_checksum, &switch_state);
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
    atc_home_info.homed = false;
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

	atc_home_info.homed = true;

}

void ATCHandler::clamp_tool()
{
	if (!atc_home_info.homed) {
		home_clamp();
	}
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = -atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
}

void ATCHandler::loose_tool()
{
	if (!atc_home_info.homed) {
		home_clamp();
	}
	float delta[ATC_AXIS + 1];
	for (size_t i = 0; i <= ATC_AXIS; i++) delta[i] = 0;
	delta[4] = atc_home_info.action_dist;
	THEROBOT->delta_move(delta, atc_home_info.action_rate, ATC_AXIS + 1);
	// wait for it
	THECONVEYOR->wait_for_idle();
}

void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
    	if (gcode->m == 6 && gcode->has_letter('T')) {
    		if (atc_status != NONE) {
    			gcode->stream->printf("ATC already begun\r\n");
    			return;
    		}
            new_tool = gcode->get_value('T');
            if (new_tool > this->tool_number) {
            	gcode->stream->printf("T%d invalid tool\r\n", new_tool);
            } else {
            	if (new_tool != active_tool) {
                    // push old state
                    THEROBOT->push_state();
                    set_inner_playing(true);
                    this->clear_script_queue();
                	if (this->active_tool < 0) {
                		// just pick up tool
                		atc_status = PICK;
                		this->fill_pick_scripts();
                		this->fill_cali_scripts();
                	} else if (new_tool < 0) {
                		// just drop tool
                		atc_status = DROP;
                		this->fill_drop_scripts();
                	} else {
                		// full atc progress
                		atc_status = FULL;
                	    this->fill_drop_scripts();
                	    this->fill_pick_scripts();
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
            set_inner_playing(true);
            this->clear_script_queue();
            atc_status = CALI;
    	    this->fill_cali_scripts();
			set_inner_playing(true);
		} else if (gcode->m == 492) {
			if (gcode->subcode == 0 || gcode->subcode == 1) {
				// check true
				if (!laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");
				}
			} else if (gcode->subcode == 2) {
				// check false
				if (laser_detect()) {
			        THEKERNEL->call_event(ON_HALT, nullptr);
			        THEKERNEL->streams->printf("ERROR: Tool confliction occured, please check tool rack!\n");

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
			this->script_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

        // update tool info
        if (this->atc_status == DROP || this->atc_status == PICK || this->atc_status == FULL) {
    		this->active_tool = this->new_tool;
        }
		set_inner_playing(false);
        this->atc_status = NONE;
        // save to config file to persist data
		// TODO
        // pop old state
        THEROBOT->pop_state();

		// if we were printing from an M command from pronterface we need to send this back
		THEKERNEL->streams->printf("Done ATC\r\n");
    }
}

void ATCHandler::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(get_active_tool_checksum)) {
    	if (this->active_tool >= 0) {
    		pdr->set_data_ptr(&act_tools[this->active_tool]);
            pdr->set_taken();
    	}
    }
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;
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

#include "ATCHandler.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "SlowTicker.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "SDFAT.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "ExtruderPublicAccess.h"

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define ATC_AXIS 4
#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

#define atc_checksum            	CHECKSUM("atc")
#define enable_checksum          	CHECKSUM("enable")
#define endstop_pin_checksum      	CHECKSUM("endstop_pin")
#define debounce_ms_checksum      	CHECKSUM("debounce_ms")
#define max_travel_mm_checksum    	CHECKSUM("max_travel_mm")
#define homing_retract_mm_checksum  CHECKSUM("homing_retract_mm")
#define action_mm_checksum      	CHECKSUM("action_mm")
#define homing_rate_mm_s_checksum   CHECKSUM("homing_rate_mm_s")
#define action_rate_mm_s_checksum   CHECKSUM("action_rate_mm_s")

ATCHandler::ATCHandler() {
    this->playing_atc = false;
    atc_home_info.homed = false;
    atc_home_info.triggered = false;
    init_script();
}

void ATCHandler::init_script() {

	// init clamp tool script
	clamp_tool_script.push_back("M491");

	// init loose tool script
	loose_tool_script.push_back("M492");

	// init atc script
	atc_script.push_back("M1000 T1");
	atc_script.push_back("M1000 T2");
	atc_script.push_back("M1000 T3");
	atc_script.push_back("M1000 T4");

	// init z probe script
	zprobe_script.push_back("");
	zprobe_script.push_back("");
	zprobe_script.push_back("");
	zprobe_script.push_back("");

}

string ATCHandler::fill_vars(string source) {

	return "";
}

void ATCHandler::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( atc_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    on_config_reload(this);

    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    atc_homing = false;
    THEKERNEL->slow_ticker->attach(1000, this, &ATCHandler::read_endstop);
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
}

void ATCHandler::on_halt(void* argument)
{
    if(argument == nullptr && this->playing_atc ) {
        stop_atc("1", &(StreamOutput::NullStream));
	}

}

void ATCHandler::stop_atc( string parameters, StreamOutput *stream )
{
    if(!playing_atc) {
        stream->printf("Not currently playing atc\r\n");
        return;
    }
    playing_atc = false;
    if(parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THEKERNEL->conveyor->flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT->reset_position_from_current_actuator_position();
        stream->printf("Stoped playing atc\r\n");
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

	atc_homing = false;

    if (!atc_home_info.triggered) {
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->streams->printf("ERROR: Homing atc failed - check the atc max travel settings\n");
        return;
    }

    THEROBOT->reset_position_from_current_actuator_position();

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
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
    	if (gcode->m == 6 && gcode->has_letter('T')) {
			if(this->playing_atc) {
				gcode->stream->printf("Currently playing atc, abort\r\n");
				return;
			}
			played_cnt = 0;
			gcode->stream->printf("playing atc, new tool: %s\r\n", args.c_str());
			this->playing_atc = true;

		} else if (gcode->m == 490)  {
			if (gcode->subcode == 0) {
				// home tool change
				gcode->stream->printf("clamp home start!\r\n");
				home_clamp();
				gcode->stream->printf("clamp homed!\r\n");

			} else if (gcode->subcode == 1) {
				// clamp tool
				gcode->stream->printf("tool clamp start!\r\n");
				clamp_tool();
				gcode->stream->printf("tool clamped!\r\n");

			} else if (gcode->subcode == 2) {
				// loose tool
				gcode->stream->printf("tool loose start!\r\n");
				loose_tool();
				gcode->stream->printf("tool loosed!\r\n");
			}
		} else if (gcode->m == 1000) {
			gcode->stream->printf("test: %s\r\n", args.c_str());
    	}
    }

}

void ATCHandler::on_main_loop(void *argument)
{
    if( this->playing_atc ) {
        if(THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("Kernel is halted!....\r\n");
            return;
        }

        while(played_cnt < atc_script.size()) {
        	THEKERNEL->streams->printf("%s\r\n", this->atc_script[this->played_cnt].c_str());
			struct SerialMessage message;
			message.message = this->atc_script[this->played_cnt];
			message.stream = THEKERNEL->serial;

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            played_cnt ++;
            return;
        }

        this->playing_atc = false;
        played_cnt = 0;

		// if we were printing from an M command from pronterface we need to send this back
		THEKERNEL->streams->printf("Done atc\r\n");
    }
}


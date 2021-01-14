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
#include "SpindlePublicAccess.h"

#include "FileStream.h"
#include <math.h>

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

#define extractor_checksum          CHECKSUM("extractor")
#define safe_z_checksum      		CHECKSUM("safe_z")
#define z_rate_work_checksum      	CHECKSUM("z_rate_work")
#define x_interval_checksum    		CHECKSUM("x_interval")
#define z_pos_work_checksum  		CHECKSUM("z_pos_work")
#define x_pos_origin_checksum   	CHECKSUM("x_pos_origin")
#define y_pos_waste_checksum      	CHECKSUM("y_pos_waste")
#define y_pos_gather_checksum   	CHECKSUM("y_pos_gather")


ATCHandler::ATCHandler()
{
	extractor_status = NONE;
}

void ATCHandler::hmi_load_tests(string parameters, StreamOutput *stream) {
	int index = 1;
	map<string, vector<struct reagent>>::iterator it;
	for (it = this->tests.begin(); it != this->tests.end(); it++) {
		THEKERNEL->streams->printf("bTest%d.txt=\"%s\"\xff\xff\xff", index, it->first.c_str());
		index ++;
		if (index > 6) {
			break;
		}
	}
	// disable lines
	for (; index <= 6; index ++) {
		THEKERNEL->streams->printf("vis bTest%d,0\xff\xff\xff", index);
	}
}

void ATCHandler::hmi_load_test_info(string parameters, StreamOutput *stream) {
    string test_name = shift_parameter( parameters );
    map<string, vector<struct reagent>>::iterator it = this->tests.find(test_name);
	if (it != this->tests.end()) {
		unsigned int index = 1;
		for (; index <= it->second.size(); index ++) {
			THEKERNEL->streams->printf("tName%d.txt=\"%s(%dmin, %d%%)\"\xff\xff\xff",
					index, it->second[index].name.c_str(), it->second[index].minutes, it->second[index].pressure);
		}
		// disable lines
		for (; index <= 5; index ++) {
			THEKERNEL->streams->printf("vis tName%d,0\xff\xff\xff", index);
			THEKERNEL->streams->printf("vis j%d,0\xff\xff\xff", index);
		}
	}
}

// update data to HMI screen
void ATCHandler::hmi_update_test(string parameters, StreamOutput *stream) {

}

void ATCHandler::hmi_update_step(string parameters, StreamOutput *stream) {

}

// execute command from HMI screen
void ATCHandler::hmi_home(string parameters, StreamOutput *stream) {
	struct SerialMessage message;
	message.message = "$H";
	message.stream = &(StreamOutput::NullStream);
	message.line = 0;
	THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
}

void ATCHandler::hmi_test(string parameters, StreamOutput *stream) {

}

void ATCHandler::hmi_step(string parameters, StreamOutput *stream) {

}

void ATCHandler::hmi_pause(string parameters, StreamOutput *stream) {

}

void ATCHandler::hmi_stop(string parameters, StreamOutput *stream) {

}


void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::fill_test_scripts(string test_name) {
	this->extractor_status = TEST;

	char buff[100];
	// home first
	this->script_queue.push("$H");

	// wait for idle
	this->script_queue.push("M490");

	// loop add step info
    map<string, vector<struct reagent>>::iterator it = this->tests.find(test_name);
	if (it != this->tests.end()) {
		for (unsigned int i = 0; i < it->second.size(); i ++) {
			// z move to safe position
			snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
			this->script_queue.push(buff);

			// move x to work position
			snprintf(buff, sizeof(buff), "G53 G0 X%.3f", this->x_pos_origin + this->x_interval * i);
			this->script_queue.push(buff);

			// move y to position
			if (i < it->second.size() - 1) {
				snprintf(buff, sizeof(buff), "G53 G0 Y%.3f", this->y_pos_waste);
				this->script_queue.push(buff);
			} else {
				snprintf(buff, sizeof(buff), "G53 G0 Y%.3f", this->y_pos_gather);
				this->script_queue.push(buff);
			}

			// move z to work position
			snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", this->z_pos_work, this->z_rate_work);
			this->script_queue.push(buff);

			// turn on motor
			snprintf(buff, sizeof(buff), "M3 S%d", it->second[i].pressure);
			this->script_queue.push(buff);

			// start pressure
			this->script_queue.push("M490.1 S%d", i);

			// wait x minutes
			snprintf(buff, sizeof(buff), "G4 P%d", it->second[i].minutes * 60);
			this->script_queue.push(buff);

			// finish pressure
			this->script_queue.push("M490.2 S%d", i);

		}
	}

	// z move to safe position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
	this->script_queue.push(buff);

}

void ATCHandler::fill_step_scripts(int index, int minutes, int pressure) {
	this->extractor_status = STEP;

}

void ATCHandler::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_SECOND_TICK);

    this->on_config_reload(this);
    // load test config
    this->load_tests();
}

void ATCHandler::on_config_reload(void *argument)
{
	this->safe_z = THEKERNEL->config->value(extractor_checksum, safe_z_checksum)->by_default(-5)->as_number();
	this->z_rate_work = THEKERNEL->config->value(extractor_checksum, z_rate_work_checksum)->by_default(200)->as_number();
	this->x_interval = THEKERNEL->config->value(extractor_checksum, x_interval_checksum)->by_default(13)->as_number();
	this->z_pos_work = THEKERNEL->config->value(extractor_checksum, z_pos_work_checksum)->by_default(-30)->as_number();
	this->x_pos_origin = THEKERNEL->config->value(extractor_checksum, x_pos_origin_checksum)->by_default(30)->as_number();
	this->y_pos_waste = THEKERNEL->config->value(extractor_checksum, y_pos_waste_checksum)->by_default(30)->as_number();
	this->y_pos_gather = THEKERNEL->config->value(extractor_checksum, y_pos_gather_checksum)->by_default(40)->as_number();
}

void ATCHandler::load_tests() {
	FILE* fp = fopen("/sd/test.txt", "r");
	if (fp == NULL) {
		THEKERNEL->streams->printf("ERROR: Unable to load test config\r\n");
		return;
	}

	char buf[130]; // lines upto 128 characters are allowed, anything longer is discarded
	bool discard = false;
	string line = "";
	string test_name = "";
	size_t left_bracket;
	size_t right_bracket;
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        int len = strlen(buf);
        if (len == 0) continue; // empty line? should not be possible
        if (buf[len - 1] == '\n' || feof(fp)) {
            if(discard) { // we are discarding a long line
                discard = false;
                continue;
            }
            if(len == 1) continue; // empty line
            // parse line
            line = buf;
            left_bracket = line.find_first_of("[", 0);
            right_bracket = line.find_first_of("]", 0);
            if (left_bracket != string::npos && right_bracket != string::npos) {
            	test_name = line.substr(left_bracket + 1, right_bracket - 1);
            	this->tests.insert(pair<string, vector<struct reagent>>(test_name, vector<struct reagent>()));
            } else {
            	vector<string> reagent_info = split(buf, ',');
            	if (reagent_info.size() == 3 && !test_name.empty()) {
            		struct reagent reagent_;
            		reagent_.name = reagent_info[0];
            		reagent_.minutes = atoi(reagent_info[1].c_str());
            		reagent_.pressure = atoi(reagent_info[2].c_str());
            		this->tests[test_name].push_back(reagent_);
            	}
            }
        } else {
            // discard long line
            discard = true;
        }
    }

	fclose(fp);
	fp = NULL;
}

void ATCHandler::on_halt(void* argument)
{
    if (argument == nullptr ) {
        this->clear_script_queue();
        this->set_inner_playing(false);
        this->extractor_status = NONE;
	}
}
void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
		if (gcode->m == 490)  {
			if (gcode->subcode == 0) {
				// wait for idle
			    THEKERNEL->conveyor->wait_for_idle();
			} else if (gcode->subcode == 1) {

			} else if (gcode->subcode == 2) {

			}
		}
    }
}

void ATCHandler::on_main_loop(void *argument)
{
    if (this->extractor_status != NONE) {
        if (THEKERNEL->is_halted()) {
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

        this->extractor_status = NONE;

		set_inner_playing(false);

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

// When a new line is received, check if it is a command, and if it is, act upon it
void ATCHandler::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter( possible_command );

    if (cmd == "hmi") {
    	cmd = shift_parameter( possible_command );
        if (cmd == "load_tests"){
            this->hmi_load_tests( possible_command, new_message.stream );
        } else if (cmd == "load_test_info"){
            this->hmi_load_test_info( possible_command, new_message.stream );
        } else if (cmd == "update_test") {
            this->hmi_update_test( possible_command, new_message.stream );
        } else if (cmd == "update_step") {
            this->hmi_update_step( possible_command, new_message.stream );
        } else if (cmd == "home") {
            this->hmi_home( possible_command, new_message.stream );
        } else if (cmd == "test") {
        	this->hmi_test( possible_command, new_message.stream );
        } else if (cmd == "step") {
        	this->hmi_step( possible_command, new_message.stream );
		} else if (cmd == "pause") {
			this->hmi_pause( possible_command, new_message.stream );
		} else if (cmd == "stop") {
			this->hmi_stop( possible_command, new_message.stream );
		} else {
			THEKERNEL->streams->printf("ALARM: Invalid hmi command: %s\r\n", cmd.c_str());
		}
    }
}

void ATCHandler::on_second_tick(void *)
{

}

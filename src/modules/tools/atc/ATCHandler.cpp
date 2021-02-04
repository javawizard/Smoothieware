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
#include "us_ticker_api.h"

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
#define z_pos_work_checksum  		CHECKSUM("z_pos_work")
#define y_pos_waste_checksum      	CHECKSUM("y_pos_waste")
#define y_pos_gather_checksum   	CHECKSUM("y_pos_gather")
#define y_pos_clear_checksum   		CHECKSUM("y_pos_clear")
#define pump_start_power_checksum   CHECKSUM("pump_start_power")
#define pump_start_seconds_checksum CHECKSUM("pump_start_seconds")
#define pump_acc_seconds_checksum	CHECKSUM("pump_acc_seconds")


ATCHandler::ATCHandler()
{
	extractor_status = NONE;
    curr_test_name = "";
    curr_total_minutes = 0;
    curr_total_start_ticket = 0;
    curr_reagent_index = 0;
    curr_reagent_minutes = 0;
    curr_reagent_start_ticket = 0;
}

void ATCHandler::hmi_do_nothing() {
	// send empty command for clean
	THEKERNEL->streams->printf("\xff\xff\xff");
}

void ATCHandler::hmi_load_tests(string parameters, StreamOutput *stream) {
	int index = 1;
	map<string, vector<struct reagent>>::iterator it;
	this->hmi_do_nothing();
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
    // send empty command for clean
    this->hmi_do_nothing();

    map<string, vector<struct reagent>>::iterator it = this->tests.find(test_name);
	if (it != this->tests.end()) {
		unsigned int index = 1;
		for (; index <= it->second.size(); index ++) {
			THEKERNEL->streams->printf("tName%d.txt=\"%s(%dmin, %d%%)\"\xff\xff\xff",
					index, it->second[index-1].name.c_str(), it->second[index-1].minutes, it->second[index-1].pressure);
		}
		// disable lines
		for (; index <= 5; index ++) {
			THEKERNEL->streams->printf("vis tName%d,0\xff\xff\xff", index);
			THEKERNEL->streams->printf("vis j%d,0\xff\xff\xff", index);
		}
	}
}

// send finish status
void ATCHandler::hmi_send_finish() {
	// send empty command for clean
	this->hmi_do_nothing();
	THEKERNEL->streams->printf("vars.vFinish.val=1\xff\xff\xff");
}

// update data to HMI screen
void ATCHandler::hmi_update(StreamOutput *stream) {
	// send empty command for clean
	this->hmi_do_nothing();

	if (this->extractor_status == NONE) {
		THEKERNEL->streams->printf("vStatus.txt=\"NONE\"\xff\xff\xff");
	} else if (this->extractor_status == TEST) {
		THEKERNEL->streams->printf("vStatus.txt=\"TEST\"\xff\xff\xff");
		uint32_t curr_ticket = us_ticker_read();
		if (curr_reagent_minutes > 0) {

			int percentage = (curr_ticket - curr_reagent_start_ticket) * 100.0 / (curr_reagent_minutes * 60 * 1000000);
			if (percentage > 100) percentage = 100;
			for (int i = 0; i < this->curr_reagent_index; i ++) {
				THEKERNEL->streams->printf("j%d.val=100\xff\xff\xff", i + 1);
			}
			THEKERNEL->streams->printf("j%d.val=%d\xff\xff\xff", curr_reagent_index + 1, percentage);
		}
		uint32_t run_seconds = (curr_ticket - curr_total_start_ticket) / 1000000;
		uint32_t left_seconds = curr_total_minutes * 60 - run_seconds;
		THEKERNEL->streams->printf("tRunTime.txt=\"%s\"\xff\xff\xff", format_seconds(run_seconds).c_str());
		THEKERNEL->streams->printf("tLeftTime.txt=\"%s\"\xff\xff\xff", format_seconds(left_seconds).c_str());
	} else if (this->extractor_status == STEP) {
		THEKERNEL->streams->printf("vStatus.txt=\"STEP\"\xff\xff\xff");
		uint32_t curr_ticket = us_ticker_read();
		if ( curr_reagent_minutes > 0) {
			THEKERNEL->streams->printf("vStatus.txt=\"STEP\"\xff\xff\xff");

			int percentage = (curr_ticket - curr_reagent_start_ticket) * 100.0 / (curr_reagent_minutes * 60 * 1000000);
			if (percentage > 100) percentage = 100;
			THEKERNEL->streams->printf("j1.val=%d\xff\xff\xff", percentage);
			//
			uint32_t run_seconds = (curr_ticket - curr_reagent_start_ticket) / 1000000;
			uint32_t left_seconds = curr_reagent_minutes * 60 - run_seconds;
			THEKERNEL->streams->printf("tRunTime.txt=\"%s\"\xff\xff\xff", format_seconds(run_seconds).c_str());
			THEKERNEL->streams->printf("tLeftTime.txt=\"%s\"\xff\xff\xff", format_seconds(left_seconds).c_str());
		}
	}
}

void ATCHandler::hmi_test(string parameters, StreamOutput *stream) {
	// check if already running
	if (this->extractor_status != NONE) {
		return;
	}

	this->curr_reagent_minutes = 0;
    string test_name = shift_parameter( parameters );
    map<string, vector<struct reagent>>::iterator it = this->tests.find(test_name);
	if (it != this->tests.end()) {
		this->curr_test_name = test_name;
		this->fill_test_scripts();
	}
}

void ATCHandler::hmi_step(string parameters, StreamOutput *stream) {
	// check if already running
	if (this->extractor_status != NONE) {
		return;
	}

	this->curr_reagent_minutes = 0;
    string index = shift_parameter( parameters );
    string minutes = shift_parameter( parameters );
    string pressure = shift_parameter( parameters );
    string waste = shift_parameter( parameters );
    if (!index.empty() && !minutes.empty() && !pressure.empty() && !waste.empty()) {
    	fill_step_scripts(atoi(index.c_str()), atoi(minutes.c_str()), atoi(pressure.c_str()), atoi(waste.c_str()));
    }
}

void ATCHandler::hmi_lift_z(string parameters, StreamOutput *stream) {
	THEKERNEL->conveyor->wait_for_idle();
	// goto safe z
	rapid_move(NAN, NAN, this->safe_z);
	// disable motors
    struct SerialMessage message;
    message.message = "M84";
    message.stream = &(StreamOutput::NullStream);
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}

void ATCHandler::clear_script_queue(){
	while (!this->script_queue.empty()) {
		this->script_queue.pop();
	}
}

void ATCHandler::fill_test_scripts() {
	this->extractor_status = TEST;

	char buff[100];
	// home first
	this->script_queue.push("$H");

	// loop add step info
    map<string, vector<struct reagent>>::iterator it = this->tests.find(this->curr_test_name);
	if (it != this->tests.end()) {
		// init variables and wait for idle
		this->script_queue.push("M490");

		for (unsigned int i = 0; i < it->second.size(); i ++) {
			// z move to safe position
			snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
			this->script_queue.push(buff);

			// move x, y to work position
			snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%0.3f", this->x_pos[i],
					i < it->second.size() - 1 ? this->y_pos_waste : this->y_pos_gather);
			this->script_queue.push(buff);

			// move z to work position
			snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", this->z_pos_work, this->z_rate_work);
			this->script_queue.push(buff);

			// start reagent timing
			snprintf(buff, sizeof(buff), "M491 S%d P%d", i, it->second[i].minutes);
			this->script_queue.push(buff);

			// turn on motor on start power
			snprintf(buff, sizeof(buff), "M3 S%d", int(this->pump_start_power * 255.0 / 100));
			this->script_queue.push(buff);

			// wait start second
			snprintf(buff, sizeof(buff), "G4 P%d", int(this->pump_start_seconds));
			this->script_queue.push(buff);

			// loop to change motor power
			float power_interval = (it->second[i].pressure - this->pump_start_power) / this->pump_acc_seconds;
			for (int i = 0; i < this->pump_acc_seconds; i ++) {
				// turn on motor and change power
				snprintf(buff, sizeof(buff), "M3 S%d", int((this->pump_start_power + power_interval * i) * 255.0 / 100));
				this->script_queue.push(buff);

				// wait for 1 seconds
				this->script_queue.push("G4 P1");
			}

			// turn on motor on normal power
			snprintf(buff, sizeof(buff), "M3 S%d", int(it->second[i].pressure * 255.0 / 100));
			this->script_queue.push(buff);

			// wait x minutes - slow second
			snprintf(buff, sizeof(buff), "G4 P%d", it->second[i].minutes * 60 - int(this->pump_start_seconds + this->pump_acc_seconds));
			this->script_queue.push(buff);

			// turn off motor
			this->script_queue.push("M5");

		}
	}

	// z move to safe position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
	this->script_queue.push(buff);

	// y move to clear position
	snprintf(buff, sizeof(buff), "G53 G0 Y%.3f", this->y_pos_clear);
	this->script_queue.push(buff);

	// turn off all motors
	this->script_queue.push("M84");

	// send finish flag
	this->script_queue.push("M493");
}

void ATCHandler::fill_step_scripts(int index, int minutes, int pressure, bool waste) {
	this->extractor_status = STEP;

	char buff[100];
	// home first
	this->script_queue.push("$H");

	// z move to safe position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
	this->script_queue.push(buff);

	// move x and y to work position
	snprintf(buff, sizeof(buff), "G53 G0 X%.3f Y%.3f",
			this->x_pos[index - 1], waste ? this->y_pos_waste : this->y_pos_gather);
	this->script_queue.push(buff);

	// move z to work position
	snprintf(buff, sizeof(buff), "G53 G1 Z%.3f F%.3f", this->z_pos_work, this->z_rate_work);
	this->script_queue.push(buff);

	// start reagent timing
	snprintf(buff, sizeof(buff), "M491 S%d P%d", index - 1, minutes);
	this->script_queue.push(buff);

	// turn on motor on start power
	snprintf(buff, sizeof(buff), "M3 S%d", int(this->pump_start_power * 255.0 / 100));
	this->script_queue.push(buff);

	// wait start second
	snprintf(buff, sizeof(buff), "G4 P%d", int(this->pump_start_seconds));
	this->script_queue.push(buff);


	// loop to change motor power
	float power_interval = (pressure - this->pump_start_power) / this->pump_acc_seconds;
	for (int i = 0; i < this->pump_acc_seconds; i ++) {
		// turn on motor and change power
		snprintf(buff, sizeof(buff), "M3 S%d", int((this->pump_start_power + power_interval * i) * 255.0 / 100));
		this->script_queue.push(buff);

		// wait for 1 seconds
		this->script_queue.push("G4 P1");
	}

	// turn on motor on normal power
	snprintf(buff, sizeof(buff), "M3 S%d", int(pressure * 255.0 / 100));
	this->script_queue.push(buff);

	// wait x minutes - slow second
	snprintf(buff, sizeof(buff), "G4 P%d", minutes * 60 - int(this->pump_start_seconds + this->pump_acc_seconds));
	this->script_queue.push(buff);


	// turn off motor
	this->script_queue.push("M5");


	// z move to safe position
	snprintf(buff, sizeof(buff), "G53 G0 Z%.3f", this->safe_z);
	this->script_queue.push(buff);

	// turn off all motors
	this->script_queue.push("M84");

	// send finish flag
	this->script_queue.push("M493");

}

void ATCHandler::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    this->on_config_reload(this);
    // load test config
    this->load_tests();
}

void ATCHandler::on_config_reload(void *argument)
{
	char buff[10];

	this->safe_z = THEKERNEL->config->value(extractor_checksum, safe_z_checksum)->by_default(-5)->as_number();
	this->z_rate_work = THEKERNEL->config->value(extractor_checksum, z_rate_work_checksum)->by_default(200)->as_number();
	this->z_pos_work = THEKERNEL->config->value(extractor_checksum, z_pos_work_checksum)->by_default(-30)->as_number();

	for (int i = 0; i < 5; i ++) {
		snprintf(buff, sizeof(buff), "x_pos%d", i + 1);
		this->x_pos[i] = THEKERNEL->config->value(extractor_checksum, get_checksum(buff))->by_default(80)->as_number();
	}

	this->y_pos_waste = THEKERNEL->config->value(extractor_checksum, y_pos_waste_checksum)->by_default(30)->as_number();
	this->y_pos_gather = THEKERNEL->config->value(extractor_checksum, y_pos_gather_checksum)->by_default(40)->as_number();
	this->y_pos_clear = THEKERNEL->config->value(extractor_checksum, y_pos_clear_checksum)->by_default(40)->as_number();

	this->pump_start_power = THEKERNEL->config->value(extractor_checksum, pump_start_power_checksum)->by_default(10)->as_number();
	this->pump_start_seconds = THEKERNEL->config->value(extractor_checksum, pump_start_seconds_checksum)->by_default(20)->as_number();
	this->pump_acc_seconds = THEKERNEL->config->value(extractor_checksum, pump_acc_seconds_checksum)->by_default(30)->as_number();
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
        this->curr_reagent_minutes = 0;

	}
}

void ATCHandler::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m) {
		if (gcode->m == 490)  {
			// init global variables and wait for idle
		    map<string, vector<struct reagent>>::iterator it = this->tests.find(curr_test_name);
			if (it != this->tests.end()) {
			    this->curr_total_minutes = 0;
				for (unsigned int i = 0; i < it->second.size(); i ++) {
					this->curr_total_minutes += it->second[i].minutes;
				}
			}
		    this->curr_total_start_ticket = us_ticker_read();
			THEKERNEL->conveyor->wait_for_idle();
		} else if (gcode->m == 491) {
			// init reagent variables
            if (gcode->has_letter('S')) {
            	curr_reagent_index = gcode->get_value('S');
            }
            if (gcode->has_letter('P')) {
            	curr_reagent_minutes = gcode->get_value('P');
            }
            curr_reagent_start_ticket = us_ticker_read();
		} else if (gcode->m == 492) {
			// reserve
		} else if (gcode->m == 493) {
			this->hmi_send_finish();
		}
    }
}

void ATCHandler::on_main_loop(void *argument)
{
    if (this->extractor_status != NONE) {
        if (THEKERNEL->is_halted()) {
            return;
        }

        while (!this->script_queue.empty()) {
			struct SerialMessage message;
			message.message = this->script_queue.front();
			message.stream = &(StreamOutput::NullStream);
			message.line = 0;
			this->script_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

        this->extractor_status = NONE;
        this->curr_reagent_minutes = 0;

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
        } else if (cmd == "test") {
        	this->hmi_test( possible_command, new_message.stream );
        } else if (cmd == "step") {
        	this->hmi_step( possible_command, new_message.stream );
        } else if (cmd == "lift_z") {
        	this->hmi_lift_z( possible_command, new_message.stream );
		} else {
            this->hmi_do_nothing();
		}
    }
}

void ATCHandler::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(query_hmi_checksum)) {
    	StreamOutput *stream = static_cast<StreamOutput *>(pdr->get_data_ptr());
    	this->hmi_update(stream);
        pdr->set_taken();
    }
}

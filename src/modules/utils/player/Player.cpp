/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "Robot.h"
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
#include "md5.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "StepTicker.h"
#include "Block.h"

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define home_on_boot_checksum             CHECKSUM("home_on_boot")
#define on_boot_gcode_checksum            CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum     CHECKSUM("on_boot_gcode_enable")
#define after_suspend_gcode_checksum      CHECKSUM("after_suspend_gcode")
#define before_resume_gcode_checksum      CHECKSUM("before_resume_gcode")
#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")

extern SDFAT mounter;

// used for XMODEM
#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x16 //0x18
#define CTRLZ 0x1A

#define MAXRETRANS 10
#define TIMEOUT_MS 100


Player::Player()
{
    this->playing_file = false;
    this->current_file_handler = nullptr;
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->inner_playing = false;
}

void Player::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(false)->as_bool();

    this->home_on_boot = THEKERNEL->config->value(home_on_boot_checksum)->by_default(true)->as_bool();

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->by_default("")->as_string();
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->by_default("")->as_string();
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();
}

void Player::on_halt(void* argument)
{
    this->clear_buffered_queue();

    if(argument == nullptr && this->playing_file ) {
        abort_command("1", &(StreamOutput::NullStream));
	}

	if(argument == nullptr && (THEKERNEL->is_suspending() || THEKERNEL->is_waiting())) {
		// clean up from suspend
		THEKERNEL->set_waiting(false);
		THEKERNEL->set_suspending(false);
		THEROBOT->pop_state();
		THEKERNEL->streams->printf("Suspend cleared\n");
	}
}

void Player::on_second_tick(void *)
{
    if(this->playing_file) this->elapsed_secs++;
}

// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            mounter.remount();
            gcode->stream->printf("SD card ok\r\n");

        } else if (gcode->m == 23) { // select file
            this->filename = "/sd/" + args; // filename is whatever is in args
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }
            this->current_file_handler = fopen( this->filename.c_str(), "r");

            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
                return;

            } else {
                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                    this->file_size = 0;
                } else {
                    this->file_size = ftell(this->current_file_handler);
                    fseek(this->current_file_handler, 0, SEEK_SET);
                }
                gcode->stream->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);
                gcode->stream->printf("File selected\r\n");
            }


            this->played_cnt = 0;
            this->played_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;

        } else if (gcode->m == 24) { // start print
            if (this->current_file_handler != NULL) {
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream = THEKERNEL->streams;
            }

        } else if (gcode->m == 25) { // pause print
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            if(this->current_file_handler != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->current_file_handler = fopen(currentfn.c_str() , "r");

                    if(this->current_file_handler == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->filename = currentfn;
                        this->file_size = old_size;
                        this->current_stream = nullptr;
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            progress_command("-b", gcode->stream);

        } else if (gcode->m == 32) { // select file and start print
            // Get filename
            this->filename = "/sd/" + args; // filename is whatever is in args including spaces
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            } else {
                this->playing_file = true;

                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                        file_size = 0;
                } else {
                        file_size = ftell(this->current_file_handler);
                        fseek(this->current_file_handler, 0, SEEK_SET);
                }
            }

            this->played_cnt = 0;
            this->played_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;

        } else if (gcode->m == 600) { // suspend print, Not entirely Marlin compliant, M600.1 will leave the heaters on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);

        } else if (gcode->m == 601) { // resume print
            this->resume_command("", gcode->stream);
        }

    }else if(gcode->has_g) {
        if(gcode->g == 28) { // homing cancels suspend
            if (THEKERNEL->is_suspending()) {
                // clean up
            	THEKERNEL->set_suspending(false);
                THEROBOT->pop_state();
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

	// new_message.stream->printf("Play Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort") {
        this->abort_command( possible_command, new_message.stream );
    }else if (cmd == "suspend") {
        this->suspend_command( possible_command, new_message.stream );
    }else if (cmd == "resume") {
        this->resume_command( possible_command, new_message.stream );
    }else if (cmd == "buffer") {
    	this->buffer_command( possible_command, new_message.stream );
    }else if (cmd == "download") {
        memset(md5_str, 0, sizeof(md5_str));
    	if (possible_command.find("config.txt") != string::npos) {
        	this->test_command( possible_command, new_message.stream );
    	}
    	this->download_command( possible_command, new_message.stream );
    }
}

// Buffer gcode to queue
void Player::buffer_command( string parameters, StreamOutput *stream )
{
	this->buffered_queue.push(parameters);
	stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{
//    // current tool number and tool offset
//    struct tool_status tool;
//    bool tool_ok = PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
//    if (tool_ok) {
//    	tool_ok = tool.active_tool > 0;
//    }
//	// check if is tool -1 or tool 0
//	if (!tool_ok) {
//		THEKERNEL->call_event(ON_HALT, nullptr);
//		THEKERNEL->set_halt_reason(MANUAL);
//		THEKERNEL->streams->printf("ERROR: No tool or probe tool!\n");
//		return;
//	}

    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(parameters);

    if(this->playing_file || THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    if(this->current_file_handler != NULL) { // must have been a paused print
        fclose(this->current_file_handler);
    }

    this->current_file_handler = fopen( this->filename.c_str(), "r");
    if(this->current_file_handler == NULL) {
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", this->filename.c_str());

    this->playing_file = true;

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        // we send to the kernels stream as it cannot go away
        this->current_stream = THEKERNEL->streams;
    }

    // get size of file
    int result = fseek(this->current_file_handler, 0, SEEK_END);
    if (0 != result) {
        stream->printf("WARNING - Could not get file size\r\n");
        file_size = 0;
    } else {
        file_size = ftell(this->current_file_handler);
        fseek(this->current_file_handler, 0, SEEK_SET);
        stream->printf("  File size %ld\r\n", file_size);
    }
    this->played_cnt = 0;
    this->played_lines = 0;
    this->elapsed_secs = 0;
    this->playing_lines = 0;

    // force into absolute mode
    THEROBOT->absolute_mode = true;
    THEROBOT->e_absolute_mode = true;

    // reset current position;
    THEROBOT->reset_position_from_current_actuator_position();
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && current_file_handler != NULL) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", played_cnt, file_size);
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est = 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec = played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est = (file_size - played_cnt) / bytespersec;
        }

        float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), this->elapsed_secs / 3600, (this->elapsed_secs % 3600) / 60, this->elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput *stream )
{
    if(!playing_file && current_file_handler == NULL) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    this->playing_file = false;
    this->played_cnt = 0;
    this->played_lines = 0;
    this->playing_lines = 0;
    this->file_size = 0;
    this->clear_buffered_queue();
    this->filename = "";
    this->current_stream = NULL;

    fclose(current_file_handler);
    current_file_handler = NULL;

    THEKERNEL->set_suspending(false);
    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);

    // turn off spindle
    {
		struct SerialMessage message;
		message.message = "M5";
		message.stream = THEKERNEL->streams;
		message.line = 0;
		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }

    if (parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THEKERNEL->conveyor->flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT->reset_position_from_current_actuator_position();
        stream->printf("Aborted playing or paused file. \r\n");
    }
}

void Player::clear_buffered_queue(){
	while (!this->buffered_queue.empty()) {
		this->buffered_queue.pop();
	}
}

void Player::on_main_loop(void *argument)
{
    if( !this->booted ) {
        this->booted = true;
        if (this->home_on_boot) {
    		struct SerialMessage message;
    		message.message = "$H";
    		message.stream = THEKERNEL->streams;
    		message.line = 0;
    		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        }

        if (this->on_boot_gcode_enable) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        }

    }

    if ( this->playing_file ) {
        if(THEKERNEL->is_halted() || THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || this->inner_playing) {
            return;
        }

        // check if there are bufferd command
        while (!this->buffered_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->buffered_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->buffered_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->buffered_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

        char buf[130]; // lines upto 128 characters are allowed, anything longer is discarded
        bool discard = false;

        while(fgets(buf, sizeof(buf), this->current_file_handler) != NULL) {
            int len = strlen(buf);
            if (len == 0) continue; // empty line? should not be possible
            if (buf[len - 1] == '\n' || feof(this->current_file_handler)) {
                if(discard) { // we are discarding a long line
                    discard = false;
                    continue;
                }
                if(len == 1) continue; // empty line

                if(this->current_stream != nullptr) {
                    this->current_stream->printf("%s", buf);
                }

                struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
                message.line = played_lines + 1;

                // waits for the queue to have enough room
                // this->current_stream->printf("Run: %s", buf);
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                played_lines += 1;
                played_cnt += len;
                return; // we feed one line per main loop

            } else {
                // discard long line
                if(this->current_stream != nullptr) { this->current_stream->printf("Warning: Discarded long line\n"); }
                discard = true;
            }
        }

        this->playing_file = false;
        this->filename = "";
        played_cnt = 0;
        played_lines = 0;
        playing_lines = 0;
        file_size = 0;
        fclose(this->current_file_handler);
        current_file_handler = NULL;
        this->current_stream = NULL;

        if(this->reply_stream != NULL) {
            // if we were printing from an M command from pronterface we need to send this back
            this->reply_stream->printf("Done printing file\r\n");
            this->reply_stream = NULL;
        }
    }
}

void Player::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum) || pdr->second_element_is(is_suspended_checksum)) {
        static bool bool_data;
        bool_data = pdr->second_element_is(is_playing_checksum) ? this->playing_file : THEKERNEL->is_suspending();
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(file_size > 0 && playing_file) {
        	if (!this->inner_playing) {
                const Block *block = StepTicker::getInstance()->get_current_block();
                // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
                // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
                if (block != nullptr && block->is_ready && block->is_g123) {
                	this->playing_lines = block->line;
                	p.played_lines = this->playing_lines;
                } else {
                	p.played_lines = this->played_lines;
                }
        	} else {
        		p.played_lines = this->played_lines;
        	}
            p.elapsed_secs = this->elapsed_secs;
            float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
            p.percent_complete = roundf(pcnt);
            p.filename = this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = this->inner_playing;
        pdr->set_data_ptr(&b);
        pdr->set_taken();
    }
}

void Player::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = *static_cast<bool *>(pdr->get_data_ptr());
    	this->inner_playing = b;
    	if (this->playing_file) pdr->set_taken();
    }
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
1a. loop on_main_loop several times to clear any buffered commmands
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!this->playing_file) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    stream->printf("Suspending , waiting for queue to empty...\n");

    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Suspend aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);
    THEKERNEL->set_suspending(true);

    // save current XYZ position in WCS
    Robot::wcs_t mpos= THEROBOT->get_axis_position();
    Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
    saved_position[0]= std::get<X_AXIS>(wpos);
    saved_position[1]= std::get<Y_AXIS>(wpos);
    saved_position[2]= std::get<Z_AXIS>(wpos);

    // save current state
    THEROBOT->push_state();

    // execute optional gcode if defined
    if(!after_suspend_gcode.empty()) {
        struct SerialMessage message;
        message.message = after_suspend_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    THEKERNEL->streams->printf("Suspended, resume to continue playing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if(!THEKERNEL->is_suspending()) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEROBOT->pop_state();
        THEKERNEL->set_suspending(false);
        return;
    }

    // execute optional gcode if defined
    if(!before_resume_gcode.empty()) {
        stream->printf("Executing before resume gcode...\n");
        struct SerialMessage message;
        message.message = before_resume_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    // Restore position
    stream->printf("Restoring saved XYZ positions and state...\n");
    // force absolute mode for restoring position, then set to the saved relative/absolute mode
    THEROBOT->absolute_mode = true;
    {
        // NOTE position was saved in WCS (for tool change which may change WCS expecially the Z)
        char buf[128];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f Z%.3f F1000", saved_position[0], saved_position[1], saved_position[2]);
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }
    THEROBOT->pop_state();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEKERNEL->set_suspending(false);
        return;
    }

	THEKERNEL->set_suspending(false);

	stream->printf("Playing file resumed\n");
}

unsigned int Player::crc16_ccitt(unsigned char *data, unsigned int len)
{
	static const unsigned short crc_table[] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
	};

	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

int Player::check_crc(int crc, unsigned char *data, unsigned int len)
{
    if (crc) {
        unsigned short crc = crc16_ccitt(data, len);
        unsigned short tcrc = (data[len] << 8) + data[len+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        unsigned char cks = 0;
        for (unsigned int i = 0; i < len; ++i) {
            cks += data[i];
        }
        if (cks == data[len])
        return 1;
    }

    return 0;
}

int Player::inbyte(StreamOutput *stream, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->_getc();
        safe_delay_us(100);
    }
    return -1;
}

int Player::inbytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->gets(buf, size);
        safe_delay_us(100);
    }
    return -1;
}

void Player::flush_input(StreamOutput *stream)
{
    while (inbyte(stream, TIMEOUT_MS) >= 0)
        continue;
}

void Player::cancel_transfer(StreamOutput *stream)
{
	stream->_putc(CAN);
	stream->_putc(CAN);
	stream->_putc(CAN);
	flush_input(stream);
}

void Player::set_serial_rx_irq(bool enable)
{
	// disable serial rx irq
    bool enable_irq = enable;
    PublicData::set_value( atc_handler_checksum, set_serial_rx_irq_checksum, &enable_irq );
}

void Player::test_command( string parameters, StreamOutput* stream ) {
	string test = parameters;
    string filename = absolute_from_relative(parameters);
	FILE *fd = fopen(filename.c_str(), "rb");
	if (NULL != fd) {
        MD5 md5;
        uint8_t md5_buf[64];
        do {
            size_t n = fread(md5_buf, 1, sizeof(md5_buf), fd);
            if (n > 0) md5.update(md5_buf, n);
            THEKERNEL->call_event(ON_IDLE);
        } while (!feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fclose(fd);
        fd = NULL;
	}
}

void Player::download_command( string parameters, StreamOutput *stream )
{
	unsigned char xbuff[135]; /* 1 for data length + 128 for XModem + 3 head chars + 2 crc + nul */
    int bufsz = 128;
    int crc = 0;
    unsigned char packetno = 0;
    int i, c = 0;
    int retry = 0;
    bool resend = true;

    // open file
	char error_msg[64];
	unsigned char md5_sent = 0;
	memset(error_msg, 0, sizeof(error_msg));
    string filename = absolute_from_relative(parameters);
    string md5_filename = change_to_md5_path(filename);

	// diasble irq
    if (stream->type() == 0) {
    	set_serial_rx_irq(false);
    }
    THEKERNEL->set_uploading(true);

    if (!THECONVEYOR->is_idle()) {
        cancel_transfer(stream);
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        THEKERNEL->set_uploading(false);
        return;
    }

    char md5[64];
    memset(md5, 0, sizeof(md5));

    FILE *fd = fopen(md5_filename.c_str(), "rb");
    if (fd != NULL) {
        fread(md5, sizeof(char), 64, fd);
        fclose(fd);
        fd = NULL;
    } else {
    	strcpy(md5, this->md5_str);
    }

    fd = fopen(filename.c_str(), "rb");
    if (NULL == fd) {
	    cancel_transfer(stream);
		sprintf(error_msg, "Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
		goto download_error;
    }

    for(;;) {
		for (retry = 0; retry < MAXRETRANS; ++retry) {
			if ((c = inbyte(stream, TIMEOUT_MS)) >= 0) {
				switch (c) {
				case 'C':
					crc = 1;
					goto start_trans;
				case NAK:
					crc = 0;
					goto start_trans;
				case CAN:
					if ((c = inbyte(stream, TIMEOUT_MS)) == CAN) {
						stream->_putc(ACK);
						flush_input(stream);
				    	sprintf(error_msg, "Info: canceled by remote!\r\n");
				        goto download_error;
					}
					break;
				default:
					break;
				}
			}
		}
        cancel_transfer(stream);
		sprintf(error_msg, "Error: download sync error! get char [%02X], retry [%d]!\r\n", c, retry);
        goto download_error;

		for(;;) {
		start_trans:
			if (packetno == 0 && md5_sent == 0) {
				c = strlen(md5);
				memcpy(&xbuff[4], md5, c);
				md5_sent = 1;
			} else {
				c = fread(&xbuff[4], sizeof(char), bufsz, fd);
				if (c <= 0) {
					for (retry = 0; retry < MAXRETRANS; ++retry) {
						stream->_putc(EOT);
						if ((c = inbyte(stream, TIMEOUT_MS)) == ACK) break;
					}
					flush_input(stream);
					if (c == ACK) {
						goto download_success;
					} else {
						sprintf(error_msg, "Error: get finish ACK error!\r\n");
				        goto download_error;
					}
				}
			}
			xbuff[0] = SOH;
			xbuff[1] = packetno;
			xbuff[2] = ~packetno;
			xbuff[3] = c;
			if (c < bufsz) {
				memset(&xbuff[4 + c], CTRLZ, bufsz - c);
			}

			if (crc) {
				unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz + 1);
				xbuff[bufsz + 4] = (ccrc >> 8) & 0xFF;
				xbuff[bufsz + 5] = ccrc & 0xFF;
			} else {
				unsigned char ccks = 0;
				for (i = 4; i < bufsz+4; ++i) {
					ccks += xbuff[i];
				}
				xbuff[bufsz + 4] = ccks;
			}

			resend = true;
			for (retry = 0; retry < MAXRETRANS; ++retry) {
				if (resend) {
					stream->puts((char *)xbuff, bufsz + 5 + (crc ? 1:0));
					resend = false;
				}
				if ((c = inbyte(stream, TIMEOUT_MS)) >= 0 ) {
					switch (c) {
					case ACK:
						++packetno;
						goto start_trans;
					case CAN:
						if ((c = inbyte(stream, TIMEOUT_MS)) == CAN) {
							stream->_putc(ACK);
							flush_input(stream);
					    	sprintf(error_msg, "Info: canceled by remote!\r\n");
					        goto download_error;
						}
						break;
					case NAK:
						resend = true;
					default:
						break;
					}
				}
			}

	        cancel_transfer(stream);
			sprintf(error_msg, "Error: transmit error, char: [%d], retry: [%d]!\r\n", c, retry);
	        goto download_error;
		}
	}
download_error:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	flush_input(stream);
    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
    THEKERNEL->set_uploading(false);
	stream->printf(error_msg);
	return;
download_success:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	flush_input(stream);
    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
    THEKERNEL->set_uploading(false);
	stream->printf("Info: download success: %s.\r\n", filename.c_str());
	return;
}


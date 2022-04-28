/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <stdarg.h>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Gcode.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole2.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "SwitchPublicAccess.h"
#include "ATCHandlerPublicAccess.h"

#define wp_checksum						CHECKSUM("wp")
#define min_voltage_checksum			CHECKSUM("min_voltage")
#define max_voltage_checksum			CHECKSUM("max_voltage")

// Wireless probe serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole2::SerialConsole2() {
    this->wp_voltage = 0.0;
}

// Called when the module has just been loaded
void SerialConsole2::on_module_loaded() {

    this->serial = new mbed::Serial( USBTX, USBRX );
    this->serial->baud(DEFAULT_SERIAL_BAUD_RATE);

    // We want to be called every time a new char is received
    this->serial->attach(this, &SerialConsole2::on_serial_char_received, mbed::Serial::RxIrq);

    // this->min_voltage = THEKERNEL->config->value(wp_checksum, min_voltage_checksum)->by_default(3.5)->as_number();
    // this->max_voltage = THEKERNEL->config->value(wp_checksum, max_voltage_checksum)->by_default(4.2)->as_number();

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole2::on_serial_char_received() {
    while (this->serial->readable()){
        char received = this->serial->getc();
        // convert CR to NL (for host OSs that don't send NL)
        if ( received == '\r' ) { received = '\n'; }
        this->buffer.push_back(received);
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole2::on_main_loop(void * argument) {
    if ( this->has_char('\n') ) {
        string received;
        received.reserve(20);
        while (1) {
           char c;
           this->buffer.pop_front(c);
           if ( c == '\n' ) {
        	   // get wireless probe voltage
        	   Gcode gc(received, &StreamOutput::NullStream);
        	   this->wp_voltage = gc.get_value('V');
        	   // compare voltage value and switch probe charger
        	   if (this->wp_voltage <= this->min_voltage) {
					bool b = true;
					PublicData::set_value( switch_checksum, probecharger_checksum, state_checksum, &b );
        	   } else  if (this->wp_voltage >= this->min_voltage) {
					bool b = false;
					PublicData::set_value( switch_checksum, probecharger_checksum, state_checksum, &b );
        	   }
               return;
            } else {
                received += c;
            }
        }
    }
}

int SerialConsole2::puts(const char* s)
{
    //return fwrite(s, strlen(s), 1, (FILE*)(*this->serial));
    size_t n= strlen(s);
    for (size_t i = 0; i < n; ++i) {
        _putc(s[i]);
    }
    return n;
}

int SerialConsole2::gets(char** buf)
{
	getc_result = this->_getc();
	*buf = &getc_result;
	return 1;
}

int SerialConsole2::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole2::_getc()
{
    return this->serial->getc();
}

// Does the queue have a given char ?
bool SerialConsole2::has_char(char letter){
    int index = this->buffer.tail;
    while( index != this->buffer.head ){
        if( this->buffer.buffer[index] == letter ){
            return true;
        }
        index = this->buffer.next_block_index(index);
    }
    return false;
}

void SerialConsole2::on_get_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(get_wp_voltage_checksum)) {
        float *t = static_cast<float*>(pdr->get_data_ptr());
        *t = this->wp_voltage;
        pdr->set_taken();
    }

}

void SerialConsole2::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_wp_laser_checksum)) {
    	this->puts("L\n");
        pdr->set_taken();
    }
}


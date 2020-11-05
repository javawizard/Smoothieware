#include "libs/Kernel.h"
#include "MainButton.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "SlowTicker.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "us_ticker_api.h"

using namespace std;

#define main_button_enable_checksum 				CHECKSUM("main_button_enable")
#define main_button_pin_checksum    				CHECKSUM("main_button_pin")
#define main_button_LED_R_pin_checksum    			CHECKSUM("main_button_LED_R_pin")
#define main_button_LED_G_pin_checksum    			CHECKSUM("main_button_LED_G_pin")
#define main_button_LED_B_pin_checksum    			CHECKSUM("main_button_LED_B_pin")
#define main_button_poll_frequency_checksum			CHECKSUM("main_button_poll_frequency")
#define main_long_press_time_ms_checksum			CHECKSUM("main_button_long_press_time")

#define power_checksum								CHECKSUM("power")
#define auto_sleep_checksum							CHECKSUM("auto_sleep")
#define auto_sleep_min_checksum						CHECKSUM("auto_sleep_min")

MainButton::MainButton()
{
	this->led_update_timer = 0;
	this->hold_toggle = 0;
    this->button_state = NONE;
    this->button_pressed = false;
    this->sleep_countdown_us = us_ticker_read();
}

void MainButton::on_module_loaded()
{
    bool main_button_enable = THEKERNEL->config->value( main_button_enable_checksum )->by_default(true)->as_bool(); // @deprecated
    if (!main_button_enable) {
        delete this;
        return;
    }

    this->main_button.from_string( THEKERNEL->config->value( main_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->main_button_LED_R.from_string( THEKERNEL->config->value( main_button_LED_R_pin_checksum )->by_default("nc")->as_string())->as_output();
    this->main_button_LED_G.from_string( THEKERNEL->config->value( main_button_LED_G_pin_checksum )->by_default("nc")->as_string())->as_output();
    this->main_button_LED_B.from_string( THEKERNEL->config->value( main_button_LED_B_pin_checksum )->by_default("nc")->as_string())->as_output();
    this->poll_frequency = THEKERNEL->config->value( main_button_poll_frequency_checksum )->by_default(20)->as_number();
    this->long_press_time_ms = THEKERNEL->config->value( main_long_press_time_ms_checksum )->by_default(3000)->as_number();

    this->auto_sleep = THEKERNEL->config->value(power_checksum, auto_sleep_checksum )->by_default(true)->as_bool();
    this->auto_sleep_min = THEKERNEL->config->value(power_checksum, auto_sleep_min_checksum )->by_default(30)->as_number();

    this->register_for_event(ON_IDLE);

    this->main_button_LED_R.set(0);
    this->main_button_LED_G.set(0);
    this->main_button_LED_B.set(0);

    THEKERNEL->slow_ticker->attach( this->poll_frequency, this, &MainButton::button_tick );
}

void MainButton::on_idle(void *argument)
{
    if (button_state ==  BUTTON_LED_UPDATE || button_state == BUTTON_SHORT_PRESSED || button_state == BUTTON_LONG_PRESSED) {
    	// get current status
    	uint8_t state = THEKERNEL->get_state();
    	if (this->auto_sleep) {
        	if (state == IDLE) {
        		// reset sleep timer counte
        		if (us_ticker_read() - sleep_countdown_us > (uint32_t)auto_sleep_min * 60 * 1000000) {
        			// go to sleep
    				THEKERNEL->set_sleeping(true);
    				THEKERNEL->call_event(ON_HALT, nullptr);
        		}
        	} else {
        		sleep_countdown_us = us_ticker_read();
        	}
    	}
    	uint8_t halt_reason;
    	if (button_state == BUTTON_SHORT_PRESSED) {
    		switch (state) {
    			case IDLE:
    			case RUN:
    			case HOME:
    				// Halt
    		        THEKERNEL->call_event(ON_HALT, nullptr);
    		        THEKERNEL->set_halt_reason(MANUAL);
    				break;
    			case HOLD:
    				// resume
    				THEKERNEL->set_feed_hold(false);
    				break;
    			case ALARM:
    				// do nothing
    				break;
    			case SLEEP:
    				// reset
    				system_reset(false);
    				break;
    		}
    	} else if (button_state == BUTTON_LONG_PRESSED) {
    		switch (state) {
    			case IDLE:
    				// sleep
    				THEKERNEL->set_sleeping(true);
    				THEKERNEL->call_event(ON_HALT, nullptr);
    				break;
    			case RUN:
    			case HOME:
    				// halt
    		        THEKERNEL->call_event(ON_HALT, nullptr);
    		        THEKERNEL->set_halt_reason(MANUAL);
    				break;
    			case HOLD:
    				// resume
    				THEKERNEL->set_feed_hold(false);
    				break;
    			case ALARM:
    				halt_reason = THEKERNEL->get_halt_reason();
    				if (halt_reason > 10) {
    					// reset
        				system_reset(false);
    				} else {
    					// unlock
    		            THEKERNEL->call_event(ON_HALT, (void *)1); // clears on_halt
    		            THEKERNEL->streams->printf("UnKill button pressed, Halt cleared\r\n");
    				}
    				break;
    			case SLEEP:
    				// reset
    				system_reset(false);
    				break;
    		}
    	} else {
    		// update led status
    		switch (state) {
    			case IDLE:
    			    this->main_button_LED_R.set(0);
    			    this->main_button_LED_G.set(0);
    			    this->main_button_LED_B.set(1);
    				break;
    			case RUN:
    			    this->main_button_LED_R.set(0);
    			    this->main_button_LED_G.set(1);
    			    this->main_button_LED_B.set(0);
    				break;
    			case HOME:
    			    this->main_button_LED_R.set(1);
    			    this->main_button_LED_G.set(1);
    			    this->main_button_LED_B.set(0);
    				break;
    			case HOLD:
    				this->hold_toggle ++;
    			    this->main_button_LED_R.set(0);
    			    this->main_button_LED_G.set(this->hold_toggle % 4  < 2 ? 1 : 0);
    			    this->main_button_LED_B.set(0);
    				break;
    			case ALARM:
    			    this->main_button_LED_R.set(1);
    			    this->main_button_LED_G.set(0);
    			    this->main_button_LED_B.set(0);
    			    break;
    			case SLEEP:
    			    this->main_button_LED_R.set(1);
    			    this->main_button_LED_G.set(1);
    			    this->main_button_LED_B.set(1);
    				break;
    		}
    	}
    	button_state = NONE;
    }
}

// Check the state of the button and act accordingly using the following FSM
// Note this is ISR so don't do anything nasty in here
// If in toggle mode (locking estop) then button down will kill, and button up will unkill if unkill is enabled
// otherwise it will look for a 2 second press on the kill button to unkill if unkill is set
uint32_t MainButton::button_tick(uint32_t dummy)
{
	if (this->main_button.get()) {
		if (!this->button_pressed) {
			// button down
			this->button_pressed = true;
			this->button_press_time = us_ticker_read();
		}
	} else {
		// button up
		if (this->button_pressed) {
			if (us_ticker_read() - this->button_press_time > this->long_press_time_ms * 1000) {
				button_state = BUTTON_LONG_PRESSED;
			} else {
				button_state = BUTTON_SHORT_PRESSED;
			}
			this->button_pressed = false;
		} else {
            if(++led_update_timer > this->poll_frequency * 0.2) {
            	button_state = BUTTON_LED_UPDATE;
            	led_update_timer = 0;
            }
		}
	}
    return 0;
}


#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

#include "Module.h"
#include "Pin.h"

#include <stdio.h>
#include <string>
#include <map>
#include <vector>
using std::string;

class StreamOutput;


class ATCHandler : public Module
{
public:
	ATCHandler();

    void on_module_loaded();
    void on_main_loop( void* argument );
    void on_gcode_received(void *argument);
    void on_halt(void *argument);
    void on_config_reload(void *argument);

private:
    void init_script();
    string fill_vars(string source);
    string extract_options(string& args);
    void stop_atc( string parameters, StreamOutput* stream );

    uint32_t read_endstop(uint32_t dummy);

    void clamp_tool();
    void loose_tool();
    void home_clamp();

    std::vector<string> zprobe_script;
    std::vector<string> clamp_tool_script;
    std::vector<string> loose_tool_script;
    std::vector<string> atc_script;

    uint16_t debounce;

    using atc_homing_info_t = struct {
        Pin pin;
        uint16_t debounce_ms;
        float max_travel;
        float retract;
        float homing_rate;
        float action_rate;
        float action_dist;

        struct {
            bool triggered:1;
            bool homed:1;
        };
    };
    atc_homing_info_t atc_home_info;


    unsigned int played_cnt;

    struct {
        bool on_boot_gcode_enable:1;
        bool booted:1;
        bool playing_atc:1;
        bool suspended:1;
        bool was_playing_file:1;
        bool atc_homing:1;
        uint8_t suspend_loops:4;
    };
};

#endif /* _ATCHANDLER_H */

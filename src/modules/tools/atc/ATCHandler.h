#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

using namespace std;
#include "Module.h"
#include <vector>
#include <queue>
#include "Pin.h"

#include "ATCHandlerPublicAccess.h"

class ATCHandler : public Module
{
public:
    ATCHandler();

    void on_module_loaded();
    void on_gcode_received(void *argument);
    void on_get_public_data(void *argument);
    void on_set_public_data(void *argument);
    void on_main_loop( void* argument );
    void on_halt(void *argument);
    int get_active_tool() const { return active_tool; }
    void on_config_reload(void *argument);


private:
    typedef enum {
        NONE,
        FULL, // M6T?
        DROP, // M6T-1
        PICK, // M6T?
		CALI, // M???
    } ATC_STATUS;

    ATC_STATUS atc_status;

    uint32_t read_endstop(uint32_t dummy);
    uint32_t read_detector(uint32_t dummy);

    // clamp actions
    void clamp_tool();
    void loose_tool();
    void home_clamp();

    // laser detect
    bool laser_detect();

    void set_inner_playing(bool inner_playing);
    bool get_inner_playing() const;

    void fill_drop_scripts();
    void fill_pick_scripts();
    void fill_cali_scripts();

    void clear_script_queue();

    std::queue<string> script_queue;

    uint16_t debounce;
    bool atc_homing;
    bool detecting;

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

    using detector_info_t = struct {
        Pin detect_pin;
        float detect_rate;
        float detect_travel;
        bool triggered;
    };
    detector_info_t detector_info;

    float safe_z_mm;
    float safe_z_offset_mm;
    float fast_z_rate;
    float slow_z_rate;
    float probe_mx_mm;
    float probe_my_mm;
    float probe_mz_mm;

    vector<struct atc_tool> atc_tools;

    int new_tool;
    int active_tool;
    int tool_number;
};

#endif /* _ATCHANDLER_H */

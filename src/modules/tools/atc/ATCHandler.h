#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

using namespace std;
#include "Module.h"
#include <vector>
#include <map>
#include <queue>
#include "Pin.h"

class StreamOutput;

class ATCHandler : public Module
{
public:
    ATCHandler();

    void on_module_loaded();
    void on_gcode_received(void *argument);
    void on_console_line_received( void *argument );
    void on_set_public_data(void* argument);
    void on_main_loop( void* argument );
    void on_halt(void *argument);
    void on_config_reload(void *argument);


private:
    typedef enum {
        NONE,
        TEST,
        STEP
    } EXTRACTOR_STATUS;

    EXTRACTOR_STATUS extractor_status;

    void set_inner_playing(bool inner_playing);
    bool get_inner_playing() const;

    void load_tests();

    // send data to HMI screen
    void hmi_load_tests(string parameters, StreamOutput *stream); //
    void hmi_load_test_info(string parameters, StreamOutput *stream); //
    void hmi_do_nothing(); //
    // update data to HMI screen
    void hmi_update(StreamOutput *stream);
    void hmi_send_finish();
	// execute command from HMI screen
	void hmi_test(string parameters, StreamOutput *stream);
	void hmi_step(string parameters, StreamOutput *stream);
	void hmi_lift_z(string parameters, StreamOutput *stream);

    void fill_test_scripts();
    void fill_step_scripts(int index, int minutes, int pressure, bool waste);

    void clear_script_queue();

    void rapid_move(float x, float y, float z);

    std::queue<string> script_queue;

    float safe_z;
    float z_rate_work;

    float z_pos_work;
    float x_pos[6];
    float y_pos_waste;
    float y_pos_gather;
    float y_pos_clear;

    float pump_start_power;
    float pump_start_seconds;
    float pump_acc_seconds;

    string curr_test_name;
    int curr_total_minutes;
    uint32_t curr_total_start_ticket;

    int curr_reagent_index;
    int curr_reagent_minutes;
    uint32_t curr_reagent_start_ticket;

    struct reagent {
    	string name;
    	int minutes;	// 1 - 100
    	int pressure;	// 10 - 100
    };

    map<string, vector<struct reagent>> tests;

};

#endif /* _ATCHANDLER_H */

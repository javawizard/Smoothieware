/*
 * WifiProvider.h
 *
 *  Created on: 2020年6月10日
 *      Author: josh
 */

#ifndef WIFIPROVIDER_H_
#define WIFIPROVIDER_H_

using namespace std;
#include <vector>
#include <queue>

#include "Pin.h"
#include "Module.h"
#include "StreamOutput.h"

#include "M8266WIFIDrv.h"
#include "libs/RingBuffer.h"

#define WIFI_IO_DATA_MAX_SIZE	128

class WifiProvider : public Module, public StreamOutput
{
public:
	WifiProvider();

    void on_module_loaded();
    void on_gcode_received(void *argument);
    void on_main_loop( void* argument );
    void on_second_tick(void* argument);
    void on_idle(void* argument);

    int puts(const char*);
    int _putc(int c);
    int _getc(void);
    bool ready();
    bool has_char(char letter);

private:
    void M8266WIFI_Module_delay_ms(u16 nms);
    void M8266WIFI_Module_Hardware_Reset(void);
    u8 M8266WIFI_Module_Init_Via_SPI();

    void init_wifi_module(bool reset);
    void query_wifi_status();
    void set_wifi_op_mode();

    void on_pin_rise();
    mbed::InterruptIn *wifi_interrupt_pin; // Interrupt pin for measuring speed
    float probe_slow_rate;

    void receive_wifi_data();

    RingBuffer<char, 256> buffer; // Receive buffer

	u8 RecvData[WIFI_IO_DATA_MAX_SIZE];
	u8 SendData[WIFI_IO_DATA_MAX_SIZE];

    struct {
    	u8  tcp_link_no;
    	u8  udp_link_no;
    	bool connected:1;
    	volatile bool halt_flag:1;
    	volatile bool query_flag:1;
    };


};

#endif /* WIFIPROVIDER_H_ */

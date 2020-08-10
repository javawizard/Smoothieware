#ifndef ATCHANDLERPUBLICACCESS_H
#define ATCHANDLERPUBLICACCESS_H

#include "checksumm.h"
#include <string>

#define atc_handler_checksum   		CHECKSUM("atc_handler")
#define get_tool_status_checksum    CHECKSUM("get_tool_status")
#define set_ref_tool_mz_checksum	CHECKSUM("set_ref_tool_mz")
#define get_atc_pin_status_checksum	CHECKSUM("set_ref_tool_mz")

struct tool_status {
	int active_tool;
	float cur_tool_mz;
	float ref_tool_mz;
	float tool_offset;
};

#endif

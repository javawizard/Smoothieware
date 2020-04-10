#ifndef ATCHANDLERPUBLICACCESS_H
#define ATCHANDLERPUBLICACCESS_H

#include "checksumm.h"
#include <string>

#define atc_handler_checksum   		CHECKSUM("atc_handler")
#define get_active_tool_checksum    CHECKSUM("get_active_tool")

struct atc_tool {
	int num;
	float mx_mm;
	float my_mm;
	float mz_mm;
	float tool_offset;
};

#endif

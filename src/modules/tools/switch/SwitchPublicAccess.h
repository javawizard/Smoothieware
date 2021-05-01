#ifndef __SWITCHPUBLICACCESS_H
#define __SWITCHPUBLICACCESS_H

// addresses used for public data access
#define switch_checksum              CHECKSUM("switch")
#define fan_checksum                 CHECKSUM("fan")
#define vacuum_checksum                 CHECKSUM("vacuum")
#define state_checksum               CHECKSUM("state")
#define light_checksum               CHECKSUM("light")
#define probelaser_checksum			CHECKSUM("probelaser")
#define ps12_checksum               CHECKSUM("ps12")
#define ps24_checksum               CHECKSUM("ps24")

struct pad_switch {
    int name;
    bool state;
    float value;
};

#endif // __SWITCHPUBLICACCESS_H

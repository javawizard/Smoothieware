#ifndef __SWITCHPUBLICACCESS_H
#define __SWITCHPUBLICACCESS_H

// addresses used for public data access
#define switch_checksum              CHECKSUM("switch")
#define fan_checksum                 CHECKSUM("fan")
#define vacuum_checksum                 CHECKSUM("vacuum")
#define state_checksum               CHECKSUM("state")

struct pad_switch {
    int name;
    bool state;
    float value;
};

#endif // __SWITCHPUBLICACCESS_H

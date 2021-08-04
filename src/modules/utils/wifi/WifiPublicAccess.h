#ifndef WIFIPUBLICACCESS_H
#define WIFIPUBLICACCESS_H

#define wlan_checksum       CHECKSUM("wlan")
#define get_wlan_checksum   CHECKSUM("get_wlan")
#define set_wlan_checksum   CHECKSUM("set_wlan")
#define set_ap_channel_checksum   CHECKSUM("set_ap_channel")
#define get_ap_channel_checksum   CHECKSUM("get_ap_channel")

struct ap_conn_info {
    char ssid[32];
    char password[64];
    char ip_address[15];
    bool has_error;
    char error_info[64];
    bool disconnect;
};


#endif

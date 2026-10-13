#include "app_control.h"
uint8_t app_control_dbus_valid(const drv_dbus_data_t *data) {
    if(data->rc.s1<1 || data->rc.s1>3 || data->rc.s2<1 || data->rc.s2>3)return 0;
    for(unsigned i=0;i<4;i++)if(data->rc.ch[i]<364 || data->rc.ch[i]>1684)return 0;
    return 1;
}

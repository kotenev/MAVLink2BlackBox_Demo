
#include "DemoDevice.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_TYPE  type = p0_type_GET(pack);
    e_MAV_AUTOPILOT  autopilot = p0_autopilot_GET(pack);
    uint8_t  base_mode = p0_base_mode_GET(pack);
    uint32_t  custom_mode = p0_custom_mode_GET(pack);
    e_MAV_STATE  system_status = p0_system_status_GET(pack);
    uint8_t  mavlink_version = p0_mavlink_version_GET(pack);
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_COAXIAL, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARDUPILOTMEGA, PH.base.pack) ;
    p0_base_mode_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)521969866L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}

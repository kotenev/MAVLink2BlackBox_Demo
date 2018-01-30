
#pragma once

#include "BlackBox/Host.h"

Pack * c_LoopBackDemoChannel_new_HEARTBEAT_0();
Pack * c_LoopBackDemoChannel_ADV_new_HEARTBEAT_0();
extern void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * bi, Pack * pack);

extern Channel c_LoopBackDemoChannel;
INLINER void c_LoopBackDemoChannel_send(Pack * pack) {c_LoopBackDemoChannel.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes(&c_LoopBackDemoChannel, dst, bytes);}

//------------------------
INLINER void  c_LoopBackDemoChannel_output_bytes(uint8_t* src, int32_t bytes) {  output_bytes(&c_LoopBackDemoChannel, src, bytes);} //pass received from a network bytes to the channel
INLINER void c_LoopBackDemoChannel_process_received() { c_LoopBackDemoChannel.process(NULL, PROCESS_RECEIVED_PACKS);}//dispatch received packs to its handlers

extern Channel c_LoopBackDemoChannel_ADV;
INLINER void c_LoopBackDemoChannel_ADV_send(Pack * pack) {c_LoopBackDemoChannel_ADV.process(pack, PROCESS_HOST_REQEST);}
INLINER int32_t  c_LoopBackDemoChannel_ADV_input_bytes(uint8_t* dst, int32_t bytes) { return input_bytes_adv(&c_LoopBackDemoChannel_ADV, dst, bytes);}

; /*
				                                              */
typedef  enum
{
    e_MAV_TYPE_MAV_TYPE_GENERIC = 0,
    e_MAV_TYPE_MAV_TYPE_FIXED_WING = 1,
    e_MAV_TYPE_MAV_TYPE_QUADROTOR = 2,
    e_MAV_TYPE_MAV_TYPE_COAXIAL = 3,
    e_MAV_TYPE_MAV_TYPE_HELICOPTER = 4,
    e_MAV_TYPE_MAV_TYPE_ANTENNA_TRACKER = 5,
    e_MAV_TYPE_MAV_TYPE_GCS = 6,
    e_MAV_TYPE_MAV_TYPE_AIRSHIP = 7,
    e_MAV_TYPE_MAV_TYPE_FREE_BALLOON = 8,
    e_MAV_TYPE_MAV_TYPE_ROCKET = 9,
    e_MAV_TYPE_MAV_TYPE_GROUND_ROVER = 10,
    e_MAV_TYPE_MAV_TYPE_SURFACE_BOAT = 11,
    e_MAV_TYPE_MAV_TYPE_SUBMARINE = 12,
    e_MAV_TYPE_MAV_TYPE_HEXAROTOR = 13,
    e_MAV_TYPE_MAV_TYPE_OCTOROTOR = 14,
    e_MAV_TYPE_MAV_TYPE_TRICOPTER = 15,
    e_MAV_TYPE_MAV_TYPE_FLAPPING_WING = 16
} e_MAV_TYPE;
; /*
				                                              */
typedef  enum
{
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC = 0,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PIXHAWK = 1,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_SLUGS = 2,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT = 4,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_INVALID = 8,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ = 9,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB = 10,
    e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP = 11
} e_MAV_AUTOPILOT;
; /*
				                                              */
typedef  enum
{
    e_MAV_STATE_MAV_STATE_UNINIT = 0,
    e_MAV_STATE_MAV_STATE_BOOT = 1,
    e_MAV_STATE_MAV_STATE_CALIBRATING = 2,
    e_MAV_STATE_MAV_STATE_STANDBY = 3,
    e_MAV_STATE_MAV_STATE_ACTIVE = 4,
    e_MAV_STATE_MAV_STATE_CRITICAL = 5,
    e_MAV_STATE_MAV_STATE_EMERGENCY = 6,
    e_MAV_STATE_MAV_STATE_POWEROFF = 7
} e_MAV_STATE;

INLINER uint32_t p0_custom_mode_GET(Pack * src)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER void p0_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER uint8_t p0_base_mode_GET(Pack * src)//System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER void p0_base_mode_SET(uint8_t  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER uint8_t p0_mavlink_version_GET(Pack * src)//MAVLink versio
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER void p0_mavlink_version_SET(uint8_t  src, Pack * dst)//MAVLink versio
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER e_MAV_TYPE p0_type_GET(Pack * src)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 48, 5);
}
INLINER void p0_type_SET(e_MAV_TYPE  src, Pack * dst)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 48);
}
INLINER e_MAV_AUTOPILOT p0_autopilot_GET(Pack * src)//Autopilot type / class. defined in MAV_AUTOPILOT ENU
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 53, 4);
}
INLINER void p0_autopilot_SET(e_MAV_AUTOPILOT  src, Pack * dst)//Autopilot type / class. defined in MAV_AUTOPILOT ENU
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 53);
}
INLINER e_MAV_STATE p0_system_status_GET(Pack * src)//System status flag, see MAV_STATE ENU
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 57, 4);
}
INLINER void p0_system_status_SET(e_MAV_STATE  src, Pack * dst)//System status flag, see MAV_STATE ENU
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 57);
}


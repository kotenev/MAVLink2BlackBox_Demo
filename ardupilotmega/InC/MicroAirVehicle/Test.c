
#pragma once
#include "MicroAirVehicle.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

INLINER int32_t Arrays_equals(uint8_t * L, uint8_t * R, int32_t bytes)
{
    for(int32_t i = 0; i < bytes; i++) if(L[i] != R[i]) return i;
    return -1;
}
static Pack * zero2empty(Pack * pack)
{
    pack = realloc(pack, sizeof(Pack) + pack->meta->packMinBytes);
    for(int32_t i = pack->meta->packMinBytes; -1 < --i;)pack->data[i] = 0; //clean bytes
    return pack;
}
void failure(Channel * ch, int32_t id, int32_t arg) {assert(false);}

INLINER uint32_t p0_custom_mode_GET(Pack * src)//A bitfield for use for autopilot-specific flags.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p0_mavlink_version_GET(Pack * src)//MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_versio
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER e_MAV_TYPE p0_type_GET(Pack * src)//Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 40, 5);
}
INLINER e_MAV_AUTOPILOT p0_autopilot_GET(Pack * src)//Autopilot type / class. defined in MAV_AUTOPILOT ENUM
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 45, 5);
}
INLINER e_MAV_MODE_FLAG p0_base_mode_GET(Pack * src)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 50, 8);
}
INLINER e_MAV_STATE p0_system_status_GET(Pack * src)//System status flag, see MAV_STATE ENUM
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 58, 4);
}
INLINER uint16_t p1_load_GET(Pack * src)//Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p1_voltage_battery_GET(Pack * src)//Battery voltage, in millivolts (1 = 1 millivolt)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
/**
*Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links
*	(packets that were corrupted on reception on the MAV*/
INLINER uint16_t p1_drop_rate_comm_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
/**
*Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
*	on reception on the MAV*/
INLINER uint16_t p1_errors_comm_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p1_errors_count1_GET(Pack * src)//Autopilot-specific errors
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p1_errors_count2_GET(Pack * src)//Autopilot-specific errors
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p1_errors_count3_GET(Pack * src)//Autopilot-specific errors
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p1_errors_count4_GET(Pack * src)//Autopilot-specific errors
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER int16_t p1_current_battery_GET(Pack * src)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER int8_t p1_battery_remaining_GET(Pack * src)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
{
    uint8_t * data = src->data;
    return ((int8_t)(get_bytes(data,  18, 1)));
}
/**
*Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1:
*	present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_present_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 152, 26);
}
/**
*Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
*	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_enabled_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 178, 26);
}
/**
*Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
*	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_health_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 204, 26);
}
INLINER uint32_t p2_time_boot_ms_GET(Pack * src)//Timestamp of the component clock since boot time in milliseconds.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint64_t p2_time_unix_usec_GET(Pack * src)//Timestamp of the master clock in microseconds since UNIX epoch.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p3_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p3_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER float p3_x_GET(Pack * src)//X Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER float p3_y_GET(Pack * src)//Y Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p3_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p3_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p3_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float p3_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER float p3_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER float p3_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER float p3_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER float p3_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER float p3_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER e_MAV_FRAME p3_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
INLINER uint32_t p4_seq_GET(Pack * src)//PING sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint64_t p4_time_usec_GET(Pack * src)//Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 8)));
}
/**
*0: request ping from all receiving systems, if greater than 0: message is a ping response and number is
*	the system id of the requesting syste*/
INLINER uint8_t p4_target_system_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
/**
*0: request ping from all receiving components, if greater than 0: message is a ping response and number
*	is the system id of the requesting syste*/
INLINER uint8_t p4_target_component_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  13, 1)));
}
INLINER uint8_t p5_target_system_GET(Pack * src)//System the GCS requests control for
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p5_control_request_GET(Pack * src)//0: request control of this MAV, 1: Release control of this MAV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
/**
*0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use
*	the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
*	message indicating an encryption mismatch*/
INLINER uint8_t p5_version_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	characters may involve A-Z, a-z, 0-9, and "!?,.-*/
INLINER char16_t * p5_passkey_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p5_passkey_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  24 && !try_visit_field(src, 24)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p5_passkey_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  24 && !try_visit_field(src, 24)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p5_passkey_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint8_t p6_gcs_system_id_GET(Pack * src)//ID of the GCS this message
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p6_control_request_GET(Pack * src)//0: request control of this MAV, 1: Release control of this MAV
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
/**
*0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under
*	contro*/
INLINER uint8_t p6_ack_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER char16_t * p7_key_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos) //key
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p7_key_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  0 && !try_visit_field(src, 0)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p7_key_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  0 && !try_visit_field(src, 0)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p7_key_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint32_t p11_custom_mode_GET(Pack * src)//The new autopilot-specific mode. This field can be ignored by an autopilot.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p11_target_system_GET(Pack * src)//The system setting the mode
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER e_MAV_MODE p11_base_mode_GET(Pack * src)//The new base mode
{
    uint8_t * data = src->data;
    switch(get_bits(data, 40, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p20_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p20_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER int16_t p20_param_index_GET(Pack * src)//Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER char16_t * p20_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p20_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p20_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  32 && !try_visit_field(src, 32)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p20_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint8_t p21_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p21_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint16_t p22_param_count_GET(Pack * src)//Total number of onboard parameters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p22_param_index_GET(Pack * src)//Index of this onboard parameter
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER float p22_param_value_GET(Pack * src)//Onboard parameter value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER e_MAV_PARAM_TYPE p22_param_type_GET(Pack * src)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 64, 4);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER char16_t * p22_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p22_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  68 && !try_visit_field(src, 68)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p22_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  68 && !try_visit_field(src, 68)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p22_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint8_t p23_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p23_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER float p23_param_value_GET(Pack * src)//Onboard parameter value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER e_MAV_PARAM_TYPE p23_param_type_GET(Pack * src)//Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 48, 4);
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER char16_t * p23_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p23_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  52 && !try_visit_field(src, 52)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p23_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  52 && !try_visit_field(src, 52)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p23_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint16_t p24_eph_GET(Pack * src)//GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p24_epv_GET(Pack * src)//GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p24_vel_GET(Pack * src)//GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
/**
*Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If
*	unknown, set to: UINT16_MA*/
INLINER uint16_t p24_cog_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint64_t p24_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER int32_t p24_lat_GET(Pack * src)//Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER int32_t p24_lon_GET(Pack * src)//Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
/**
*Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide
*	the AMSL altitude in addition to the WGS84 altitude*/
INLINER int32_t p24_alt_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER uint8_t p24_satellites_visible_GET(Pack * src)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 1)));
}
INLINER e_GPS_FIX_TYPE p24_fix_type_GET(Pack * src)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 232, 4);
}
INLINER int32_t  p24_alt_ellipsoid_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  236 && !try_visit_field(src, 236)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((int32_t)(get_bytes(data,  src->BYTE, 4)));
}
INLINER uint32_t  p24_h_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  237 && !try_visit_field(src, 237)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER uint32_t  p24_v_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  238 && !try_visit_field(src, 238)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER uint32_t  p24_vel_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  239 && !try_visit_field(src, 239)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER uint32_t  p24_hdg_acc_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  240 && !try_visit_field(src, 240)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 4)));
}
INLINER uint8_t p25_satellites_visible_GET(Pack * src)//Number of satellites visible
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t* p25_satellite_prn_GET(Pack * src, uint8_t*  dst, int32_t pos) //Global satellite ID
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 1, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_prn_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_prn_GET_(Pack * src) {return p25_satellite_prn_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p25_satellite_used_GET(Pack * src, uint8_t*  dst, int32_t pos) //0: Satellite not used, 1: used for localization
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 21, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_used_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_used_GET_(Pack * src) {return p25_satellite_used_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p25_satellite_elevation_GET(Pack * src, uint8_t*  dst, int32_t pos) //Elevation (0: right on top of receiver, 90: on the horizon) of satellite
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 41, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_elevation_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_elevation_GET_(Pack * src) {return p25_satellite_elevation_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p25_satellite_azimuth_GET(Pack * src, uint8_t*  dst, int32_t pos) //Direction of satellite, 0: 0 deg, 255: 360 deg.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 61, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_azimuth_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_azimuth_GET_(Pack * src) {return p25_satellite_azimuth_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint8_t* p25_satellite_snr_GET(Pack * src, uint8_t*  dst, int32_t pos) //Signal to noise ratio of satellite
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 81, dst_max = pos + 20; pos < dst_max ; pos++,  BYTE += 1)
        dst[pos] = ((get_bytes(data,  BYTE, 1)));
    return dst;
}

static const  uint32_t p25_satellite_snr_LEN = 20; //return array length

INLINER  uint8_t*  p25_satellite_snr_GET_(Pack * src) {return p25_satellite_snr_GET(src, malloc(20 * sizeof(uint8_t)), 0);}
INLINER uint32_t p26_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER int16_t p26_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER int16_t p26_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  6, 2)));
}
INLINER int16_t p26_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p26_xgyro_GET(Pack * src)//Angular speed around X axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p26_ygyro_GET(Pack * src)//Angular speed around Y axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER int16_t p26_zgyro_GET(Pack * src)//Angular speed around Z axis (millirad /sec)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER int16_t p26_xmag_GET(Pack * src)//X Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER int16_t p26_ymag_GET(Pack * src)//Y Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER int16_t p26_zmag_GET(Pack * src)//Z Magnetic field (milli tesla)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER uint64_t p27_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER int16_t p27_xacc_GET(Pack * src)//X acceleration (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p27_yacc_GET(Pack * src)//Y acceleration (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p27_zacc_GET(Pack * src)//Z acceleration (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER int16_t p27_xgyro_GET(Pack * src)//Angular speed around X axis (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER int16_t p27_ygyro_GET(Pack * src)//Angular speed around Y axis (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  16, 2)));
}
INLINER int16_t p27_zgyro_GET(Pack * src)//Angular speed around Z axis (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  18, 2)));
}
INLINER int16_t p27_xmag_GET(Pack * src)//X Magnetic field (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  20, 2)));
}
INLINER int16_t p27_ymag_GET(Pack * src)//Y Magnetic field (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER int16_t p27_zmag_GET(Pack * src)//Z Magnetic field (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER uint64_t p28_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER int16_t p28_press_abs_GET(Pack * src)//Absolute pressure (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  8, 2)));
}
INLINER int16_t p28_press_diff1_GET(Pack * src)//Differential pressure 1 (raw, 0 if nonexistant)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p28_press_diff2_GET(Pack * src)//Differential pressure 2 (raw, 0 if nonexistant)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER int16_t p28_temperature_GET(Pack * src)//Raw Temperature measurement (raw)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  14, 2)));
}
INLINER uint32_t p29_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p29_press_abs_GET(Pack * src)//Absolute pressure (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p29_press_diff_GET(Pack * src)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER int16_t p29_temperature_GET(Pack * src)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER uint32_t p30_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p30_roll_GET(Pack * src)//Roll angle (rad, -pi..+pi)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p30_pitch_GET(Pack * src)//Pitch angle (rad, -pi..+pi)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p30_yaw_GET(Pack * src)//Yaw angle (rad, -pi..+pi)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p30_rollspeed_GET(Pack * src)//Roll angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p30_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p30_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER uint32_t p31_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p31_q1_GET(Pack * src)//Quaternion component 1, w (1 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p31_q2_GET(Pack * src)//Quaternion component 2, x (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p31_q3_GET(Pack * src)//Quaternion component 3, y (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p31_q4_GET(Pack * src)//Quaternion component 4, z (0 in null-rotation)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p31_rollspeed_GET(Pack * src)//Roll angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p31_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p31_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER uint32_t p32_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p32_x_GET(Pack * src)//X Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p32_y_GET(Pack * src)//Y Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p32_z_GET(Pack * src)//Z Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p32_vx_GET(Pack * src)//X Speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p32_vy_GET(Pack * src)//Y Speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p32_vz_GET(Pack * src)//Z Speed
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER uint16_t p33_hdg_GET(Pack * src)//Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p33_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER int32_t p33_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER int32_t p33_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
/**
*Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules
*	provide the AMSL as well*/
INLINER int32_t p33_alt_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  14, 4)));
}
INLINER int32_t p33_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  18, 4)));
}
INLINER int16_t p33_vx_GET(Pack * src)//Ground X Speed (Latitude, positive north), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  22, 2)));
}
INLINER int16_t p33_vy_GET(Pack * src)//Ground Y Speed (Longitude, positive east), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  24, 2)));
}
INLINER int16_t p33_vz_GET(Pack * src)//Ground Z Speed (Altitude, positive down), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  26, 2)));
}
INLINER uint32_t p34_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*	8 servos*/
INLINER uint8_t p34_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER int16_t p34_chan1_scaled_GET(Pack * src)//RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
INLINER int16_t p34_chan2_scaled_GET(Pack * src)//RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
INLINER int16_t p34_chan3_scaled_GET(Pack * src)//RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER int16_t p34_chan4_scaled_GET(Pack * src)//RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER int16_t p34_chan5_scaled_GET(Pack * src)//RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  13, 2)));
}
INLINER int16_t p34_chan6_scaled_GET(Pack * src)//RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  15, 2)));
}
INLINER int16_t p34_chan7_scaled_GET(Pack * src)//RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  17, 2)));
}
INLINER int16_t p34_chan8_scaled_GET(Pack * src)//RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  19, 2)));
}
INLINER uint8_t p34_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER uint16_t p35_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p35_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p35_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p35_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p35_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p35_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p35_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p35_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint32_t p35_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than
*	8 servos*/
INLINER uint8_t p35_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER uint8_t p35_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER uint16_t p36_servo1_raw_GET(Pack * src)//Servo output 1 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p36_servo2_raw_GET(Pack * src)//Servo output 2 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p36_servo3_raw_GET(Pack * src)//Servo output 3 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p36_servo4_raw_GET(Pack * src)//Servo output 4 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p36_servo5_raw_GET(Pack * src)//Servo output 5 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p36_servo6_raw_GET(Pack * src)//Servo output 6 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p36_servo7_raw_GET(Pack * src)//Servo output 7 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p36_servo8_raw_GET(Pack * src)//Servo output 8 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint32_t p36_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 4)));
}
/**
*Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode
*	more than 8 servos*/
INLINER uint8_t p36_port_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER uint16_t  p36_servo9_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  168 && !try_visit_field(src, 168)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo10_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  169 && !try_visit_field(src, 169)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo11_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  170 && !try_visit_field(src, 170)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo12_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  171 && !try_visit_field(src, 171)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo13_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  172 && !try_visit_field(src, 172)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo14_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  173 && !try_visit_field(src, 173)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo15_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  174 && !try_visit_field(src, 174)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint16_t  p36_servo16_raw_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  175 && !try_visit_field(src, 175)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 2)));
}
INLINER uint8_t p37_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p37_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER int16_t p37_start_index_GET(Pack * src)//Start index, 0 by default
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER int16_t p37_end_index_GET(Pack * src)//End index, -1 by default (-1: send list to end). Else a valid index of the list
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER e_MAV_MISSION_TYPE p37_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 48, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p38_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p38_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER int16_t p38_start_index_GET(Pack * src)//Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
INLINER int16_t p38_end_index_GET(Pack * src)//End index, equal or greater than start index.
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  4, 2)));
}
INLINER e_MAV_MISSION_TYPE p38_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 48, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p39_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p39_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p39_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p39_current_GET(Pack * src)//false:0, true:1
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p39_autocontinue_GET(Pack * src)//autocontinue to next wp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER float p39_param1_GET(Pack * src)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER float p39_param2_GET(Pack * src)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p39_param3_GET(Pack * src)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p39_param4_GET(Pack * src)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p39_x_GET(Pack * src)//PARAM5 / local: x position, global: latitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float p39_y_GET(Pack * src)//PARAM6 / y position: global: longitude
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER float p39_z_GET(Pack * src)//PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER e_MAV_FRAME p39_frame_GET(Pack * src)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER e_MAV_CMD p39_command_GET(Pack * src)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = src->data;
    switch(get_bits(data, 276, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_MISSION_TYPE p39_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 284, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p40_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p40_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p40_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER e_MAV_MISSION_TYPE p40_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p41_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p41_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p41_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint16_t p42_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p43_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p43_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER e_MAV_MISSION_TYPE p43_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 16, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p44_count_GET(Pack * src)//Number of mission items in the sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p44_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p44_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER e_MAV_MISSION_TYPE p44_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p45_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p45_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER e_MAV_MISSION_TYPE p45_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 16, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p46_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p47_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p47_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER e_MAV_MISSION_RESULT p47_type_GET(Pack * src)//See MAV_MISSION_RESULT enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 16, 4);
}
INLINER e_MAV_MISSION_TYPE p47_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 20, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p48_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER int32_t p48_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  1, 4)));
}
INLINER int32_t p48_longitude_GET(Pack * src)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  5, 4)));
}
INLINER int32_t p48_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  9, 4)));
}
INLINER uint64_t  p48_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  104 && !try_visit_field(src, 104)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER int32_t p49_latitude_GET(Pack * src)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  0, 4)));
}
INLINER int32_t p49_longitude_GET(Pack * src)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  4, 4)));
}
INLINER int32_t p49_altitude_GET(Pack * src)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER uint64_t  p49_time_usec_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  96 && !try_visit_field(src, 96)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 8)));
}
INLINER uint8_t p50_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p50_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
/**
*Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored),
*	send -2 to disable any existing map for this rc_channel_index*/
INLINER int16_t p50_param_index_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
/**
*Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
*	on the RC*/
INLINER uint8_t p50_parameter_rc_channel_index_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER float p50_param_value0_GET(Pack * src)//Initial parameter value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  5, 4)));
}
INLINER float p50_scale_GET(Pack * src)//Scale, maps the RC range [-1, 1] to a parameter value
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  9, 4)));
}
/**
*Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends
*	on implementation*/
INLINER float p50_param_value_min_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
/**
*Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
*	on implementation*/
INLINER float p50_param_value_max_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	storage if the ID is stored as strin*/
INLINER char16_t * p50_param_id_GET(Bounds_Inside * src, char16_t *  dst, int32_t pos)
{
    uint8_t * data = src->base.pack->data;
    for(int32_t BYTE = src->BYTE, dst_max = pos + src->items; pos < dst_max ; pos++,  BYTE += 2)
        dst[pos] = ((get_bytes(data,  BYTE, 2)));
    return dst;
}
INLINER int32_t p50_param_id_LEN(Bounds_Inside * src)
{
    return (src->base.field_bit !=  168 && !try_visit_field(src, 168)  ||  !try_visit_item(src, 0)) ? 0 : src->items;
}
INLINER  char16_t* p50_param_id_TRY_(Bounds_Inside * src)
{
    if(src->base.field_bit !=  168 && !try_visit_field(src, 168)  ||  !try_visit_item(src, 0)) return NULL;
    char16_t * ret = p50_param_id_GET(src, malloc((src->items + 1) * 2), 0);
    ret[src->items] = 0;//the terminating null character.
    return ret;
}
INLINER uint16_t p51_seq_GET(Pack * src)//Sequence
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p51_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p51_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER e_MAV_MISSION_TYPE p51_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 32, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p54_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p54_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER float p54_p1x_GET(Pack * src)//x position 1 / Latitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER float p54_p1y_GET(Pack * src)//y position 1 / Longitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER float p54_p1z_GET(Pack * src)//z position 1 / Altitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p54_p2x_GET(Pack * src)//x position 2 / Latitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p54_p2y_GET(Pack * src)//y position 2 / Longitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p54_p2z_GET(Pack * src)//z position 2 / Altitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
/**
*Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
*	with Z axis up or local, right handed, Z axis down*/
INLINER e_MAV_FRAME p54_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 208, 4);
}
INLINER float p55_p1x_GET(Pack * src)//x position 1 / Latitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  0, 4)));
}
INLINER float p55_p1y_GET(Pack * src)//y position 1 / Longitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p55_p1z_GET(Pack * src)//z position 1 / Altitude 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p55_p2x_GET(Pack * src)//x position 2 / Latitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p55_p2y_GET(Pack * src)//y position 2 / Longitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p55_p2z_GET(Pack * src)//z position 2 / Altitude 2
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
/**
*Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed
*	with Z axis up or local, right handed, Z axis down*/
INLINER e_MAV_FRAME p55_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 192, 4);
}
INLINER uint64_t p61_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float* p61_q_GET(Pack * src, float*  dst, int32_t pos) //Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p61_q_LEN = 4; //return array length

INLINER  float*  p61_q_GET_(Pack * src) {return p61_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER float p61_rollspeed_GET(Pack * src)//Roll angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p61_pitchspeed_GET(Pack * src)//Pitch angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p61_yawspeed_GET(Pack * src)//Yaw angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float* p61_covariance_GET(Pack * src, float*  dst, int32_t pos) //Attitude covariance
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 9; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p61_covariance_LEN = 9; //return array length

INLINER  float*  p61_covariance_GET_(Pack * src) {return p61_covariance_GET(src, malloc(9 * sizeof(float)), 0);}
INLINER uint16_t p62_wp_dist_GET(Pack * src)//Distance to active waypoint in meters
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER float p62_nav_roll_GET(Pack * src)//Current desired roll in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER float p62_nav_pitch_GET(Pack * src)//Current desired pitch in degrees
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER int16_t p62_nav_bearing_GET(Pack * src)//Current desired heading in degrees
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER int16_t p62_target_bearing_GET(Pack * src)//Bearing to current waypoint/target in degrees
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  12, 2)));
}
INLINER float p62_alt_error_GET(Pack * src)//Current altitude error in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p62_aspd_error_GET(Pack * src)//Current airspeed error in meters/second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p62_xtrack_error_GET(Pack * src)//Current crosstrack error on x-y plane in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER uint64_t p63_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER int32_t p63_lat_GET(Pack * src)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER int32_t p63_lon_GET(Pack * src)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER int32_t p63_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters), above MSL
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  16, 4)));
}
INLINER int32_t p63_relative_alt_GET(Pack * src)//Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER float p63_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p63_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p63_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float* p63_covariance_GET(Pack * src, float*  dst, int32_t pos) //Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 36, dst_max = pos + 36; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p63_covariance_LEN = 36; //return array length

INLINER  float*  p63_covariance_GET_(Pack * src) {return p63_covariance_GET(src, malloc(36 * sizeof(float)), 0);}
INLINER e_MAV_ESTIMATOR_TYPE p63_estimator_type_GET(Pack * src)//Class id of the estimator this estimate originated from.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 1440, 3);
}
INLINER uint64_t p64_time_usec_GET(Pack * src)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p64_x_GET(Pack * src)//X Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p64_y_GET(Pack * src)//Y Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p64_z_GET(Pack * src)//Z Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p64_vx_GET(Pack * src)//X Speed (m/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p64_vy_GET(Pack * src)//Y Speed (m/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p64_vz_GET(Pack * src)//Z Speed (m/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p64_ax_GET(Pack * src)//X Acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p64_ay_GET(Pack * src)//Y Acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER float p64_az_GET(Pack * src)//Z Acceleration (m/s^2)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
/**
*Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
*	the second row, etc.*/
INLINER float* p64_covariance_GET(Pack * src, float*  dst, int32_t pos)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 44, dst_max = pos + 45; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p64_covariance_LEN = 45; //return array length
/**
*Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are
*	the second row, etc.*/

INLINER  float*  p64_covariance_GET_(Pack * src) {return p64_covariance_GET(src, malloc(45 * sizeof(float)), 0);}
INLINER e_MAV_ESTIMATOR_TYPE p64_estimator_type_GET(Pack * src)//Class id of the estimator this estimate originated from.
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 1792, 3);
}
INLINER uint16_t p65_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p65_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p65_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p65_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p65_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p65_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p65_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p65_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint16_t p65_chan9_raw_GET(Pack * src)//RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER uint16_t p65_chan10_raw_GET(Pack * src)//RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER uint16_t p65_chan11_raw_GET(Pack * src)//RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER uint16_t p65_chan12_raw_GET(Pack * src)//RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER uint16_t p65_chan13_raw_GET(Pack * src)//RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 2)));
}
INLINER uint16_t p65_chan14_raw_GET(Pack * src)//RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  26, 2)));
}
INLINER uint16_t p65_chan15_raw_GET(Pack * src)//RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  28, 2)));
}
INLINER uint16_t p65_chan16_raw_GET(Pack * src)//RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  30, 2)));
}
INLINER uint16_t p65_chan17_raw_GET(Pack * src)//RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 2)));
}
INLINER uint16_t p65_chan18_raw_GET(Pack * src)//RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  34, 2)));
}
INLINER uint32_t p65_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  36, 4)));
}
/**
*Total number of RC channels being received. This can be larger than 18, indicating that more channels
*	are available but not given in this message. This value should be 0 when no RC channels are available*/
INLINER uint8_t p65_chancount_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER uint8_t p65_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  41, 1)));
}
INLINER uint16_t p66_req_message_rate_GET(Pack * src)//The requested message rate
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p66_target_system_GET(Pack * src)//The target requested to send the message stream.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p66_target_component_GET(Pack * src)//The target requested to send the message stream.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p66_req_stream_id_GET(Pack * src)//The ID of the requested data stream
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p66_start_stop_GET(Pack * src)//1 to start sending, 0 to stop sending.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER uint16_t p67_message_rate_GET(Pack * src)//The message rate
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p67_stream_id_GET(Pack * src)//The ID of the requested data stream
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p67_on_off_GET(Pack * src)//1 stream is enabled, 0 stream is stopped.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
/**
*A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest
*	bit corresponds to Button 1*/
INLINER uint16_t p69_buttons_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p69_target_GET(Pack * src)//The system to be controlled.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
/**
*X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
INLINER int16_t p69_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  3, 2)));
}
/**
*Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
INLINER int16_t p69_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
/**
*Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
*	a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
*	thrust*/
INLINER int16_t p69_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
/**
*R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
*	being -1000, and the yaw of a vehicle*/
INLINER int16_t p69_r_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER uint16_t p70_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p70_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p70_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p70_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p70_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p70_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p70_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p70_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint8_t p70_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 1)));
}
INLINER uint8_t p70_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  17, 1)));
}
/**
*Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the
*	sequence (0,1,2,3,4)*/
INLINER uint16_t p73_seq_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint8_t p73_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p73_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER uint8_t p73_current_GET(Pack * src)//false:0, true:1
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p73_autocontinue_GET(Pack * src)//autocontinue to next wp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
INLINER float p73_param1_GET(Pack * src)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER float p73_param2_GET(Pack * src)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  10, 4)));
}
INLINER float p73_param3_GET(Pack * src)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p73_param4_GET(Pack * src)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER int32_t p73_x_GET(Pack * src)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  22, 4)));
}
INLINER int32_t p73_y_GET(Pack * src)//PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  26, 4)));
}
INLINER float p73_z_GET(Pack * src)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER e_MAV_FRAME p73_frame_GET(Pack * src)//The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 272, 4);
}
INLINER e_MAV_CMD p73_command_GET(Pack * src)//The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = src->data;
    switch(get_bits(data, 276, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_MISSION_TYPE p73_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 284, 3))
    {
        case 0:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION;
        case 1:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE;
        case 2:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY;
        case 3:
            return e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p74_throttle_GET(Pack * src)//Current throttle setting in integer percent, 0 to 100
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER float p74_airspeed_GET(Pack * src)//Current airspeed in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  2, 4)));
}
INLINER float p74_groundspeed_GET(Pack * src)//Current ground speed in m/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  6, 4)));
}
INLINER int16_t p74_heading_GET(Pack * src)//Current heading in degrees, in compass units (0..360, 0=north)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  10, 2)));
}
INLINER float p74_alt_GET(Pack * src)//Current altitude (MSL), in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p74_climb_GET(Pack * src)//Current climb rate in meters/second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER uint8_t p75_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p75_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p75_current_GET(Pack * src)//false:0, true:1
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER uint8_t p75_autocontinue_GET(Pack * src)//autocontinue to next wp
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  3, 1)));
}
INLINER float p75_param1_GET(Pack * src)//PARAM1, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p75_param2_GET(Pack * src)//PARAM2, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p75_param3_GET(Pack * src)//PARAM3, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p75_param4_GET(Pack * src)//PARAM4, see MAV_CMD enum
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER int32_t p75_x_GET(Pack * src)//PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  20, 4)));
}
INLINER int32_t p75_y_GET(Pack * src)//PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  24, 4)));
}
INLINER float p75_z_GET(Pack * src)//PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER e_MAV_FRAME p75_frame_GET(Pack * src)//The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 256, 4);
}
INLINER e_MAV_CMD p75_command_GET(Pack * src)//The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
{
    uint8_t * data = src->data;
    switch(get_bits(data, 260, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint8_t p76_target_system_GET(Pack * src)//System which should execute the command
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 1)));
}
INLINER uint8_t p76_target_component_GET(Pack * src)//Component which should execute the command, 0 for all components
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  1, 1)));
}
INLINER uint8_t p76_confirmation_GET(Pack * src)//0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
INLINER float p76_param1_GET(Pack * src)//Parameter 1, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  3, 4)));
}
INLINER float p76_param2_GET(Pack * src)//Parameter 2, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  7, 4)));
}
INLINER float p76_param3_GET(Pack * src)//Parameter 3, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  11, 4)));
}
INLINER float p76_param4_GET(Pack * src)//Parameter 4, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  15, 4)));
}
INLINER float p76_param5_GET(Pack * src)//Parameter 5, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  19, 4)));
}
INLINER float p76_param6_GET(Pack * src)//Parameter 6, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER float p76_param7_GET(Pack * src)//Parameter 7, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  27, 4)));
}
INLINER e_MAV_CMD p76_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 248, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_CMD p77_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 0, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 14:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_ALTITUDE_WAIT;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 23:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 28:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 56:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_GRIPPER;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_AUTOTUNE_ENABLE;
        case 59:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 64:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 65:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 69:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 70:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 71:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 72:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 73:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 75:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 76:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 77:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 84:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 85:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 86:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 87:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 88:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 94:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 95:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 96:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 98:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 99:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 100:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 101:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 102:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 103:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 104:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 105:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 110:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 111:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 112:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 113:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 114:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 130:
            return e_MAV_CMD_MAV_CMD_POWER_OFF_INITIATED;
        case 131:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_CLICK;
        case 132:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_FLY_HOLD;
        case 133:
            return e_MAV_CMD_MAV_CMD_SOLO_BTN_PAUSE_CLICK;
        case 134:
            return e_MAV_CMD_MAV_CMD_DO_START_MAG_CAL;
        case 135:
            return e_MAV_CMD_MAV_CMD_DO_ACCEPT_MAG_CAL;
        case 136:
            return e_MAV_CMD_MAV_CMD_DO_CANCEL_MAG_CAL;
        case 137:
            return e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE;
        case 138:
            return e_MAV_CMD_MAV_CMD_DO_SEND_BANNER;
        case 139:
            return e_MAV_CMD_MAV_CMD_ACCELCAL_VEHICLE_POS;
        case 140:
            return e_MAV_CMD_MAV_CMD_GIMBAL_RESET;
        case 141:
            return e_MAV_CMD_MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS;
        case 142:
            return e_MAV_CMD_MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION;
        case 143:
            return e_MAV_CMD_MAV_CMD_GIMBAL_FULL_RESET;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_RESULT p77_result_GET(Pack * src)//See MAV_RESULT enum
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 8, 3);
}
INLINER uint8_t  p77_progress_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  11 && !try_visit_field(src, 11)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER int32_t  p77_result_param2_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  12 && !try_visit_field(src, 12)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((int32_t)(get_bytes(data,  src->BYTE, 4)));
}
INLINER uint8_t  p77_target_system_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  13 && !try_visit_field(src, 13)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER uint8_t  p77_target_component_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  14 && !try_visit_field(src, 14)) return 0;
    uint8_t * data = src->base.pack->data;
    return ((get_bytes(data,  src->BYTE, 1)));
}
INLINER uint32_t p81_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p81_roll_GET(Pack * src)//Desired roll rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p81_pitch_GET(Pack * src)//Desired pitch rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p81_yaw_GET(Pack * src)//Desired yaw rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p81_thrust_GET(Pack * src)//Collective thrust, normalized to 0 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER uint8_t p81_mode_switch_GET(Pack * src)//Flight mode switch position, 0.. 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 1)));
}
INLINER uint8_t p81_manual_override_switch_GET(Pack * src)//Override mode switch position, 0.. 255
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER uint32_t p82_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER uint8_t p82_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER uint8_t p82_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  5, 1)));
}
/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
INLINER uint8_t p82_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER float* p82_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 7, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p82_q_LEN = 4; //return array length

INLINER  float*  p82_q_GET_(Pack * src) {return p82_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER float p82_body_roll_rate_GET(Pack * src)//Body roll rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  23, 4)));
}
INLINER float p82_body_pitch_rate_GET(Pack * src)//Body roll rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  27, 4)));
}
INLINER float p82_body_yaw_rate_GET(Pack * src)//Body roll rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  31, 4)));
}
INLINER float p82_thrust_GET(Pack * src)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  35, 4)));
}
INLINER uint32_t p83_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
/**
*Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate,
*	bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
INLINER uint8_t p83_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 1)));
}
INLINER float* p83_q_GET(Pack * src, float*  dst, int32_t pos) //Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 5, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p83_q_LEN = 4; //return array length

INLINER  float*  p83_q_GET_(Pack * src) {return p83_q_GET(src, malloc(4 * sizeof(float)), 0);}
INLINER float p83_body_roll_rate_GET(Pack * src)//Body roll rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  21, 4)));
}
INLINER float p83_body_pitch_rate_GET(Pack * src)//Body pitch rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  25, 4)));
}
INLINER float p83_body_yaw_rate_GET(Pack * src)//Body yaw rate in radians per second
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  29, 4)));
}
INLINER float p83_thrust_GET(Pack * src)//Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  33, 4)));
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p84_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint32_t p84_time_boot_ms_GET(Pack * src)//Timestamp in milliseconds since system boot
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER uint8_t p84_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p84_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER float p84_x_GET(Pack * src)//X Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p84_y_GET(Pack * src)//Y Position in NED frame in meters
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p84_z_GET(Pack * src)//Z Position in NED frame in meters (note, altitude is negative in NED)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p84_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p84_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p84_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p84_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p84_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER float p84_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER float p84_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER float p84_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
/**
*Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED
*	=*/
INLINER e_MAV_FRAME p84_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p86_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER uint32_t p86_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER uint8_t p86_target_system_GET(Pack * src)//System ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 1)));
}
INLINER uint8_t p86_target_component_GET(Pack * src)//Component ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  7, 1)));
}
INLINER int32_t p86_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  8, 4)));
}
INLINER int32_t p86_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  12, 4)));
}
INLINER float p86_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p86_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p86_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p86_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p86_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p86_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER float p86_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  40, 4)));
}
INLINER float p86_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  44, 4)));
}
INLINER float p86_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  48, 4)));
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER e_MAV_FRAME p86_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p87_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	processing latency*/
INLINER uint32_t p87_time_boot_ms_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 4)));
}
INLINER int32_t p87_lat_int_GET(Pack * src)//X Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  6, 4)));
}
INLINER int32_t p87_lon_int_GET(Pack * src)//Y Position in WGS84 frame in 1e7 * meters
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  10, 4)));
}
INLINER float p87_alt_GET(Pack * src)//Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_IN
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  14, 4)));
}
INLINER float p87_vx_GET(Pack * src)//X velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  18, 4)));
}
INLINER float p87_vy_GET(Pack * src)//Y velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float p87_vz_GET(Pack * src)//Z velocity in NED frame in meter / s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  26, 4)));
}
INLINER float p87_afx_GET(Pack * src)//X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  30, 4)));
}
INLINER float p87_afy_GET(Pack * src)//Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  34, 4)));
}
INLINER float p87_afz_GET(Pack * src)//Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  38, 4)));
}
INLINER float p87_yaw_GET(Pack * src)//yaw setpoint in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  42, 4)));
}
INLINER float p87_yaw_rate_GET(Pack * src)//yaw rate setpoint in rad/s
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  46, 4)));
}
/**
*Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
*	= 1*/
INLINER e_MAV_FRAME p87_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 400, 4);
}
INLINER uint32_t p89_time_boot_ms_GET(Pack * src)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 4)));
}
INLINER float p89_x_GET(Pack * src)//X Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  4, 4)));
}
INLINER float p89_y_GET(Pack * src)//Y Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p89_z_GET(Pack * src)//Z Position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p89_roll_GET(Pack * src)//Roll
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p89_pitch_GET(Pack * src)//Pitch
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p89_yaw_GET(Pack * src)//Yaw
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER uint64_t p90_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p90_roll_GET(Pack * src)//Roll angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p90_pitch_GET(Pack * src)//Pitch angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p90_yaw_GET(Pack * src)//Yaw angle (rad)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p90_rollspeed_GET(Pack * src)//Body frame roll / phi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p90_pitchspeed_GET(Pack * src)//Body frame pitch / theta angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p90_yawspeed_GET(Pack * src)//Body frame yaw / psi angular speed (rad/s)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER int32_t p90_lat_GET(Pack * src)//Latitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  32, 4)));
}
INLINER int32_t p90_lon_GET(Pack * src)//Longitude, expressed as * 1E7
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  36, 4)));
}
INLINER int32_t p90_alt_GET(Pack * src)//Altitude in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = src->data;
    return ((int32_t)(get_bytes(data,  40, 4)));
}
INLINER int16_t p90_vx_GET(Pack * src)//Ground X Speed (Latitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  44, 2)));
}
INLINER int16_t p90_vy_GET(Pack * src)//Ground Y Speed (Longitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  46, 2)));
}
INLINER int16_t p90_vz_GET(Pack * src)//Ground Z Speed (Altitude), expressed as m/s * 100
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  48, 2)));
}
INLINER int16_t p90_xacc_GET(Pack * src)//X acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  50, 2)));
}
INLINER int16_t p90_yacc_GET(Pack * src)//Y acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  52, 2)));
}
INLINER int16_t p90_zacc_GET(Pack * src)//Z acceleration (mg)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  54, 2)));
}
INLINER uint64_t p91_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p91_roll_ailerons_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p91_pitch_elevator_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p91_yaw_rudder_GET(Pack * src)//Control output -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p91_throttle_GET(Pack * src)//Throttle 0 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p91_aux1_GET(Pack * src)//Aux 1, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p91_aux2_GET(Pack * src)//Aux 2, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER float p91_aux3_GET(Pack * src)//Aux 3, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  32, 4)));
}
INLINER float p91_aux4_GET(Pack * src)//Aux 4, -1 .. 1
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  36, 4)));
}
INLINER uint8_t p91_nav_mode_GET(Pack * src)//Navigation mode (MAV_NAV_MODE)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  40, 1)));
}
INLINER e_MAV_MODE p91_mode_GET(Pack * src)//System mode (MAV_MODE)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 328, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint16_t p92_chan1_raw_GET(Pack * src)//RC channel 1 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
INLINER uint16_t p92_chan2_raw_GET(Pack * src)//RC channel 2 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 2)));
}
INLINER uint16_t p92_chan3_raw_GET(Pack * src)//RC channel 3 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
INLINER uint16_t p92_chan4_raw_GET(Pack * src)//RC channel 4 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  6, 2)));
}
INLINER uint16_t p92_chan5_raw_GET(Pack * src)//RC channel 5 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 2)));
}
INLINER uint16_t p92_chan6_raw_GET(Pack * src)//RC channel 6 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  10, 2)));
}
INLINER uint16_t p92_chan7_raw_GET(Pack * src)//RC channel 7 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 2)));
}
INLINER uint16_t p92_chan8_raw_GET(Pack * src)//RC channel 8 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  14, 2)));
}
INLINER uint16_t p92_chan9_raw_GET(Pack * src)//RC channel 9 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  16, 2)));
}
INLINER uint16_t p92_chan10_raw_GET(Pack * src)//RC channel 10 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  18, 2)));
}
INLINER uint16_t p92_chan11_raw_GET(Pack * src)//RC channel 11 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  20, 2)));
}
INLINER uint16_t p92_chan12_raw_GET(Pack * src)//RC channel 12 value, in microseconds
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  22, 2)));
}
INLINER uint64_t p92_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  24, 8)));
}
INLINER uint8_t p92_rssi_GET(Pack * src)//Receive signal strength indicator, 0: 0%, 255: 100%
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  32, 1)));
}
INLINER uint64_t p93_time_usec_GET(Pack * src)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER uint64_t p93_flags_GET(Pack * src)//Flags as bitfield, reserved for future use.
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 8)));
}
INLINER float* p93_controls_GET(Pack * src, float*  dst, int32_t pos) //Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
{
    uint8_t * data = src->data;
    for(int32_t BYTE = 16, dst_max = pos + 16; pos < dst_max ; pos++,  BYTE += 4)
        dst[pos] = (intBitsToFloat(get_bytes(data,  BYTE, 4)));
    return dst;
}

static const  uint32_t p93_controls_LEN = 16; //return array length

INLINER  float*  p93_controls_GET_(Pack * src) {return p93_controls_GET(src, malloc(16 * sizeof(float)), 0);}
INLINER e_MAV_MODE p93_mode_GET(Pack * src)//System mode (MAV_MODE), includes arming state.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 640, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER uint64_t p100_time_usec_GET(Pack * src)//Timestamp (UNIX)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER uint8_t p100_sensor_id_GET(Pack * src)//Sensor ID
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  8, 1)));
}
INLINER int16_t p100_flow_x_GET(Pack * src)//Flow in pixels * 10 in x-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  9, 2)));
}
INLINER int16_t p100_flow_y_GET(Pack * src)//Flow in pixels * 10 in y-sensor direction (dezi-pixels)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  11, 2)));
}
INLINER float p100_flow_comp_m_x_GET(Pack * src)//Flow in meters in x-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
INLINER float p100_flow_comp_m_y_GET(Pack * src)//Flow in meters in y-sensor direction, angular-speed compensated
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
INLINER uint8_t p100_quality_GET(Pack * src)//Optical flow quality / confidence. 0: bad, 255: maximum quality
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  21, 1)));
}
INLINER float p100_ground_distance_GET(Pack * src)//Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  22, 4)));
}
INLINER float  p100_flow_rate_x_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  208 && !try_visit_field(src, 208)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER float  p100_flow_rate_y_TRY(Bounds_Inside * src)
{
    if(src->base.field_bit !=  209 && !try_visit_field(src, 209)) return 0;
    uint8_t * data = src->base.pack->data;
    return (intBitsToFloat(get_bytes(data,  src->BYTE, 4)));
}
INLINER uint64_t p101_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p101_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p101_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p101_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p101_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p101_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p101_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER uint64_t p102_usec_GET(Pack * src)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 8)));
}
INLINER float p102_x_GET(Pack * src)//Global X position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  8, 4)));
}
INLINER float p102_y_GET(Pack * src)//Global Y position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  12, 4)));
}
INLINER float p102_z_GET(Pack * src)//Global Z position
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  16, 4)));
}
INLINER float p102_roll_GET(Pack * src)//Roll angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  20, 4)));
}
INLINER float p102_pitch_GET(Pack * src)//Pitch angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  24, 4)));
}
INLINER float p102_yaw_GET(Pack * src)//Yaw angle in rad
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  28, 4)));
}
INLINER void p150_mag_ofs_x_SET(int16_t  src, Pack * dst)//magnetometer X offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  0);
}
INLINER void p150_mag_ofs_y_SET(int16_t  src, Pack * dst)//magnetometer Y offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p150_mag_ofs_z_SET(int16_t  src, Pack * dst)//magnetometer Z offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p150_mag_declination_SET(float  src, Pack * dst)//magnetic declination (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p150_raw_press_SET(int32_t  src, Pack * dst)//raw pressure from barometer
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p150_raw_temp_SET(int32_t  src, Pack * dst)//raw temperature from barometer
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p150_gyro_cal_x_SET(float  src, Pack * dst)//gyro X calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p150_gyro_cal_y_SET(float  src, Pack * dst)//gyro Y calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p150_gyro_cal_z_SET(float  src, Pack * dst)//gyro Z calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p150_accel_cal_x_SET(float  src, Pack * dst)//accel X calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p150_accel_cal_y_SET(float  src, Pack * dst)//accel Y calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p150_accel_cal_z_SET(float  src, Pack * dst)//accel Z calibration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
Pack * c_TEST_Channel_new_SENSOR_OFFSETS_150()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 150));
};
INLINER void p151_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p151_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p151_mag_ofs_x_SET(int16_t  src, Pack * dst)//magnetometer X offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p151_mag_ofs_y_SET(int16_t  src, Pack * dst)//magnetometer Y offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p151_mag_ofs_z_SET(int16_t  src, Pack * dst)//magnetometer Z offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
Pack * c_TEST_Channel_new_SET_MAG_OFFSETS_151()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 151));
};
INLINER void p152_brkval_SET(uint16_t  src, Pack * dst)//heap top
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p152_freemem_SET(uint16_t  src, Pack * dst)//free memory
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p152_freemem32_SET(uint32_t  src, Bounds_Inside * dst)//free memory (32 bit)
{
    if(dst->base.field_bit != 32)insert_field(dst, 32, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 4, data,  dst->BYTE);
}
Pack * c_TEST_Channel_new_MEMINFO_152()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 152));
};
INLINER void p153_adc1_SET(uint16_t  src, Pack * dst)//ADC output 1
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p153_adc2_SET(uint16_t  src, Pack * dst)//ADC output 2
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p153_adc3_SET(uint16_t  src, Pack * dst)//ADC output 3
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p153_adc4_SET(uint16_t  src, Pack * dst)//ADC output 4
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p153_adc5_SET(uint16_t  src, Pack * dst)//ADC output 5
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p153_adc6_SET(uint16_t  src, Pack * dst)//ADC output 6
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
Pack * c_TEST_Channel_new_AP_ADC_153()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 153));
};
INLINER void p154_shutter_speed_SET(uint16_t  src, Pack * dst)//Divisor number e.g. 1000 means 1/1000 (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p154_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p154_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p154_mode_SET(uint8_t  src, Pack * dst)//Mode enumeration from 1 to N P, TV, AV, M, Etc (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p154_aperture_SET(uint8_t  src, Pack * dst)//F stop number x 10 e.g. 28 means 2.8 (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p154_iso_SET(uint8_t  src, Pack * dst)//ISO enumeration from 1 to N e.g. 80, 100, 200, Etc (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p154_exposure_type_SET(uint8_t  src, Pack * dst)//Exposure type enumeration from 1 to N (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*	just onc*/
INLINER void p154_command_id_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p154_engine_cut_off_SET(uint8_t  src, Pack * dst)//Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p154_extra_param_SET(uint8_t  src, Pack * dst)//Extra parameters enumeration (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p154_extra_value_SET(float  src, Pack * dst)//Correspondent value to given extra_param
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
Pack * c_TEST_Channel_new_DIGICAM_CONFIGURE_154()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 154));
};
INLINER void p155_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p155_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p155_session_SET(uint8_t  src, Pack * dst)//0: stop, 1: start or keep it up Session control e.g. show/hide lens
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p155_zoom_pos_SET(uint8_t  src, Pack * dst)//1 to N Zoom's absolute position (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p155_zoom_step_SET(int8_t  src, Pack * dst)//-100 to 100 Zooming step value to offset zoom from the current position
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  4);
}
INLINER void p155_focus_lock_SET(uint8_t  src, Pack * dst)//0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p155_shot_SET(uint8_t  src, Pack * dst)//0: ignore, 1: shot or start filming
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
/**
*Command Identity (incremental loop: 0 to 255)A command sent multiple times will be executed or pooled
*	just onc*/
INLINER void p155_command_id_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p155_extra_param_SET(uint8_t  src, Pack * dst)//Extra parameters enumeration (0 means ignore)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p155_extra_value_SET(float  src, Pack * dst)//Correspondent value to given extra_param
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
Pack * c_TEST_Channel_new_DIGICAM_CONTROL_155()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 155));
};
INLINER void p156_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p156_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p156_stab_roll_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p156_stab_pitch_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p156_stab_yaw_SET(uint8_t  src, Pack * dst)//(1 = yes, 0 = no)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p156_mount_mode_SET(e_MAV_MOUNT_MODE  src, Pack * dst)//mount operating mode (see MAV_MOUNT_MODE enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 40);
}
Pack * c_TEST_Channel_new_MOUNT_CONFIGURE_156()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 156));
};
INLINER void p157_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p157_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p157_input_a_SET(int32_t  src, Pack * dst)//pitch(deg*100) or lat, depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER void p157_input_b_SET(int32_t  src, Pack * dst)//roll(deg*100) or lon depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p157_input_c_SET(int32_t  src, Pack * dst)//yaw(deg*100) or alt (in cm) depending on mount mode
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p157_save_position_SET(uint8_t  src, Pack * dst)//if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
Pack * c_TEST_Channel_new_MOUNT_CONTROL_157()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 157));
};
INLINER void p158_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p158_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p158_pointing_a_SET(int32_t  src, Pack * dst)//pitch(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER void p158_pointing_b_SET(int32_t  src, Pack * dst)//roll(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p158_pointing_c_SET(int32_t  src, Pack * dst)//yaw(deg*100)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_MOUNT_STATUS_158()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 158));
};
INLINER void p160_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p160_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p160_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 1, 0 is for return point)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p160_count_SET(uint8_t  src, Pack * dst)//total number of points (for sanity checking)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p160_lat_SET(float  src, Pack * dst)//Latitude of point
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p160_lng_SET(float  src, Pack * dst)//Longitude of point
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
Pack * c_TEST_Channel_new_FENCE_POINT_160()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 160));
};
INLINER void p161_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p161_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p161_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 1, 0 is for return point)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_FENCE_FETCH_POINT_161()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 161));
};
INLINER void p162_breach_count_SET(uint16_t  src, Pack * dst)//number of fence breaches
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p162_breach_time_SET(uint32_t  src, Pack * dst)//time of last breach in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p162_breach_status_SET(uint8_t  src, Pack * dst)//0 if currently inside fence, 1 if outside
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p162_breach_type_SET(e_FENCE_BREACH  src, Pack * dst)//last breach type (see FENCE_BREACH_* enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 56);
}
Pack * c_TEST_Channel_new_FENCE_STATUS_162()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 162));
};
INLINER void p163_omegaIx_SET(float  src, Pack * dst)//X gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p163_omegaIy_SET(float  src, Pack * dst)//Y gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p163_omegaIz_SET(float  src, Pack * dst)//Z gyro drift estimate rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p163_accel_weight_SET(float  src, Pack * dst)//average accel_weight
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p163_renorm_val_SET(float  src, Pack * dst)//average renormalisation value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p163_error_rp_SET(float  src, Pack * dst)//average error_roll_pitch value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p163_error_yaw_SET(float  src, Pack * dst)//average error_yaw value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
Pack * c_TEST_Channel_new_AHRS_163()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 163));
};
INLINER void p164_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p164_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p164_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p164_xacc_SET(float  src, Pack * dst)//X acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p164_yacc_SET(float  src, Pack * dst)//Y acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p164_zacc_SET(float  src, Pack * dst)//Z acceleration m/s/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p164_xgyro_SET(float  src, Pack * dst)//Angular speed around X axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p164_ygyro_SET(float  src, Pack * dst)//Angular speed around Y axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p164_zgyro_SET(float  src, Pack * dst)//Angular speed around Z axis rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p164_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  36);
}
INLINER void p164_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  40);
}
Pack * c_TEST_Channel_new_SIMSTATE_164()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 164));
};
INLINER void p165_Vcc_SET(uint16_t  src, Pack * dst)//board voltage (mV)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p165_I2Cerr_SET(uint8_t  src, Pack * dst)//I2C error count
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_HWSTATUS_165()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 165));
};
INLINER void p166_rxerrors_SET(uint16_t  src, Pack * dst)//receive errors
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p166_fixed__SET(uint16_t  src, Pack * dst)//count of error corrected packets
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p166_rssi_SET(uint8_t  src, Pack * dst)//local signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p166_remrssi_SET(uint8_t  src, Pack * dst)//remote signal strength
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p166_txbuf_SET(uint8_t  src, Pack * dst)//how full the tx buffer is as a percentage
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p166_noise_SET(uint8_t  src, Pack * dst)//background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p166_remnoise_SET(uint8_t  src, Pack * dst)//remote background noise level
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
Pack * c_TEST_Channel_new_RADIO_166()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 166));
};
INLINER void p167_breach_count_SET(uint16_t  src, Pack * dst)//number of fence breaches
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p167_last_trigger_SET(uint32_t  src, Pack * dst)//time of last breach in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p167_last_action_SET(uint32_t  src, Pack * dst)//time of last recovery action in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p167_last_recovery_SET(uint32_t  src, Pack * dst)//time of last successful recovery in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER void p167_last_clear_SET(uint32_t  src, Pack * dst)//time of last all-clear in milliseconds since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  14);
}
INLINER void p167_limits_state_SET(e_LIMITS_STATE  src, Pack * dst)//state of AP_Limits, (see enum LimitState, LIMITS_STATE)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 144);
}
INLINER void p167_mods_enabled_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 147);
}
INLINER void p167_mods_required_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 150);
}
INLINER void p167_mods_triggered_SET(e_LIMIT_MODULE  src, Pack * dst)//AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 153);
}
Pack * c_TEST_Channel_new_LIMITS_STATUS_167()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 167));
};
INLINER void p168_direction_SET(float  src, Pack * dst)//wind direction that wind is coming from (degrees)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p168_speed_SET(float  src, Pack * dst)//wind speed in ground plane (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p168_speed_z_SET(float  src, Pack * dst)//vertical wind speed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
Pack * c_TEST_Channel_new_WIND_168()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 168));
};
INLINER void p169_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p169_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p169_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA16_169()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 169));
};
INLINER void p170_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p170_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p170_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA32_170()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 170));
};
INLINER void p171_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p171_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p171_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 64; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA64_171()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 171));
};
INLINER void p172_type_SET(uint8_t  src, Pack * dst)//data type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p172_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p172_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //raw data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 96; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DATA96_172()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 172));
};
INLINER void p173_distance_SET(float  src, Pack * dst)//distance in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p173_voltage_SET(float  src, Pack * dst)//raw voltage if available, zero otherwise
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
Pack * c_TEST_Channel_new_RANGEFINDER_173()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 173));
};
INLINER void p174_vx_SET(float  src, Pack * dst)//GPS velocity north m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p174_vy_SET(float  src, Pack * dst)//GPS velocity east m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p174_vz_SET(float  src, Pack * dst)//GPS velocity down m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p174_diff_pressure_SET(float  src, Pack * dst)//Differential pressure pascals
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p174_EAS2TAS_SET(float  src, Pack * dst)//Estimated to true airspeed ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p174_ratio_SET(float  src, Pack * dst)//Airspeed ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p174_state_x_SET(float  src, Pack * dst)//EKF state x
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p174_state_y_SET(float  src, Pack * dst)//EKF state y
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p174_state_z_SET(float  src, Pack * dst)//EKF state z
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p174_Pax_SET(float  src, Pack * dst)//EKF Pax
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p174_Pby_SET(float  src, Pack * dst)//EKF Pby
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p174_Pcz_SET(float  src, Pack * dst)//EKF Pcz
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
Pack * c_TEST_Channel_new_AIRSPEED_AUTOCAL_174()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 174));
};
INLINER void p175_land_dir_SET(uint16_t  src, Pack * dst)//Heading to aim for when landing. In centi-degrees.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p175_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p175_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p175_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 0)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p175_count_SET(uint8_t  src, Pack * dst)//total number of points (for sanity checking)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p175_lat_SET(int32_t  src, Pack * dst)//Latitude of point in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p175_lng_SET(int32_t  src, Pack * dst)//Longitude of point in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p175_alt_SET(int16_t  src, Pack * dst)//Transit / loiter altitude in meters relative to home
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p175_break_alt_SET(int16_t  src, Pack * dst)//Break altitude in meters relative to home
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p175_flags_SET(e_RALLY_FLAGS  src, Pack * dst)//See RALLY_FLAGS enum for definition of the bitmask.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 1, data, 144);
}
Pack * c_TEST_Channel_new_RALLY_POINT_175()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 175));
};
INLINER void p176_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p176_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p176_idx_SET(uint8_t  src, Pack * dst)//point index (first point is 0)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_RALLY_FETCH_POINT_176()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 176));
};
INLINER void p177_throttle_SET(uint16_t  src, Pack * dst)//throttle (percent*10)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p177_interference_SET(uint16_t  src, Pack * dst)//interference (percent)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p177_current_SET(float  src, Pack * dst)//current (Ampere)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p177_CompensationX_SET(float  src, Pack * dst)//Motor Compensation X
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p177_CompensationY_SET(float  src, Pack * dst)//Motor Compensation Y
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p177_CompensationZ_SET(float  src, Pack * dst)//Motor Compensation Z
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
Pack * c_TEST_Channel_new_COMPASSMOT_STATUS_177()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 177));
};
INLINER void p178_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p178_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p178_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p178_altitude_SET(float  src, Pack * dst)//Altitude (MSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p178_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p178_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
Pack * c_TEST_Channel_new_AHRS2_178()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 178));
};
INLINER void p179_img_idx_SET(uint16_t  src, Pack * dst)//Image index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p179_time_usec_SET(uint64_t  src, Pack * dst)//Image timestamp (microseconds since UNIX epoch, according to camera clock)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER void p179_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p179_cam_idx_SET(uint8_t  src, Pack * dst)//Camera ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p179_p1_SET(float  src, Pack * dst)//Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p179_p2_SET(float  src, Pack * dst)//Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p179_p3_SET(float  src, Pack * dst)//Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p179_p4_SET(float  src, Pack * dst)//Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p179_event_id_SET(e_CAMERA_STATUS_TYPES  src, Pack * dst)//See CAMERA_STATUS_TYPES enum for definition of the bitmask
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 224);
}
Pack * c_TEST_Channel_new_CAMERA_STATUS_179()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 179));
};
INLINER void p180_img_idx_SET(uint16_t  src, Pack * dst)//Image index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if
*	no CCB*/
INLINER void p180_time_usec_SET(uint64_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER void p180_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p180_cam_idx_SET(uint8_t  src, Pack * dst)//Camera ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p180_lat_SET(int32_t  src, Pack * dst)//Latitude in (deg * 1E7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER void p180_lng_SET(int32_t  src, Pack * dst)//Longitude in (deg * 1E7)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p180_alt_msl_SET(float  src, Pack * dst)//Altitude Absolute (meters AMSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p180_alt_rel_SET(float  src, Pack * dst)//Altitude Relative (meters above HOME location)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p180_roll_SET(float  src, Pack * dst)//Camera Roll angle (earth frame, degrees, +-180)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p180_pitch_SET(float  src, Pack * dst)//Camera Pitch angle (earth frame, degrees, +-180)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p180_yaw_SET(float  src, Pack * dst)//Camera Yaw (earth frame, degrees, 0-360, true)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p180_foc_len_SET(float  src, Pack * dst)//Focal Length (mm)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS  src, Pack * dst)//See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 352);
}
Pack * c_TEST_Channel_new_CAMERA_FEEDBACK_180()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 180));
};
INLINER void p181_voltage_SET(uint16_t  src, Pack * dst)//voltage in millivolts
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p181_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
Pack * c_TEST_Channel_new_BATTERY2_181()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 181));
};
INLINER void p182_roll_SET(float  src, Pack * dst)//Roll angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p182_pitch_SET(float  src, Pack * dst)//Pitch angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p182_yaw_SET(float  src, Pack * dst)//Yaw angle (rad)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p182_altitude_SET(float  src, Pack * dst)//Altitude (MSL)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p182_lat_SET(int32_t  src, Pack * dst)//Latitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p182_lng_SET(int32_t  src, Pack * dst)//Longitude in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p182_v1_SET(float  src, Pack * dst)//test variable1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p182_v2_SET(float  src, Pack * dst)//test variable2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p182_v3_SET(float  src, Pack * dst)//test variable3
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p182_v4_SET(float  src, Pack * dst)//test variable4
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
Pack * c_TEST_Channel_new_AHRS3_182()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 182));
};
INLINER void p183_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p183_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
Pack * c_TEST_Channel_new_AUTOPILOT_VERSION_REQUEST_183()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 183));
};
INLINER void p184_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p184_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p184_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //log data block
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 200; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  src, Pack * dst)//log data block sequence number
{
    uint8_t * data = dst->data;
    set_bits(- 2147483645 +   src, 1, data, 1616);
}
Pack * c_TEST_Channel_new_REMOTE_LOG_DATA_BLOCK_184()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 184));
};
INLINER void p185_seqno_SET(uint32_t  src, Pack * dst)//log data block sequence number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p185_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p185_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  src, Pack * dst)//log data block status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 48);
}
Pack * c_TEST_Channel_new_REMOTE_LOG_BLOCK_STATUS_185()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 185));
};
INLINER void p186_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p186_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p186_instance_SET(uint8_t  src, Pack * dst)//Instance (LED instance to control or 255 for all LEDs)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p186_pattern_SET(uint8_t  src, Pack * dst)//Pattern (see LED_PATTERN_ENUM)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p186_custom_len_SET(uint8_t  src, Pack * dst)//Custom Byte Length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p186_custom_bytes_SET(uint8_t*  src, int32_t pos, Pack * dst) //Custom Bytes
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 24; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_LED_CONTROL_186()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 186));
};
INLINER void p191_compass_id_SET(uint8_t  src, Pack * dst)//Compass being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p191_cal_mask_SET(uint8_t  src, Pack * dst)//Bitmask of compasses being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p191_attempt_SET(uint8_t  src, Pack * dst)//Attempt number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p191_completion_pct_SET(uint8_t  src, Pack * dst)//Completion percentage
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p191_completion_mask_SET(uint8_t*  src, int32_t pos, Pack * dst) //Bitmask of sphere sections (see http:en.wikipedia.org/wiki/Geodesic_grid)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 10; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p191_direction_x_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p191_direction_y_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p191_direction_z_SET(float  src, Pack * dst)//Body frame direction vector for display
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p191_cal_status_SET(e_MAG_CAL_STATUS  src, Pack * dst)//Status (see MAG_CAL_STATUS enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 208);
}
Pack * c_TEST_Channel_new_MAG_CAL_PROGRESS_191()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 191));
};
INLINER void p192_compass_id_SET(uint8_t  src, Pack * dst)//Compass being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p192_cal_mask_SET(uint8_t  src, Pack * dst)//Bitmask of compasses being calibrated
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p192_autosaved_SET(uint8_t  src, Pack * dst)//0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p192_fitness_SET(float  src, Pack * dst)//RMS milligauss residuals
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  3);
}
INLINER void p192_ofs_x_SET(float  src, Pack * dst)//X offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER void p192_ofs_y_SET(float  src, Pack * dst)//Y offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER void p192_ofs_z_SET(float  src, Pack * dst)//Z offset
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER void p192_diag_x_SET(float  src, Pack * dst)//X diagonal (matrix 11)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER void p192_diag_y_SET(float  src, Pack * dst)//Y diagonal (matrix 22)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER void p192_diag_z_SET(float  src, Pack * dst)//Z diagonal (matrix 33)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER void p192_offdiag_x_SET(float  src, Pack * dst)//X off-diagonal (matrix 12 and 21)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  31);
}
INLINER void p192_offdiag_y_SET(float  src, Pack * dst)//Y off-diagonal (matrix 13 and 31)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  35);
}
INLINER void p192_offdiag_z_SET(float  src, Pack * dst)//Z off-diagonal (matrix 32 and 23)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  39);
}
INLINER void p192_cal_status_SET(e_MAG_CAL_STATUS  src, Pack * dst)//Status (see MAG_CAL_STATUS enum)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 344);
}
Pack * c_TEST_Channel_new_MAG_CAL_REPORT_192()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 192));
};
INLINER void p193_velocity_variance_SET(float  src, Pack * dst)//Velocity variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p193_pos_horiz_variance_SET(float  src, Pack * dst)//Horizontal Position variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p193_pos_vert_variance_SET(float  src, Pack * dst)//Vertical Position variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p193_compass_variance_SET(float  src, Pack * dst)//Compass variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p193_terrain_alt_variance_SET(float  src, Pack * dst)//Terrain Altitude variance
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p193_flags_SET(e_EKF_STATUS_FLAGS  src, Pack * dst)//Flags
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 10, data, 160);
}
Pack * c_TEST_Channel_new_EKF_STATUS_REPORT_193()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 193));
};
INLINER void p194_desired_SET(float  src, Pack * dst)//desired rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p194_achieved_SET(float  src, Pack * dst)//achieved rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p194_FF_SET(float  src, Pack * dst)//FF component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p194_P_SET(float  src, Pack * dst)//P component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p194_I_SET(float  src, Pack * dst)//I component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p194_D_SET(float  src, Pack * dst)//D component
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p194_axis_SET(e_PID_TUNING_AXIS  src, Pack * dst)//axis
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 192);
}
Pack * c_TEST_Channel_new_PID_TUNING_194()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 194));
};
INLINER void p200_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p200_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p200_delta_time_SET(float  src, Pack * dst)//Time since last update (seconds)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p200_delta_angle_x_SET(float  src, Pack * dst)//Delta angle X (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p200_delta_angle_y_SET(float  src, Pack * dst)//Delta angle Y (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p200_delta_angle_z_SET(float  src, Pack * dst)//Delta angle X (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p200_delta_velocity_x_SET(float  src, Pack * dst)//Delta velocity X (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p200_delta_velocity_y_SET(float  src, Pack * dst)//Delta velocity Y (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p200_delta_velocity_z_SET(float  src, Pack * dst)//Delta velocity Z (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p200_joint_roll_SET(float  src, Pack * dst)//Joint ROLL (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p200_joint_el_SET(float  src, Pack * dst)//Joint EL (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p200_joint_az_SET(float  src, Pack * dst)//Joint AZ (radians)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
Pack * c_TEST_Channel_new_GIMBAL_REPORT_200()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 200));
};
INLINER void p201_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p201_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p201_demanded_rate_x_SET(float  src, Pack * dst)//Demanded angular rate X (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p201_demanded_rate_y_SET(float  src, Pack * dst)//Demanded angular rate Y (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p201_demanded_rate_z_SET(float  src, Pack * dst)//Demanded angular rate Z (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_GIMBAL_CONTROL_201()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 201));
};
INLINER void p214_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p214_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p214_rl_torque_cmd_SET(int16_t  src, Pack * dst)//Roll Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p214_el_torque_cmd_SET(int16_t  src, Pack * dst)//Elevation Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p214_az_torque_cmd_SET(int16_t  src, Pack * dst)//Azimuth Torque Command
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
Pack * c_TEST_Channel_new_GIMBAL_TORQUE_CMD_REPORT_214()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 214));
};
INLINER void p215_status_SET(e_GOPRO_HEARTBEAT_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 0);
}
INLINER void p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE  src, Pack * dst)//Current capture mode
{
    uint8_t * data = dst->data;
    UMAX id;
    switch(src)
    {
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO:
            id = 0;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PHOTO:
            id = 1;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_BURST:
            id = 2;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE:
            id = 3;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_MULTI_SHOT:
            id = 4;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_PLAYBACK:
            id = 5;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP:
            id = 6;
            break;
        case e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_UNKNOWN:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 2);
}
INLINER void p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS  src, Pack * dst)//additional status bits
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 1, data, 6);
}
Pack * c_TEST_Channel_new_GOPRO_HEARTBEAT_215()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 215));
};
INLINER void p216_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p216_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p216_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 16);
}
Pack * c_TEST_Channel_new_GOPRO_GET_REQUEST_216()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 216));
};
INLINER void p217_value_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p217_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 32);
}
INLINER void p217_status_SET(e_GOPRO_REQUEST_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 37);
}
Pack * c_TEST_Channel_new_GOPRO_GET_RESPONSE_217()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 217));
};
INLINER void p218_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p218_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p218_value_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p218_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 48);
}
Pack * c_TEST_Channel_new_GOPRO_SET_REQUEST_218()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 218));
};
INLINER void p219_cmd_id_SET(e_GOPRO_COMMAND  src, Pack * dst)//Command ID
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 0);
}
INLINER void p219_status_SET(e_GOPRO_REQUEST_STATUS  src, Pack * dst)//Status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 5);
}
Pack * c_TEST_Channel_new_GOPRO_SET_RESPONSE_219()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 219));
};
INLINER void p226_rpm1_SET(float  src, Pack * dst)//RPM Sensor1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p226_rpm2_SET(float  src, Pack * dst)//RPM Sensor2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
Pack * c_TEST_Channel_new_RPM_226()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 226));
};
INLINER void p265_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p265_roll_SET(float  src, Pack * dst)//Roll in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p265_pitch_SET(float  src, Pack * dst)//Pitch in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p265_yaw_SET(float  src, Pack * dst)//Yaw in degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
Pack * c_TEST_Channel_new_MOUNT_ORIENTATION_265()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 265));
};
INLINER void p266_sequence_SET(uint16_t  src, Pack * dst)//sequence number (can wrap)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p266_target_system_SET(uint8_t  src, Pack * dst)//system ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p266_target_component_SET(uint8_t  src, Pack * dst)//component ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p266_length_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*	lost (set to 255 if no start exists)*/
INLINER void p266_first_message_offset_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p266_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //logged data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_LOGGING_DATA_266()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 266));
};
INLINER void p267_sequence_SET(uint16_t  src, Pack * dst)//sequence number (can wrap)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p267_target_system_SET(uint8_t  src, Pack * dst)//system ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p267_target_component_SET(uint8_t  src, Pack * dst)//component ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p267_length_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
/**
*offset into data where first message starts. This can be used for recovery, when a previous message got
*	lost (set to 255 if no start exists)*/
INLINER void p267_first_message_offset_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p267_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //logged data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_LOGGING_DATA_ACKED_267()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 267));
};
INLINER void p268_sequence_SET(uint16_t  src, Pack * dst)//sequence number (must match the one in LOGGING_DATA_ACKED)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p268_target_system_SET(uint8_t  src, Pack * dst)//system ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p268_target_component_SET(uint8_t  src, Pack * dst)//component ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
Pack * c_TEST_Channel_new_LOGGING_ACK_268()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 268));
};
INLINER void p269_resolution_h_SET(uint16_t  src, Pack * dst)//Resolution horizontal in pixels
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p269_resolution_v_SET(uint16_t  src, Pack * dst)//Resolution vertical in pixels
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p269_rotation_SET(uint16_t  src, Pack * dst)//Video image rotation clockwise
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p269_bitrate_SET(uint32_t  src, Pack * dst)//Bit rate in bits per second
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p269_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p269_status_SET(uint8_t  src, Pack * dst)//Current status of video streaming (0: not running, 1: in progress)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p269_framerate_SET(float  src, Pack * dst)//Frames per second
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p269_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Video stream URI
{
    if(dst->base.field_bit != 130 && insert_field(dst, 130, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p269_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p269_uri_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 269));
};
INLINER void p270_resolution_h_SET(uint16_t  src, Pack * dst)//Resolution horizontal in pixels (set to -1 for highest resolution possible)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p270_resolution_v_SET(uint16_t  src, Pack * dst)//Resolution vertical in pixels (set to -1 for highest resolution possible)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p270_rotation_SET(uint16_t  src, Pack * dst)//Video image rotation clockwise (0-359 degrees)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p270_bitrate_SET(uint32_t  src, Pack * dst)//Bit rate in bits per second (set to -1 for auto)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p270_target_system_SET(uint8_t  src, Pack * dst)//system ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p270_target_component_SET(uint8_t  src, Pack * dst)//component ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p270_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER void p270_framerate_SET(float  src, Pack * dst)//Frames per second (set to -1 for highest framerate possible)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER void p270_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Video stream URI
{
    if(dst->base.field_bit != 138 && insert_field(dst, 138, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p270_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p270_uri_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 270));
};
INLINER void p299_ssid_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged.
{
    if(dst->base.field_bit != 2 && insert_field(dst, 2, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p299_ssid_SET_(char16_t*  src, Bounds_Inside * dst) {p299_ssid_SET(src, 0, strlen16(src), dst);}
INLINER void p299_password_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Password. Leave it blank for an open AP.
{
    if(dst->base.field_bit != 3 && insert_field(dst, 3, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p299_password_SET_(char16_t*  src, Bounds_Inside * dst) {p299_password_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_WIFI_CONFIG_AP_299()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 299));
};
INLINER void p300_version_SET(uint16_t  src, Pack * dst)//Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p300_min_version_SET(uint16_t  src, Pack * dst)//Minimum MAVLink version supported
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p300_max_version_SET(uint16_t  src, Pack * dst)//Maximum MAVLink version supported (set to the same value as version by default)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p300_spec_version_hash_SET(uint8_t*  src, int32_t pos, Pack * dst) //The first 8 bytes (not characters printed in hex!) of the git hash.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  6, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p300_library_version_hash_SET(uint8_t*  src, int32_t pos, Pack * dst) //The first 8 bytes (not characters printed in hex!) of the git hash.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  14, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_PROTOCOL_VERSION_300()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 300));
};
INLINER void p310_vendor_specific_status_code_SET(uint16_t  src, Pack * dst)//Vendor-specific status information.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p310_uptime_sec_SET(uint32_t  src, Pack * dst)//The number of seconds since the start-up of the node.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p310_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER void p310_sub_mode_SET(uint8_t  src, Pack * dst)//Not used currently.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER void p310_health_SET(e_UAVCAN_NODE_HEALTH  src, Pack * dst)//Generalized node health status.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 120);
}
INLINER void p310_mode_SET(e_UAVCAN_NODE_MODE  src, Pack * dst)//Generalized operating mode.
{
    uint8_t * data = dst->data;
    UMAX id;
    switch(src)
    {
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL:
            id = 0;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION:
            id = 1;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE:
            id = 2;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE:
            id = 3;
            break;
        case e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE:
            id = 4;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 122);
}
Pack * c_TEST_Channel_new_UAVCAN_NODE_STATUS_310()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 310));
};
INLINER void p311_uptime_sec_SET(uint32_t  src, Pack * dst)//The number of seconds since the start-up of the node.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p311_sw_vcs_commit_SET(uint32_t  src, Pack * dst)//Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p311_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p311_hw_version_major_SET(uint8_t  src, Pack * dst)//Hardware major version number.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER void p311_hw_version_minor_SET(uint8_t  src, Pack * dst)//Hardware minor version number.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  17);
}
INLINER void p311_hw_unique_id_SET(uint8_t*  src, int32_t pos, Pack * dst) //Hardware unique 128-bit ID.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  18, src_max = pos + 16; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p311_sw_version_major_SET(uint8_t  src, Pack * dst)//Software major version number.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  34);
}
INLINER void p311_sw_version_minor_SET(uint8_t  src, Pack * dst)//Software minor version number.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER void p311_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Node name string. For example, "sapog.px4.io".
{
    if(dst->base.field_bit != 288 && insert_field(dst, 288, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p311_name_SET_(char16_t*  src, Bounds_Inside * dst) {p311_name_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_UAVCAN_NODE_INFO_311()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 311));
};
INLINER void p320_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p320_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p320_param_index_SET(int16_t  src, Pack * dst)//Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p320_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p320_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p320_param_id_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 320));
};
INLINER void p321_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p321_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
Pack * c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 321));
};
INLINER void p322_param_count_SET(uint16_t  src, Pack * dst)//Total number of parameters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p322_param_index_SET(uint16_t  src, Pack * dst)//Index of this parameter
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p322_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 32);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p322_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 38 && insert_field(dst, 38, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p322_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p322_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p322_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter value
{
    if(dst->base.field_bit != 39 && insert_field(dst, 39, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p322_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p322_param_value_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_PARAM_EXT_VALUE_322()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 322));
};
INLINER void p323_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p323_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p323_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 16);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p323_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 22 && insert_field(dst, 22, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p323_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p323_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p323_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter value
{
    if(dst->base.field_bit != 23 && insert_field(dst, 23, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p323_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p323_param_value_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_PARAM_EXT_SET_323()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 323));
};
INLINER void p324_param_type_SET(e_MAV_PARAM_EXT_TYPE  src, Pack * dst)//Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 4, data, 0);
}
INLINER void p324_param_result_SET(e_PARAM_ACK  src, Pack * dst)//Result code: see the PARAM_ACK enum for possible codes.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 4);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p324_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 8 && insert_field(dst, 8, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p324_param_id_SET_(char16_t*  src, Bounds_Inside * dst) {p324_param_id_SET(src, 0, strlen16(src), dst);}
INLINER void p324_param_value_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
{
    if(dst->base.field_bit != 9 && insert_field(dst, 9, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p324_param_value_SET_(char16_t*  src, Bounds_Inside * dst) {p324_param_value_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_PARAM_EXT_ACK_324()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 324));
};/**
*Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle
*	is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*	for unknown/not used. In a array element, each unit corresponds to 1cm*/
INLINER void p330_distances_SET(uint16_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 72; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p330_min_distance_SET(uint16_t  src, Pack * dst)//Minimum distance the sensor can measure in centimeters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  144);
}
INLINER void p330_max_distance_SET(uint16_t  src, Pack * dst)//Maximum distance the sensor can measure in centimeters
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  146);
}
INLINER void p330_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since system boot or since UNIX epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  148);
}
INLINER void p330_increment_SET(uint8_t  src, Pack * dst)//Angular width in degrees of each array element.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  156);
}
INLINER void p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR  src, Pack * dst)//Class id of the distance sensor type.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 1256);
}
Pack * c_TEST_Channel_new_OBSTACLE_DISTANCE_330()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 330));
};
INLINER void p10001_stallSpeed_SET(uint16_t  src, Pack * dst)//Aircraft stall speed in cm/s
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p10001_ICAO_SET(uint32_t  src, Pack * dst)//Vehicle address (24 bit)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p10001_emitterType_SET(e_ADSB_EMITTER_TYPE  src, Pack * dst)//Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 48);
}
INLINER void p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE  src, Pack * dst)//Aircraft length and width encoding (table 2-35 of DO-282B)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 53);
}
INLINER void p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT  src, Pack * dst)//GPS antenna lateral offset (table 2-36 of DO-282B)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 57);
}
/**
*GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add
*	one] (table 2-37 DO-282B*/
INLINER void p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 60);
}
INLINER void p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT  src, Pack * dst)//ADS-B transponder reciever and transmit enable flags
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 61);
}
INLINER void p10001_callsign_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
{
    if(dst->base.field_bit != 63 && insert_field(dst, 63, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p10001_callsign_SET_(char16_t*  src, Bounds_Inside * dst) {p10001_callsign_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_OUT_CFG_10001()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10001));
};
INLINER void p10002_accuracyVert_SET(uint16_t  src, Pack * dst)//Vertical accuracy in cm. If unknown set to UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p10002_accuracyVel_SET(uint16_t  src, Pack * dst)//Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p10002_squawk_SET(uint16_t  src, Pack * dst)//Mode A code (typically 1200 [0x04B0] for VFR)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p10002_utcTime_SET(uint32_t  src, Pack * dst)//UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p10002_accuracyHor_SET(uint32_t  src, Pack * dst)//Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER void p10002_gpsLat_SET(int32_t  src, Pack * dst)//Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p10002_gpsLon_SET(int32_t  src, Pack * dst)//Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER void p10002_gpsAlt_SET(int32_t  src, Pack * dst)//Altitude in mm (m * 1E-3) UP +ve. WGS84 altitude. If unknown set to INT32_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  22);
}
INLINER void p10002_numSats_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown set to UINT8_MAX
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  26);
}
/**
*Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude
*	(m * 1E-3). (up +ve). If unknown set to INT32_MA*/
INLINER void p10002_baroAltMSL_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  27);
}
INLINER void p10002_velVert_SET(int16_t  src, Pack * dst)//GPS vertical speed in cm/s. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  31);
}
INLINER void p10002_velNS_SET(int16_t  src, Pack * dst)//North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  33);
}
INLINER void p10002_VelEW_SET(int16_t  src, Pack * dst)//East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  35);
}
INLINER void p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX  src, Pack * dst)//0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 296);
}
INLINER void p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS  src, Pack * dst)//Emergency status
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 299);
}
INLINER void p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE  src, Pack * dst)//ADS-B transponder dynamic input state flags
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 5, data, 302);
}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10002));
};
INLINER void p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH  src, Pack * dst)//ADS-B transponder messages
{
    uint8_t * data = dst->data;
    UMAX id;
    switch(src)
    {
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_INITIALIZING:
            id = 0;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_OK:
            id = 1;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX:
            id = 2;
            break;
        case e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX:
            id = 3;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 0);
}
Pack * c_TEST_Channel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 10003));
};
INLINER void p11000_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied to reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11000_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11000_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11000_bus_SET(uint8_t  src, Pack * dst)//Bus number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11000_address_SET(uint8_t  src, Pack * dst)//Bus address
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p11000_regstart_SET(uint8_t  src, Pack * dst)//First register to read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p11000_count_SET(uint8_t  src, Pack * dst)//count of registers to read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p11000_bustype_SET(e_DEVICE_OP_BUSTYPE  src, Pack * dst)//The bus type
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 80);
}
INLINER void p11000_busname_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of device on bus (for SPI)
{
    if(dst->base.field_bit != 81 && insert_field(dst, 81, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p11000_busname_SET_(char16_t*  src, Bounds_Inside * dst) {p11000_busname_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_DEVICE_OP_READ_11000()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11000));
};
INLINER void p11001_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied from request
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11001_result_SET(uint8_t  src, Pack * dst)//0 for success, anything else is failure code
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11001_regstart_SET(uint8_t  src, Pack * dst)//starting register
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11001_count_SET(uint8_t  src, Pack * dst)//count of bytes read
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11001_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //reply data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  7, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_DEVICE_OP_READ_REPLY_11001()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11001));
};
INLINER void p11002_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied to reply
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11002_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p11002_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p11002_bus_SET(uint8_t  src, Pack * dst)//Bus number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p11002_address_SET(uint8_t  src, Pack * dst)//Bus address
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p11002_regstart_SET(uint8_t  src, Pack * dst)//First register to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p11002_count_SET(uint8_t  src, Pack * dst)//count of registers to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p11002_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //write data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 128; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p11002_bustype_SET(e_DEVICE_OP_BUSTYPE  src, Pack * dst)//The bus type
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 1104);
}
INLINER void p11002_busname_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of device on bus (for SPI)
{
    if(dst->base.field_bit != 1105 && insert_field(dst, 1105, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p11002_busname_SET_(char16_t*  src, Bounds_Inside * dst) {p11002_busname_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_DEVICE_OP_WRITE_11002()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11002));
};
INLINER void p11003_request_id_SET(uint32_t  src, Pack * dst)//request ID - copied from request
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p11003_result_SET(uint8_t  src, Pack * dst)//0 for success, anything else is failure code
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
Pack * c_TEST_Channel_new_DEVICE_OP_WRITE_REPLY_11003()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11003));
};
INLINER void p11010_desired_SET(float  src, Pack * dst)//desired rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p11010_achieved_SET(float  src, Pack * dst)//achieved rate (degrees/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p11010_error_SET(float  src, Pack * dst)//error between model and vehicle
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p11010_theta_SET(float  src, Pack * dst)//theta estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p11010_omega_SET(float  src, Pack * dst)//omega estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p11010_sigma_SET(float  src, Pack * dst)//sigma estimated state predictor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p11010_theta_dot_SET(float  src, Pack * dst)//theta derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p11010_omega_dot_SET(float  src, Pack * dst)//omega derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p11010_sigma_dot_SET(float  src, Pack * dst)//sigma derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p11010_f_SET(float  src, Pack * dst)//projection operator value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p11010_f_dot_SET(float  src, Pack * dst)//projection operator derivative
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p11010_u_SET(float  src, Pack * dst)//u adaptive controlled output command
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p11010_axis_SET(e_PID_TUNING_AXIS  src, Pack * dst)//axis
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 3, data, 384);
}
Pack * c_TEST_Channel_new_ADAP_TUNING_11010()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11010));
};
INLINER void p11011_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds, synced to UNIX time or since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p11011_time_delta_usec_SET(uint64_t  src, Pack * dst)//Time in microseconds since the last reported camera frame
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p11011_angle_delta_SET(float*  src, int32_t pos, Pack * dst) //Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientatio
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  16, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right,
*	2=down*/
INLINER void p11011_position_delta_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p11011_confidence_SET(float  src, Pack * dst)//normalised confidence value from 0 to 100
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
Pack * c_TEST_Channel_new_VISION_POSITION_DELTA_11011()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 11011));
};


void c_TEST_Channel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_BOOT);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_SURFACE_BOAT);
    assert(p0_custom_mode_GET(pack) == (uint32_t)3473256338L);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -57);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)50589);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)62457);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)54030);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)10954);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)23946);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)55104);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)6219);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)51245);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)34658);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)911959736L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)8701466209600242140L);
};


void c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_afz_GET(pack) == (float)1.8253228E38F);
    assert(p3_yaw_rate_GET(pack) == (float)1.6429501E38F);
    assert(p3_x_GET(pack) == (float) -1.3093658E38F);
    assert(p3_vy_GET(pack) == (float) -2.7590016E38F);
    assert(p3_yaw_GET(pack) == (float) -2.7133485E38F);
    assert(p3_vz_GET(pack) == (float)1.4269583E38F);
    assert(p3_afx_GET(pack) == (float)1.6213494E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2994866487L);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)4603);
    assert(p3_vx_GET(pack) == (float) -3.107327E38F);
    assert(p3_y_GET(pack) == (float)2.9511224E38F);
    assert(p3_z_GET(pack) == (float)3.318852E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p3_afy_GET(pack) == (float)2.6816858E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p4_seq_GET(pack) == (uint32_t)2567245695L);
    assert(p4_time_usec_GET(pack) == (uint64_t)5250370577373848287L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)81);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p5_passkey_LEN(ph) == 3);
    {
        char16_t * exemplary = u"duj";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)218);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 18);
    {
        char16_t * exemplary = u"emxyyjfyfTexehrvns";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)2696139591L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p20_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"hnvwlHlzjuwrypvz";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -13124);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)139);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"co";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_value_GET(pack) == (float) -1.899329E38F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)14066);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)14010);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p23_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"gokUtggxWrjdX";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p23_param_value_GET(pack) == (float) -9.055052E37F);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_v_acc_TRY(ph) == (uint32_t)2213722939L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p24_alt_GET(pack) == (int32_t)1615096522);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)11163707L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)3541);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)32166);
    assert(p24_lon_GET(pack) == (int32_t) -1905035772);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)34073);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)8784);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)167634487L);
    assert(p24_time_usec_GET(pack) == (uint64_t)4057538721221030602L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)351072165L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -278050018);
    assert(p24_lat_GET(pack) == (int32_t)2079354658);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)250, (uint8_t)170, (uint8_t)119, (uint8_t)113, (uint8_t)202, (uint8_t)212, (uint8_t)111, (uint8_t)236, (uint8_t)62, (uint8_t)114, (uint8_t)135, (uint8_t)40, (uint8_t)134, (uint8_t)203, (uint8_t)20, (uint8_t)194, (uint8_t)229, (uint8_t)53, (uint8_t)233, (uint8_t)225} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)109, (uint8_t)204, (uint8_t)220, (uint8_t)133, (uint8_t)39, (uint8_t)64, (uint8_t)157, (uint8_t)174, (uint8_t)218, (uint8_t)137, (uint8_t)100, (uint8_t)223, (uint8_t)184, (uint8_t)135, (uint8_t)11, (uint8_t)125, (uint8_t)112, (uint8_t)11, (uint8_t)191, (uint8_t)106} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)16, (uint8_t)71, (uint8_t)217, (uint8_t)160, (uint8_t)23, (uint8_t)1, (uint8_t)50, (uint8_t)254, (uint8_t)171, (uint8_t)160, (uint8_t)225, (uint8_t)54, (uint8_t)27, (uint8_t)160, (uint8_t)170, (uint8_t)119, (uint8_t)43, (uint8_t)254, (uint8_t)113} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)160);
    {
        uint8_t exemplary[] =  {(uint8_t)208, (uint8_t)163, (uint8_t)173, (uint8_t)255, (uint8_t)228, (uint8_t)206, (uint8_t)246, (uint8_t)252, (uint8_t)49, (uint8_t)230, (uint8_t)66, (uint8_t)117, (uint8_t)98, (uint8_t)136, (uint8_t)206, (uint8_t)118, (uint8_t)64, (uint8_t)153, (uint8_t)251, (uint8_t)135} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)149, (uint8_t)206, (uint8_t)17, (uint8_t)42, (uint8_t)44, (uint8_t)155, (uint8_t)201, (uint8_t)250, (uint8_t)175, (uint8_t)169, (uint8_t)88, (uint8_t)136, (uint8_t)79, (uint8_t)194, (uint8_t)101, (uint8_t)111, (uint8_t)94, (uint8_t)213, (uint8_t)129, (uint8_t)44} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -18089);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)31494);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -23211);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -12457);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -12810);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -6739);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -13887);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -17665);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)59307658L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)31814);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -13481);
    assert(p27_time_usec_GET(pack) == (uint64_t)6240130441531071882L);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)24800);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)9569);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)2967);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -205);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)9562);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -17407);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)31957);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)12613);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -2506);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -7783);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)6893);
    assert(p28_time_usec_GET(pack) == (uint64_t)3868045732865181282L);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)13051);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)417424326L);
    assert(p29_press_diff_GET(pack) == (float) -2.9520368E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -31565);
    assert(p29_press_abs_GET(pack) == (float)2.7463825E38F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float) -5.365904E37F);
    assert(p30_yaw_GET(pack) == (float) -2.1782726E38F);
    assert(p30_pitch_GET(pack) == (float) -1.9091804E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2063342657L);
    assert(p30_yawspeed_GET(pack) == (float) -2.4381418E38F);
    assert(p30_roll_GET(pack) == (float)3.4903154E37F);
    assert(p30_pitchspeed_GET(pack) == (float)1.3075602E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_pitchspeed_GET(pack) == (float) -1.8698631E38F);
    assert(p31_yawspeed_GET(pack) == (float)2.3263054E38F);
    assert(p31_q2_GET(pack) == (float)2.9257911E38F);
    assert(p31_q3_GET(pack) == (float) -3.4717445E36F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)4000088348L);
    assert(p31_q1_GET(pack) == (float) -1.793905E38F);
    assert(p31_rollspeed_GET(pack) == (float) -3.0867312E38F);
    assert(p31_q4_GET(pack) == (float) -3.2202921E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vy_GET(pack) == (float) -3.7692898E37F);
    assert(p32_z_GET(pack) == (float) -1.043197E38F);
    assert(p32_vz_GET(pack) == (float)1.3874381E38F);
    assert(p32_vx_GET(pack) == (float)2.5306764E38F);
    assert(p32_x_GET(pack) == (float)3.2834475E38F);
    assert(p32_y_GET(pack) == (float)4.3581133E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)630259066L);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lon_GET(pack) == (int32_t) -2143268002);
    assert(p33_lat_GET(pack) == (int32_t)1108799583);
    assert(p33_alt_GET(pack) == (int32_t) -1506923446);
    assert(p33_relative_alt_GET(pack) == (int32_t)147942293);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -5755);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)29790);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)24605);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)3129597245L);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -12754);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -16419);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)3723);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)28366);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)10972);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -2528);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -2942);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -24832);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1218782226L);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -1597);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)31241);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)29861);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3893730859L);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)7601);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)34758);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)54373);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)58617);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)22447);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)1287);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)50613);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)23140);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)27220);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)38211);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)25416);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)15185);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)30567);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)42835);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)5384);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)55770);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)28926);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)26964);
    assert(p36_time_usec_GET(pack) == (uint32_t)3174655675L);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)60806);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)62437);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)53564);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)20947);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)15441);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -5751);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)11237);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)20064);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_y_GET(pack) == (float) -2.4461456E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)40201);
    assert(p39_z_GET(pack) == (float)9.904857E37F);
    assert(p39_param4_GET(pack) == (float)7.479992E37F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p39_param3_GET(pack) == (float) -1.674966E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p39_param2_GET(pack) == (float)8.695268E37F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p39_x_GET(pack) == (float) -2.1980869E38F);
    assert(p39_param1_GET(pack) == (float)9.228544E37F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)57348);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)41377);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)8153);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)39064);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)38885);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p48_altitude_GET(pack) == (int32_t) -1390960782);
    assert(p48_longitude_GET(pack) == (int32_t)543957699);
    assert(p48_time_usec_TRY(ph) == (uint64_t)6529670338612087669L);
    assert(p48_latitude_GET(pack) == (int32_t) -1055381470);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)4890014982723322770L);
    assert(p49_latitude_GET(pack) == (int32_t) -700907913);
    assert(p49_longitude_GET(pack) == (int32_t) -1259449083);
    assert(p49_altitude_GET(pack) == (int32_t) -1695486520);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)15940);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p50_scale_GET(pack) == (float) -2.150964E37F);
    assert(p50_param_value_min_GET(pack) == (float)1.53327E37F);
    assert(p50_param_value_max_GET(pack) == (float) -7.5350323E37F);
    assert(p50_param_value0_GET(pack) == (float)1.2636306E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"x";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)29610);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1z_GET(pack) == (float) -1.7053513E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p54_p1x_GET(pack) == (float)8.087403E37F);
    assert(p54_p2z_GET(pack) == (float)1.159999E37F);
    assert(p54_p2x_GET(pack) == (float) -1.5536912E38F);
    assert(p54_p1y_GET(pack) == (float)1.5000523E38F);
    assert(p54_p2y_GET(pack) == (float)1.8828015E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)120);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float) -7.2593945E36F);
    assert(p55_p1y_GET(pack) == (float)1.0212652E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p55_p1z_GET(pack) == (float)3.0376926E38F);
    assert(p55_p1x_GET(pack) == (float)1.3301561E38F);
    assert(p55_p2x_GET(pack) == (float) -1.6229152E38F);
    assert(p55_p2y_GET(pack) == (float)2.94403E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_rollspeed_GET(pack) == (float) -1.5636592E38F);
    {
        float exemplary[] =  {1.8319259E38F, 6.2625705E36F, 3.1676512E37F, -3.2698613E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)6868532390058177135L);
    {
        float exemplary[] =  {-2.6542882E38F, 8.768874E37F, 1.7195139E37F, -5.725912E37F, -2.1791634E38F, -2.1836872E38F, -5.1119674E37F, -1.1706831E38F, 3.5431813E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float)4.5123123E37F);
    assert(p61_yawspeed_GET(pack) == (float) -3.3393105E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float)3.2785442E38F);
    assert(p62_aspd_error_GET(pack) == (float)2.3301941E38F);
    assert(p62_alt_error_GET(pack) == (float)2.7564536E36F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -20447);
    assert(p62_nav_pitch_GET(pack) == (float) -4.8372903E37F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)13535);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)12074);
    assert(p62_xtrack_error_GET(pack) == (float)3.245488E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.042942E38F, -3.3770569E38F, 2.4165187E38F, -3.3222916E38F, 2.0426066E38F, -2.540912E38F, -1.0193723E38F, -3.2027032E38F, -2.7461366E38F, 1.7111667E38F, 1.8360053E38F, -1.7130199E38F, 3.1888832E38F, 2.4502913E38F, 2.1693652E38F, -2.2462255E38F, -1.5726621E38F, 2.3421035E38F, -3.0749274E37F, 7.05773E37F, -3.0093378E38F, -3.1132497E37F, 4.7149955E37F, -3.5916788E37F, -2.4585538E38F, -5.9794475E37F, 2.2707752E38F, 2.8507523E38F, 2.3514846E38F, 2.41959E38F, 4.1102683E37F, -3.3948953E38F, -5.294936E37F, -9.912239E37F, 7.637759E37F, -3.3491613E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vx_GET(pack) == (float)7.2206266E37F);
    assert(p63_lon_GET(pack) == (int32_t)1781977607);
    assert(p63_time_usec_GET(pack) == (uint64_t)3914575210239425035L);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_alt_GET(pack) == (int32_t)1332501695);
    assert(p63_vy_GET(pack) == (float)7.087076E37F);
    assert(p63_relative_alt_GET(pack) == (int32_t)1939944674);
    assert(p63_lat_GET(pack) == (int32_t) -1905357514);
    assert(p63_vz_GET(pack) == (float) -1.9182411E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p64_z_GET(pack) == (float)1.2428261E38F);
    assert(p64_x_GET(pack) == (float)8.714836E37F);
    assert(p64_ax_GET(pack) == (float) -2.8027907E38F);
    assert(p64_ay_GET(pack) == (float) -3.3245656E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)6840861446907552184L);
    assert(p64_az_GET(pack) == (float) -2.7447445E38F);
    assert(p64_vz_GET(pack) == (float) -3.4533913E37F);
    assert(p64_y_GET(pack) == (float) -1.0210766E38F);
    {
        float exemplary[] =  {-2.0267678E38F, -2.3802128E38F, 3.2095587E38F, 4.3625673E37F, 1.5825956E38F, -1.1901752E38F, 9.88296E37F, 3.2841426E38F, 2.9126341E38F, 7.047598E37F, -3.0045E38F, -1.0342931E38F, 1.4822627E38F, -5.1415944E37F, 5.489218E37F, 1.4470436E37F, -4.9143655E37F, -3.3971472E37F, 1.6171797E38F, 2.514227E38F, 2.968204E38F, -2.5292633E38F, -3.0479397E38F, -8.280108E37F, -1.5879017E38F, -1.345013E38F, 3.9642332E37F, -1.2919841E38F, 8.873542E37F, -4.4770748E36F, -2.8778032E38F, -1.2368249E38F, -1.3501561E37F, 7.1637694E37F, -2.126477E38F, 2.2818585E38F, -3.2283083E38F, 2.215962E38F, 1.8835815E38F, -2.0684796E38F, -6.7636877E37F, -2.3651245E38F, 1.2578159E38F, 1.0837243E38F, -2.5993979E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vy_GET(pack) == (float)4.748064E37F);
    assert(p64_vx_GET(pack) == (float) -9.838845E37F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)2544);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)43606);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)21005);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)2743);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)27436);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)63369);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)10363);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)61555);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)27705);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)34108);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)11404);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)1358418158L);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)55156);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)47448);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)13946);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)44773);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)1777);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)58471);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)39009);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)60430);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)51);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)17412);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t)24978);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -28557);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)14166);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)52292);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -17393);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)38391);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)3561);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)17084);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)30826);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)26327);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)21745);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)57080);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)64019);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_z_GET(pack) == (float)1.4716172E37F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_param1_GET(pack) == (float) -5.015129E37F);
    assert(p73_x_GET(pack) == (int32_t)1833981712);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p73_y_GET(pack) == (int32_t) -1825103608);
    assert(p73_param4_GET(pack) == (float) -2.5691168E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)51436);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p73_param2_GET(pack) == (float)2.8786773E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_USER_4);
    assert(p73_param3_GET(pack) == (float)1.6314174E38F);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_climb_GET(pack) == (float) -2.1007745E38F);
    assert(p74_groundspeed_GET(pack) == (float) -3.6161726E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)1466);
    assert(p74_airspeed_GET(pack) == (float) -2.3945303E38F);
    assert(p74_alt_GET(pack) == (float)5.055715E37F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)35668);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p75_param1_GET(pack) == (float) -3.399694E38F);
    assert(p75_param2_GET(pack) == (float)8.548283E37F);
    assert(p75_param4_GET(pack) == (float) -5.0740276E37F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p75_x_GET(pack) == (int32_t) -1066873593);
    assert(p75_y_GET(pack) == (int32_t) -1165759413);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPOSITION);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p75_z_GET(pack) == (float)2.3041381E38F);
    assert(p75_param3_GET(pack) == (float)1.3842496E38F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param3_GET(pack) == (float) -2.9528657E38F);
    assert(p76_param7_GET(pack) == (float)1.3754551E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p76_param2_GET(pack) == (float)9.984129E37F);
    assert(p76_param5_GET(pack) == (float)1.7800285E38F);
    assert(p76_param1_GET(pack) == (float) -1.7039703E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p76_param4_GET(pack) == (float)1.2580275E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
    assert(p76_param6_GET(pack) == (float) -2.2581682E38F);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
    assert(p77_result_param2_TRY(ph) == (int32_t)1919772252);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)63);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)86);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)197);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_STORAGE_FORMAT);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p81_pitch_GET(pack) == (float) -1.9361337E38F);
    assert(p81_thrust_GET(pack) == (float)1.6786203E38F);
    assert(p81_yaw_GET(pack) == (float) -1.1193933E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3121835409L);
    assert(p81_roll_GET(pack) == (float) -3.240329E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_yaw_rate_GET(pack) == (float)1.4478203E37F);
    assert(p82_thrust_GET(pack) == (float)6.45079E37F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.776839E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)2.9441723E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)124);
    {
        float exemplary[] =  {-1.2861756E38F, -1.0440092E38F, -2.081663E38F, -3.183554E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)447063262L);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float)3.2967063E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3535735512L);
    assert(p83_thrust_GET(pack) == (float) -3.166687E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)4.7352667E37F);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.5878473E37F);
    {
        float exemplary[] =  {1.016169E38F, 1.2416461E38F, -1.8213924E38F, -1.5204962E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_x_GET(pack) == (float)2.281713E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p84_vz_GET(pack) == (float) -1.3270502E38F);
    assert(p84_y_GET(pack) == (float) -2.5538324E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)7433);
    assert(p84_yaw_GET(pack) == (float) -2.5397098E38F);
    assert(p84_afz_GET(pack) == (float)3.3071728E38F);
    assert(p84_z_GET(pack) == (float)2.4646164E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p84_vx_GET(pack) == (float)3.3870748E38F);
    assert(p84_afx_GET(pack) == (float)2.0011033E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -1.3329793E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3080430978L);
    assert(p84_vy_GET(pack) == (float)2.5219446E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p84_afy_GET(pack) == (float)4.5636907E37F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vy_GET(pack) == (float)2.7367303E38F);
    assert(p86_lat_int_GET(pack) == (int32_t) -506630299);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)6621);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p86_alt_GET(pack) == (float)2.7520544E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)963049746);
    assert(p86_afz_GET(pack) == (float)3.0521136E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -4.5022705E37F);
    assert(p86_yaw_GET(pack) == (float) -9.825652E36F);
    assert(p86_afy_GET(pack) == (float) -2.6273503E38F);
    assert(p86_vx_GET(pack) == (float) -8.947105E37F);
    assert(p86_vz_GET(pack) == (float)2.3918451E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p86_afx_GET(pack) == (float) -2.8067245E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)434876613L);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)175);
};


void c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afz_GET(pack) == (float)1.3004249E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)498);
    assert(p87_afy_GET(pack) == (float)3.3855057E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -2.8086967E38F);
    assert(p87_lat_int_GET(pack) == (int32_t)1530578360);
    assert(p87_alt_GET(pack) == (float)5.9577296E37F);
    assert(p87_vx_GET(pack) == (float) -1.3773908E38F);
    assert(p87_vy_GET(pack) == (float) -2.1146448E38F);
    assert(p87_yaw_GET(pack) == (float)2.4187005E38F);
    assert(p87_vz_GET(pack) == (float) -1.2772831E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2929938786L);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p87_lon_int_GET(pack) == (int32_t) -469102545);
    assert(p87_afx_GET(pack) == (float)1.7120013E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_pitch_GET(pack) == (float) -6.7218086E37F);
    assert(p89_z_GET(pack) == (float)2.5593036E38F);
    assert(p89_yaw_GET(pack) == (float) -5.588522E36F);
    assert(p89_x_GET(pack) == (float)1.6617693E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)517236725L);
    assert(p89_y_GET(pack) == (float)2.4958393E38F);
    assert(p89_roll_GET(pack) == (float) -1.8976786E38F);
};


void c_TEST_Channel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)17978);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -9783);
    assert(p90_pitch_GET(pack) == (float) -3.2463781E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)16326);
    assert(p90_alt_GET(pack) == (int32_t)138637301);
    assert(p90_lon_GET(pack) == (int32_t)1289865568);
    assert(p90_lat_GET(pack) == (int32_t)1778208770);
    assert(p90_yawspeed_GET(pack) == (float)2.012785E38F);
    assert(p90_rollspeed_GET(pack) == (float) -1.0990084E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)10854);
    assert(p90_pitchspeed_GET(pack) == (float)1.8417E37F);
    assert(p90_time_usec_GET(pack) == (uint64_t)1308806021531935658L);
    assert(p90_yaw_GET(pack) == (float) -2.3987036E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -24503);
    assert(p90_roll_GET(pack) == (float) -3.1415244E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)787);
};


void c_TEST_Channel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p91_aux4_GET(pack) == (float) -1.7767685E38F);
    assert(p91_aux2_GET(pack) == (float)2.7127415E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)1.163045E38F);
    assert(p91_aux1_GET(pack) == (float) -1.7561705E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)2.9800314E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)1502598631686122471L);
    assert(p91_throttle_GET(pack) == (float) -9.636375E37F);
    assert(p91_aux3_GET(pack) == (float) -3.0406445E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)3.3754272E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
};


void c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)7001);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)36827);
    assert(p92_time_usec_GET(pack) == (uint64_t)27269981483190511L);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)65306);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)31166);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)34393);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)14977);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)11724);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)59902);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)12291);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)35803);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)2533);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)57784);
};


void c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)2427409631316157394L);
    {
        float exemplary[] =  {1.296122E38F, 2.4771765E38F, -1.6323735E38F, -2.5396027E38F, -5.649296E37F, 1.0329847E38F, 2.0473287E38F, -2.6651287E37F, -3.2771508E38F, 1.3150501E38F, -2.6064369E38F, -2.2323063E37F, -9.631881E37F, 3.3706468E38F, -1.3839112E38F, -2.1172913E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)5732493282806625187L);
};


void c_TEST_Channel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_comp_m_y_GET(pack) == (float) -1.0340761E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -5.582826E37F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)5.702404E37F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -8227);
    assert(p100_ground_distance_GET(pack) == (float) -2.5283796E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)20204);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p100_flow_rate_y_TRY(ph) == (float)2.887878E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)6454181654021257243L);
};


void c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_yaw_GET(pack) == (float)1.4119027E38F);
    assert(p101_z_GET(pack) == (float) -2.6450434E38F);
    assert(p101_roll_GET(pack) == (float) -1.7423957E38F);
    assert(p101_pitch_GET(pack) == (float) -2.974933E38F);
    assert(p101_x_GET(pack) == (float)2.452712E38F);
    assert(p101_usec_GET(pack) == (uint64_t)5016536245288174239L);
    assert(p101_y_GET(pack) == (float)8.558883E37F);
};


void c_TEST_Channel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_pitch_GET(pack) == (float) -2.7903943E38F);
    assert(p102_roll_GET(pack) == (float)5.517476E37F);
    assert(p102_usec_GET(pack) == (uint64_t)8809574321192565503L);
    assert(p102_yaw_GET(pack) == (float) -8.1296416E37F);
    assert(p102_x_GET(pack) == (float) -1.7929413E38F);
    assert(p102_y_GET(pack) == (float) -2.4305862E38F);
    assert(p102_z_GET(pack) == (float)1.1526458E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float) -2.1738731E38F);
    assert(p103_usec_GET(pack) == (uint64_t)6384236830692217058L);
    assert(p103_y_GET(pack) == (float)3.083999E38F);
    assert(p103_x_GET(pack) == (float) -3.865643E37F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float) -8.4217686E37F);
    assert(p104_y_GET(pack) == (float)2.9631375E38F);
    assert(p104_z_GET(pack) == (float) -3.2674218E38F);
    assert(p104_yaw_GET(pack) == (float) -1.0449101E38F);
    assert(p104_pitch_GET(pack) == (float)2.819721E37F);
    assert(p104_roll_GET(pack) == (float) -1.1464653E38F);
    assert(p104_usec_GET(pack) == (uint64_t)7930840519941980549L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_abs_pressure_GET(pack) == (float) -1.0328859E38F);
    assert(p105_xgyro_GET(pack) == (float)3.0346586E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8709509946926204667L);
    assert(p105_zmag_GET(pack) == (float) -2.0786385E38F);
    assert(p105_zgyro_GET(pack) == (float) -5.4022067E35F);
    assert(p105_xacc_GET(pack) == (float) -2.845407E38F);
    assert(p105_yacc_GET(pack) == (float) -2.2729306E38F);
    assert(p105_ygyro_GET(pack) == (float) -8.1727214E37F);
    assert(p105_pressure_alt_GET(pack) == (float) -3.1362463E38F);
    assert(p105_ymag_GET(pack) == (float) -4.0989366E37F);
    assert(p105_temperature_GET(pack) == (float)3.18487E38F);
    assert(p105_zacc_GET(pack) == (float) -2.1361917E38F);
    assert(p105_diff_pressure_GET(pack) == (float) -3.0594374E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)28749);
    assert(p105_xmag_GET(pack) == (float) -9.415653E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p106_time_usec_GET(pack) == (uint64_t)4863102767190122976L);
    assert(p106_integrated_ygyro_GET(pack) == (float)2.554222E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p106_distance_GET(pack) == (float) -2.2105303E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float)2.2989442E38F);
    assert(p106_integrated_x_GET(pack) == (float) -2.9263296E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3391285960L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3877446271L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)20114);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.1844881E37F);
    assert(p106_integrated_y_GET(pack) == (float) -3.10678E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_pressure_alt_GET(pack) == (float)1.5149414E38F);
    assert(p107_diff_pressure_GET(pack) == (float)2.0428548E38F);
    assert(p107_abs_pressure_GET(pack) == (float)2.6731007E37F);
    assert(p107_xmag_GET(pack) == (float) -2.790117E38F);
    assert(p107_ymag_GET(pack) == (float) -2.2505765E38F);
    assert(p107_xgyro_GET(pack) == (float)6.737639E37F);
    assert(p107_zacc_GET(pack) == (float)8.915135E37F);
    assert(p107_temperature_GET(pack) == (float)2.8750119E38F);
    assert(p107_xacc_GET(pack) == (float) -2.8537178E38F);
    assert(p107_yacc_GET(pack) == (float) -2.3809523E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)4846370867762031452L);
    assert(p107_zmag_GET(pack) == (float)3.0965272E38F);
    assert(p107_ygyro_GET(pack) == (float)1.1860863E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1702696425L);
    assert(p107_zgyro_GET(pack) == (float)1.4572054E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_lon_GET(pack) == (float) -1.0745593E38F);
    assert(p108_xacc_GET(pack) == (float) -2.2982923E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.7448996E38F);
    assert(p108_ve_GET(pack) == (float) -3.1755037E38F);
    assert(p108_alt_GET(pack) == (float) -1.3344527E37F);
    assert(p108_zacc_GET(pack) == (float) -6.396129E37F);
    assert(p108_zgyro_GET(pack) == (float)9.722182E37F);
    assert(p108_pitch_GET(pack) == (float)9.372106E37F);
    assert(p108_q3_GET(pack) == (float) -7.1383905E37F);
    assert(p108_vn_GET(pack) == (float) -3.4288547E37F);
    assert(p108_q4_GET(pack) == (float)1.5308777E38F);
    assert(p108_q2_GET(pack) == (float) -1.4183924E38F);
    assert(p108_roll_GET(pack) == (float)5.346685E36F);
    assert(p108_yaw_GET(pack) == (float) -2.1164978E38F);
    assert(p108_lat_GET(pack) == (float)2.0770287E38F);
    assert(p108_xgyro_GET(pack) == (float)1.329684E38F);
    assert(p108_ygyro_GET(pack) == (float) -1.0662154E38F);
    assert(p108_yacc_GET(pack) == (float) -2.2321696E38F);
    assert(p108_q1_GET(pack) == (float) -7.9086926E37F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.6691852E38F);
    assert(p108_vd_GET(pack) == (float) -2.5532493E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)1903);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)40437);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)113);
    {
        uint8_t exemplary[] =  {(uint8_t)14, (uint8_t)141, (uint8_t)177, (uint8_t)106, (uint8_t)135, (uint8_t)45, (uint8_t)156, (uint8_t)131, (uint8_t)111, (uint8_t)91, (uint8_t)25, (uint8_t)142, (uint8_t)67, (uint8_t)85, (uint8_t)67, (uint8_t)87, (uint8_t)152, (uint8_t)196, (uint8_t)177, (uint8_t)33, (uint8_t)38, (uint8_t)70, (uint8_t)102, (uint8_t)29, (uint8_t)110, (uint8_t)178, (uint8_t)16, (uint8_t)142, (uint8_t)25, (uint8_t)182, (uint8_t)102, (uint8_t)4, (uint8_t)248, (uint8_t)57, (uint8_t)233, (uint8_t)251, (uint8_t)73, (uint8_t)177, (uint8_t)144, (uint8_t)64, (uint8_t)200, (uint8_t)94, (uint8_t)234, (uint8_t)174, (uint8_t)38, (uint8_t)53, (uint8_t)71, (uint8_t)244, (uint8_t)57, (uint8_t)38, (uint8_t)3, (uint8_t)136, (uint8_t)77, (uint8_t)239, (uint8_t)56, (uint8_t)16, (uint8_t)102, (uint8_t)59, (uint8_t)26, (uint8_t)251, (uint8_t)66, (uint8_t)156, (uint8_t)186, (uint8_t)28, (uint8_t)6, (uint8_t)187, (uint8_t)81, (uint8_t)225, (uint8_t)146, (uint8_t)248, (uint8_t)177, (uint8_t)117, (uint8_t)59, (uint8_t)198, (uint8_t)35, (uint8_t)136, (uint8_t)180, (uint8_t)224, (uint8_t)198, (uint8_t)188, (uint8_t)185, (uint8_t)68, (uint8_t)124, (uint8_t)37, (uint8_t)250, (uint8_t)217, (uint8_t)127, (uint8_t)84, (uint8_t)64, (uint8_t)237, (uint8_t)10, (uint8_t)50, (uint8_t)233, (uint8_t)178, (uint8_t)82, (uint8_t)168, (uint8_t)101, (uint8_t)103, (uint8_t)174, (uint8_t)222, (uint8_t)83, (uint8_t)181, (uint8_t)135, (uint8_t)118, (uint8_t)200, (uint8_t)89, (uint8_t)47, (uint8_t)161, (uint8_t)15, (uint8_t)172, (uint8_t)156, (uint8_t)154, (uint8_t)2, (uint8_t)100, (uint8_t)85, (uint8_t)47, (uint8_t)116, (uint8_t)94, (uint8_t)149, (uint8_t)117, (uint8_t)156, (uint8_t)151, (uint8_t)45, (uint8_t)148, (uint8_t)198, (uint8_t)177, (uint8_t)20, (uint8_t)189, (uint8_t)14, (uint8_t)172, (uint8_t)132, (uint8_t)245, (uint8_t)162, (uint8_t)247, (uint8_t)39, (uint8_t)167, (uint8_t)91, (uint8_t)96, (uint8_t)9, (uint8_t)211, (uint8_t)17, (uint8_t)47, (uint8_t)247, (uint8_t)52, (uint8_t)103, (uint8_t)226, (uint8_t)53, (uint8_t)191, (uint8_t)27, (uint8_t)183, (uint8_t)253, (uint8_t)154, (uint8_t)117, (uint8_t)56, (uint8_t)27, (uint8_t)87, (uint8_t)153, (uint8_t)208, (uint8_t)15, (uint8_t)189, (uint8_t)111, (uint8_t)73, (uint8_t)246, (uint8_t)101, (uint8_t)4, (uint8_t)106, (uint8_t)241, (uint8_t)159, (uint8_t)165, (uint8_t)133, (uint8_t)95, (uint8_t)145, (uint8_t)84, (uint8_t)108, (uint8_t)3, (uint8_t)70, (uint8_t)198, (uint8_t)217, (uint8_t)108, (uint8_t)112, (uint8_t)148, (uint8_t)0, (uint8_t)221, (uint8_t)56, (uint8_t)159, (uint8_t)82, (uint8_t)148, (uint8_t)162, (uint8_t)180, (uint8_t)135, (uint8_t)3, (uint8_t)115, (uint8_t)119, (uint8_t)230, (uint8_t)184, (uint8_t)235, (uint8_t)61, (uint8_t)238, (uint8_t)190, (uint8_t)54, (uint8_t)95, (uint8_t)148, (uint8_t)63, (uint8_t)98, (uint8_t)232, (uint8_t)102, (uint8_t)178, (uint8_t)57, (uint8_t)41, (uint8_t)195, (uint8_t)197, (uint8_t)214, (uint8_t)73, (uint8_t)15, (uint8_t)20, (uint8_t)19, (uint8_t)63, (uint8_t)220, (uint8_t)206, (uint8_t)7, (uint8_t)174, (uint8_t)14, (uint8_t)241, (uint8_t)132, (uint8_t)192, (uint8_t)203, (uint8_t)139, (uint8_t)157, (uint8_t)238, (uint8_t)169, (uint8_t)225, (uint8_t)167, (uint8_t)102, (uint8_t)142, (uint8_t)76, (uint8_t)149, (uint8_t)175, (uint8_t)136, (uint8_t)231, (uint8_t)223, (uint8_t)155, (uint8_t)91, (uint8_t)168, (uint8_t)9, (uint8_t)69, (uint8_t)37, (uint8_t)86, (uint8_t)173, (uint8_t)117, (uint8_t)129, (uint8_t)170} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -6303449974993382477L);
    assert(p111_tc1_GET(pack) == (int64_t)342863465457903769L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)1564453917102649671L);
    assert(p112_seq_GET(pack) == (uint32_t)223074054L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)55261);
    assert(p113_alt_GET(pack) == (int32_t) -457058146);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)51727);
    assert(p113_time_usec_GET(pack) == (uint64_t)7360602235232762631L);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)16296);
    assert(p113_lat_GET(pack) == (int32_t)1691933438);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)41159);
    assert(p113_lon_GET(pack) == (int32_t)884344387);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -31513);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -13602);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -25221);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.1219105E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)234345247L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1182563282L);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.6808253E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.8738557E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -17913);
    assert(p114_distance_GET(pack) == (float)2.0696244E38F);
    assert(p114_integrated_y_GET(pack) == (float)1.805269E37F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p114_time_usec_GET(pack) == (uint64_t)7942756989323883226L);
    assert(p114_integrated_x_GET(pack) == (float)2.3649883E37F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -18282);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)42170);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)25066);
    assert(p115_alt_GET(pack) == (int32_t)1970435894);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)27660);
    assert(p115_time_usec_GET(pack) == (uint64_t)8388180451266151954L);
    {
        float exemplary[] =  {-2.0995912E38F, 7.1376233E37F, 2.4338732E37F, 6.5911437E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)4411);
    assert(p115_rollspeed_GET(pack) == (float) -3.2549426E38F);
    assert(p115_yawspeed_GET(pack) == (float) -2.2386933E38F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)30684);
    assert(p115_lon_GET(pack) == (int32_t)2098267362);
    assert(p115_pitchspeed_GET(pack) == (float) -1.5326824E38F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -31835);
    assert(p115_lat_GET(pack) == (int32_t) -1312834695);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)18536);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)12267);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)17877);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)9328);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)19283);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -4468);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)2839);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)22040);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2072618281L);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)32575);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)13211);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)3219);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)41931);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)256833865L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)47429);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)20264);
    assert(p118_time_utc_GET(pack) == (uint32_t)298310947L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)15114);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)25426);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p119_ofs_GET(pack) == (uint32_t)569797457L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p119_count_GET(pack) == (uint32_t)2191085204L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)1434441270L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)19106);
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)103, (uint8_t)23, (uint8_t)27, (uint8_t)179, (uint8_t)112, (uint8_t)214, (uint8_t)217, (uint8_t)239, (uint8_t)174, (uint8_t)213, (uint8_t)243, (uint8_t)196, (uint8_t)32, (uint8_t)3, (uint8_t)142, (uint8_t)23, (uint8_t)12, (uint8_t)0, (uint8_t)238, (uint8_t)15, (uint8_t)108, (uint8_t)244, (uint8_t)60, (uint8_t)20, (uint8_t)230, (uint8_t)107, (uint8_t)157, (uint8_t)219, (uint8_t)86, (uint8_t)224, (uint8_t)111, (uint8_t)174, (uint8_t)67, (uint8_t)248, (uint8_t)230, (uint8_t)175, (uint8_t)236, (uint8_t)107, (uint8_t)133, (uint8_t)143, (uint8_t)164, (uint8_t)97, (uint8_t)82, (uint8_t)143, (uint8_t)184, (uint8_t)66, (uint8_t)71, (uint8_t)146, (uint8_t)149, (uint8_t)56, (uint8_t)131, (uint8_t)212, (uint8_t)38, (uint8_t)216, (uint8_t)58, (uint8_t)97, (uint8_t)84, (uint8_t)222, (uint8_t)184, (uint8_t)31, (uint8_t)113, (uint8_t)125, (uint8_t)228, (uint8_t)96, (uint8_t)95, (uint8_t)19, (uint8_t)110, (uint8_t)186, (uint8_t)44, (uint8_t)14, (uint8_t)30, (uint8_t)198, (uint8_t)6, (uint8_t)49, (uint8_t)132, (uint8_t)213, (uint8_t)113, (uint8_t)229, (uint8_t)229, (uint8_t)29, (uint8_t)170, (uint8_t)62, (uint8_t)165, (uint8_t)83, (uint8_t)168, (uint8_t)85, (uint8_t)230, (uint8_t)23, (uint8_t)37} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)72);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)20);
    {
        uint8_t exemplary[] =  {(uint8_t)81, (uint8_t)98, (uint8_t)85, (uint8_t)74, (uint8_t)164, (uint8_t)116, (uint8_t)235, (uint8_t)225, (uint8_t)194, (uint8_t)52, (uint8_t)56, (uint8_t)78, (uint8_t)99, (uint8_t)243, (uint8_t)78, (uint8_t)161, (uint8_t)253, (uint8_t)130, (uint8_t)105, (uint8_t)246, (uint8_t)134, (uint8_t)28, (uint8_t)44, (uint8_t)244, (uint8_t)175, (uint8_t)238, (uint8_t)21, (uint8_t)31, (uint8_t)215, (uint8_t)135, (uint8_t)213, (uint8_t)214, (uint8_t)215, (uint8_t)103, (uint8_t)233, (uint8_t)114, (uint8_t)30, (uint8_t)50, (uint8_t)158, (uint8_t)17, (uint8_t)2, (uint8_t)217, (uint8_t)136, (uint8_t)43, (uint8_t)147, (uint8_t)215, (uint8_t)248, (uint8_t)233, (uint8_t)74, (uint8_t)12, (uint8_t)190, (uint8_t)44, (uint8_t)65, (uint8_t)10, (uint8_t)100, (uint8_t)245, (uint8_t)88, (uint8_t)168, (uint8_t)50, (uint8_t)196, (uint8_t)78, (uint8_t)89, (uint8_t)232, (uint8_t)228, (uint8_t)135, (uint8_t)207, (uint8_t)26, (uint8_t)81, (uint8_t)142, (uint8_t)202, (uint8_t)59, (uint8_t)110, (uint8_t)183, (uint8_t)32, (uint8_t)122, (uint8_t)225, (uint8_t)143, (uint8_t)133, (uint8_t)109, (uint8_t)233, (uint8_t)61, (uint8_t)10, (uint8_t)158, (uint8_t)170, (uint8_t)161, (uint8_t)61, (uint8_t)152, (uint8_t)193, (uint8_t)113, (uint8_t)181, (uint8_t)88, (uint8_t)246, (uint8_t)36, (uint8_t)208, (uint8_t)54, (uint8_t)68, (uint8_t)92, (uint8_t)105, (uint8_t)165, (uint8_t)222, (uint8_t)162, (uint8_t)178, (uint8_t)40, (uint8_t)16, (uint8_t)179, (uint8_t)115, (uint8_t)64, (uint8_t)175, (uint8_t)140, (uint8_t)77} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)11717);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p124_lon_GET(pack) == (int32_t)1490068487);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)13147);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_lat_GET(pack) == (int32_t)347602116);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)15633);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p124_time_usec_GET(pack) == (uint64_t)552671631494735314L);
    assert(p124_dgps_age_GET(pack) == (uint32_t)562287537L);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)12483);
    assert(p124_alt_GET(pack) == (int32_t)518241545);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)24071);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)59550);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY));
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)31708);
    {
        uint8_t exemplary[] =  {(uint8_t)146, (uint8_t)177, (uint8_t)195, (uint8_t)94, (uint8_t)106, (uint8_t)176, (uint8_t)255, (uint8_t)31, (uint8_t)19, (uint8_t)55, (uint8_t)68, (uint8_t)183, (uint8_t)8, (uint8_t)98, (uint8_t)75, (uint8_t)18, (uint8_t)19, (uint8_t)107, (uint8_t)155, (uint8_t)69, (uint8_t)176, (uint8_t)56, (uint8_t)99, (uint8_t)51, (uint8_t)79, (uint8_t)8, (uint8_t)232, (uint8_t)177, (uint8_t)18, (uint8_t)96, (uint8_t)21, (uint8_t)3, (uint8_t)162, (uint8_t)59, (uint8_t)95, (uint8_t)50, (uint8_t)164, (uint8_t)15, (uint8_t)205, (uint8_t)123, (uint8_t)98, (uint8_t)116, (uint8_t)110, (uint8_t)175, (uint8_t)152, (uint8_t)83, (uint8_t)15, (uint8_t)86, (uint8_t)37, (uint8_t)203, (uint8_t)179, (uint8_t)45, (uint8_t)90, (uint8_t)120, (uint8_t)115, (uint8_t)26, (uint8_t)63, (uint8_t)153, (uint8_t)211, (uint8_t)48, (uint8_t)38, (uint8_t)22, (uint8_t)95, (uint8_t)40, (uint8_t)93, (uint8_t)196, (uint8_t)83, (uint8_t)154, (uint8_t)163, (uint8_t)25} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_baudrate_GET(pack) == (uint32_t)1317965482L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1457971662);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p127_accuracy_GET(pack) == (uint32_t)3450257308L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)1553950476);
    assert(p127_tow_GET(pack) == (uint32_t)1809941862L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)3677915637L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -131308683);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)17772);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1661680039);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_accuracy_GET(pack) == (uint32_t)3384311445L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)825159956);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)226486067);
    assert(p128_tow_GET(pack) == (uint32_t)773927872L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)64867);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1299914287);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -85722918);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)2617604077L);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)3);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -16064);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)7065);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -999);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)7552);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -17174);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -18120);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -26083);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)27953);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -14683);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1839688710L);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p130_size_GET(pack) == (uint32_t)1575004946L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)14419);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)47650);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)46677);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)209, (uint8_t)193, (uint8_t)219, (uint8_t)172, (uint8_t)165, (uint8_t)68, (uint8_t)167, (uint8_t)175, (uint8_t)85, (uint8_t)17, (uint8_t)234, (uint8_t)17, (uint8_t)36, (uint8_t)85, (uint8_t)15, (uint8_t)146, (uint8_t)155, (uint8_t)4, (uint8_t)151, (uint8_t)44, (uint8_t)215, (uint8_t)65, (uint8_t)123, (uint8_t)48, (uint8_t)85, (uint8_t)235, (uint8_t)71, (uint8_t)151, (uint8_t)174, (uint8_t)239, (uint8_t)239, (uint8_t)95, (uint8_t)99, (uint8_t)191, (uint8_t)114, (uint8_t)197, (uint8_t)57, (uint8_t)255, (uint8_t)131, (uint8_t)229, (uint8_t)30, (uint8_t)184, (uint8_t)97, (uint8_t)213, (uint8_t)42, (uint8_t)121, (uint8_t)174, (uint8_t)133, (uint8_t)99, (uint8_t)116, (uint8_t)219, (uint8_t)160, (uint8_t)103, (uint8_t)46, (uint8_t)134, (uint8_t)59, (uint8_t)126, (uint8_t)207, (uint8_t)58, (uint8_t)90, (uint8_t)29, (uint8_t)198, (uint8_t)35, (uint8_t)159, (uint8_t)167, (uint8_t)198, (uint8_t)206, (uint8_t)222, (uint8_t)66, (uint8_t)72, (uint8_t)20, (uint8_t)121, (uint8_t)169, (uint8_t)201, (uint8_t)118, (uint8_t)11, (uint8_t)12, (uint8_t)116, (uint8_t)56, (uint8_t)91, (uint8_t)1, (uint8_t)46, (uint8_t)13, (uint8_t)129, (uint8_t)66, (uint8_t)11, (uint8_t)124, (uint8_t)111, (uint8_t)124, (uint8_t)181, (uint8_t)4, (uint8_t)18, (uint8_t)47, (uint8_t)103, (uint8_t)201, (uint8_t)253, (uint8_t)255, (uint8_t)146, (uint8_t)75, (uint8_t)224, (uint8_t)203, (uint8_t)66, (uint8_t)54, (uint8_t)75, (uint8_t)247, (uint8_t)250, (uint8_t)45, (uint8_t)3, (uint8_t)173, (uint8_t)104, (uint8_t)94, (uint8_t)32, (uint8_t)61, (uint8_t)2, (uint8_t)164, (uint8_t)176, (uint8_t)6, (uint8_t)105, (uint8_t)43, (uint8_t)136, (uint8_t)228, (uint8_t)108, (uint8_t)181, (uint8_t)157, (uint8_t)218, (uint8_t)226, (uint8_t)122, (uint8_t)4, (uint8_t)195, (uint8_t)18, (uint8_t)184, (uint8_t)166, (uint8_t)170, (uint8_t)112, (uint8_t)137, (uint8_t)40, (uint8_t)10, (uint8_t)184, (uint8_t)122, (uint8_t)100, (uint8_t)94, (uint8_t)231, (uint8_t)59, (uint8_t)213, (uint8_t)85, (uint8_t)74, (uint8_t)185, (uint8_t)227, (uint8_t)13, (uint8_t)41, (uint8_t)36, (uint8_t)44, (uint8_t)138, (uint8_t)210, (uint8_t)163, (uint8_t)129, (uint8_t)110, (uint8_t)248, (uint8_t)197, (uint8_t)152, (uint8_t)241, (uint8_t)13, (uint8_t)182, (uint8_t)63, (uint8_t)87, (uint8_t)193, (uint8_t)234, (uint8_t)115, (uint8_t)251, (uint8_t)233, (uint8_t)246, (uint8_t)13, (uint8_t)239, (uint8_t)19, (uint8_t)148, (uint8_t)46, (uint8_t)95, (uint8_t)205, (uint8_t)112, (uint8_t)175, (uint8_t)184, (uint8_t)253, (uint8_t)29, (uint8_t)20, (uint8_t)229, (uint8_t)222, (uint8_t)237, (uint8_t)138, (uint8_t)108, (uint8_t)167, (uint8_t)231, (uint8_t)221, (uint8_t)104, (uint8_t)217, (uint8_t)25, (uint8_t)68, (uint8_t)36, (uint8_t)215, (uint8_t)158, (uint8_t)107, (uint8_t)166, (uint8_t)253, (uint8_t)99, (uint8_t)205, (uint8_t)60, (uint8_t)96, (uint8_t)115, (uint8_t)236, (uint8_t)236, (uint8_t)55, (uint8_t)149, (uint8_t)224, (uint8_t)9, (uint8_t)114, (uint8_t)74, (uint8_t)240, (uint8_t)94, (uint8_t)87, (uint8_t)166, (uint8_t)189, (uint8_t)36, (uint8_t)117, (uint8_t)182, (uint8_t)178, (uint8_t)46, (uint8_t)208, (uint8_t)128, (uint8_t)176, (uint8_t)202, (uint8_t)235, (uint8_t)18, (uint8_t)230, (uint8_t)133, (uint8_t)69, (uint8_t)17, (uint8_t)138, (uint8_t)1, (uint8_t)117, (uint8_t)8, (uint8_t)39, (uint8_t)179, (uint8_t)121, (uint8_t)241, (uint8_t)15, (uint8_t)253, (uint8_t)230, (uint8_t)1, (uint8_t)182, (uint8_t)102, (uint8_t)51, (uint8_t)104, (uint8_t)152, (uint8_t)153} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)673);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)49697);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3273661576L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)1763);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)45432);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)8551005329862089722L);
    assert(p133_lat_GET(pack) == (int32_t)843643676);
    assert(p133_lon_GET(pack) == (int32_t) -853417661);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)24730);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)48477);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p134_lat_GET(pack) == (int32_t)1281321427);
    {
        int16_t exemplary[] =  {(int16_t) -19426, (int16_t)6192, (int16_t)29372, (int16_t) -32541, (int16_t) -11856, (int16_t)21362, (int16_t) -13975, (int16_t)14772, (int16_t)5461, (int16_t)4862, (int16_t)17822, (int16_t)21158, (int16_t)4734, (int16_t) -10135, (int16_t)15255, (int16_t) -24904} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t)932913780);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)977341911);
    assert(p135_lon_GET(pack) == (int32_t) -1647561783);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)23777);
    assert(p136_current_height_GET(pack) == (float) -5.2644924E36F);
    assert(p136_lon_GET(pack) == (int32_t) -1034020909);
    assert(p136_terrain_height_GET(pack) == (float)2.8410405E38F);
    assert(p136_lat_GET(pack) == (int32_t)925947156);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)8604);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)4186);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -16166);
    assert(p137_press_abs_GET(pack) == (float)2.6068143E38F);
    assert(p137_press_diff_GET(pack) == (float) -2.5149456E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)4016560535L);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)5980536357930566149L);
    assert(p138_y_GET(pack) == (float)3.1084097E38F);
    assert(p138_z_GET(pack) == (float)2.0361552E38F);
    assert(p138_x_GET(pack) == (float) -2.8895607E38F);
    {
        float exemplary[] =  {-1.2252261E38F, -8.7575376E36F, 1.681753E38F, -2.8685343E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)3536411804479647248L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)177);
    {
        float exemplary[] =  {7.8464384E36F, 1.4914905E38F, -1.2211145E38F, 6.6566985E37F, 2.5477546E38F, -1.7821658E38F, 3.2474553E38F, -1.780408E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.1984695E38F, -3.1091695E38F, -8.958287E37F, 1.9829749E38F, 3.032142E38F, -3.3821354E38F, -1.2694561E38F, 2.3702679E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p140_time_usec_GET(pack) == (uint64_t)4586872990326143893L);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_local_GET(pack) == (float) -1.0001361E38F);
    assert(p141_altitude_relative_GET(pack) == (float)9.183482E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -2.4132964E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -1.5380464E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -3.3409832E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)3.026209E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)124878507215970304L);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)35, (uint8_t)138, (uint8_t)36, (uint8_t)161, (uint8_t)196, (uint8_t)182, (uint8_t)12, (uint8_t)124, (uint8_t)164, (uint8_t)216, (uint8_t)53, (uint8_t)139, (uint8_t)118, (uint8_t)227, (uint8_t)145, (uint8_t)132, (uint8_t)86, (uint8_t)203, (uint8_t)158, (uint8_t)211, (uint8_t)125, (uint8_t)185, (uint8_t)194, (uint8_t)182, (uint8_t)18, (uint8_t)8, (uint8_t)111, (uint8_t)218, (uint8_t)11, (uint8_t)196, (uint8_t)251, (uint8_t)235, (uint8_t)216, (uint8_t)242, (uint8_t)202, (uint8_t)192, (uint8_t)212, (uint8_t)115, (uint8_t)53, (uint8_t)121, (uint8_t)73, (uint8_t)39, (uint8_t)222, (uint8_t)151, (uint8_t)29, (uint8_t)124, (uint8_t)143, (uint8_t)159, (uint8_t)203, (uint8_t)89, (uint8_t)176, (uint8_t)160, (uint8_t)205, (uint8_t)106, (uint8_t)165, (uint8_t)135, (uint8_t)160, (uint8_t)96, (uint8_t)61, (uint8_t)252, (uint8_t)247, (uint8_t)76, (uint8_t)231, (uint8_t)123, (uint8_t)40, (uint8_t)187, (uint8_t)106, (uint8_t)16, (uint8_t)220, (uint8_t)255, (uint8_t)179, (uint8_t)99, (uint8_t)199, (uint8_t)18, (uint8_t)66, (uint8_t)109, (uint8_t)182, (uint8_t)186, (uint8_t)151, (uint8_t)204, (uint8_t)69, (uint8_t)4, (uint8_t)59, (uint8_t)69, (uint8_t)251, (uint8_t)67, (uint8_t)251, (uint8_t)147, (uint8_t)136, (uint8_t)171, (uint8_t)187, (uint8_t)212, (uint8_t)56, (uint8_t)101, (uint8_t)243, (uint8_t)12, (uint8_t)35, (uint8_t)221, (uint8_t)203, (uint8_t)195, (uint8_t)41, (uint8_t)158, (uint8_t)58, (uint8_t)200, (uint8_t)201, (uint8_t)179, (uint8_t)141, (uint8_t)127, (uint8_t)26, (uint8_t)14, (uint8_t)109, (uint8_t)158, (uint8_t)13, (uint8_t)26, (uint8_t)76, (uint8_t)39, (uint8_t)99, (uint8_t)21, (uint8_t)196, (uint8_t)144} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)72);
    {
        uint8_t exemplary[] =  {(uint8_t)119, (uint8_t)128, (uint8_t)194, (uint8_t)253, (uint8_t)65, (uint8_t)35, (uint8_t)183, (uint8_t)181, (uint8_t)179, (uint8_t)199, (uint8_t)60, (uint8_t)146, (uint8_t)76, (uint8_t)175, (uint8_t)64, (uint8_t)169, (uint8_t)116, (uint8_t)164, (uint8_t)8, (uint8_t)118, (uint8_t)208, (uint8_t)109, (uint8_t)5, (uint8_t)13, (uint8_t)97, (uint8_t)155, (uint8_t)186, (uint8_t)145, (uint8_t)231, (uint8_t)47, (uint8_t)30, (uint8_t)77, (uint8_t)15, (uint8_t)194, (uint8_t)211, (uint8_t)132, (uint8_t)85, (uint8_t)183, (uint8_t)61, (uint8_t)45, (uint8_t)157, (uint8_t)23, (uint8_t)13, (uint8_t)148, (uint8_t)31, (uint8_t)78, (uint8_t)164, (uint8_t)244, (uint8_t)162, (uint8_t)255, (uint8_t)107, (uint8_t)220, (uint8_t)152, (uint8_t)8, (uint8_t)3, (uint8_t)10, (uint8_t)239, (uint8_t)18, (uint8_t)138, (uint8_t)153, (uint8_t)187, (uint8_t)159, (uint8_t)235, (uint8_t)65, (uint8_t)191, (uint8_t)214, (uint8_t)209, (uint8_t)110, (uint8_t)135, (uint8_t)206, (uint8_t)215, (uint8_t)50, (uint8_t)217, (uint8_t)185, (uint8_t)226, (uint8_t)10, (uint8_t)143, (uint8_t)205, (uint8_t)8, (uint8_t)113, (uint8_t)239, (uint8_t)209, (uint8_t)50, (uint8_t)15, (uint8_t)245, (uint8_t)116, (uint8_t)180, (uint8_t)198, (uint8_t)242, (uint8_t)224, (uint8_t)240, (uint8_t)104, (uint8_t)3, (uint8_t)102, (uint8_t)161, (uint8_t)239, (uint8_t)117, (uint8_t)89, (uint8_t)252, (uint8_t)73, (uint8_t)243, (uint8_t)65, (uint8_t)233, (uint8_t)36, (uint8_t)14, (uint8_t)247, (uint8_t)120, (uint8_t)2, (uint8_t)121, (uint8_t)239, (uint8_t)73, (uint8_t)176, (uint8_t)126, (uint8_t)147, (uint8_t)248, (uint8_t)73, (uint8_t)15, (uint8_t)158, (uint8_t)55, (uint8_t)200} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)2.471792E38F);
    assert(p143_press_diff_GET(pack) == (float)7.2793573E37F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)1233816172L);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -20398);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-6.1877585E37F, -2.7693991E38F, 2.7404085E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.5062812E37F, -3.125209E38F, -1.6939905E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.039113E38F, 3.3312225E38F, -6.524356E37F, 1.1358256E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t) -1392718340);
    assert(p144_custom_state_GET(pack) == (uint64_t)6210441172642704568L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)37);
    {
        float exemplary[] =  {-9.908583E37F, 2.4752292E38F, -2.201051E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)111284836);
    assert(p144_alt_GET(pack) == (float)2.6886861E38F);
    assert(p144_timestamp_GET(pack) == (uint64_t)6381942527891513783L);
    {
        float exemplary[] =  {-2.7560253E38F, -1.1068467E38F, 2.2030895E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_pos_GET(pack) == (float)4.654246E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)7736542273191355681L);
    assert(p146_y_pos_GET(pack) == (float)1.8418087E38F);
    {
        float exemplary[] =  {-2.4813242E38F, 2.747353E38F, -1.7955605E38F, 9.409315E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float)3.0321874E38F);
    {
        float exemplary[] =  {-2.3730711E38F, -4.7420715E37F, -1.080617E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_pos_GET(pack) == (float) -6.5846315E37F);
    assert(p146_y_acc_GET(pack) == (float) -9.802328E37F);
    assert(p146_pitch_rate_GET(pack) == (float) -1.4999511E38F);
    assert(p146_roll_rate_GET(pack) == (float) -2.9667828E37F);
    assert(p146_x_acc_GET(pack) == (float) -2.7117196E37F);
    assert(p146_z_vel_GET(pack) == (float) -2.252626E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.290356E38F);
    {
        float exemplary[] =  {6.573743E37F, 1.2661057E38F, 7.5094425E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float)1.4983169E38F);
    assert(p146_z_acc_GET(pack) == (float)1.183897E38F);
    assert(p146_x_vel_GET(pack) == (float) -6.202711E37F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)40);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -3266);
    {
        uint16_t exemplary[] =  {(uint16_t)25852, (uint16_t)42655, (uint16_t)36426, (uint16_t)25650, (uint16_t)17407, (uint16_t)49223, (uint16_t)48561, (uint16_t)19317, (uint16_t)52270, (uint16_t)47251} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_consumed_GET(pack) == (int32_t) -262913710);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -32165);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -1038579729);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3534720312L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2808021338L);
    {
        uint8_t exemplary[] =  {(uint8_t)130, (uint8_t)150, (uint8_t)218, (uint8_t)3, (uint8_t)231, (uint8_t)156, (uint8_t)246, (uint8_t)176} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)42280);
    {
        uint8_t exemplary[] =  {(uint8_t)185, (uint8_t)220, (uint8_t)240, (uint8_t)122, (uint8_t)76, (uint8_t)156, (uint8_t)148, (uint8_t)157} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)19692);
    assert(p148_uid_GET(pack) == (uint64_t)2187919626460316104L);
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)114, (uint8_t)1, (uint8_t)38, (uint8_t)120, (uint8_t)154, (uint8_t)181, (uint8_t)39, (uint8_t)143, (uint8_t)24, (uint8_t)19, (uint8_t)243, (uint8_t)73, (uint8_t)61, (uint8_t)174, (uint8_t)71, (uint8_t)213, (uint8_t)12} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3525153175L);
    {
        uint8_t exemplary[] =  {(uint8_t)55, (uint8_t)56, (uint8_t)183, (uint8_t)27, (uint8_t)30, (uint8_t)13, (uint8_t)12, (uint8_t)124} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)47399782L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_z_TRY(ph) == (float) -2.3352164E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p149_size_y_GET(pack) == (float)1.5738029E38F);
    assert(p149_angle_x_GET(pack) == (float) -3.517707E36F);
    assert(p149_angle_y_GET(pack) == (float)1.4247271E38F);
    assert(p149_size_x_GET(pack) == (float)1.9795392E38F);
    {
        float exemplary[] =  {-3.0973243E38F, -9.132675E37F, 1.5047419E38F, 3.670054E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_x_TRY(ph) == (float)3.0786864E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)158);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_distance_GET(pack) == (float)2.2381584E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p149_y_TRY(ph) == (float) -1.431919E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)6322294710075288096L);
};


void c_CommunicationChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_accel_cal_y_GET(pack) == (float)1.5554018E38F);
    assert(p150_mag_ofs_x_GET(pack) == (int16_t)(int16_t)17117);
    assert(p150_accel_cal_x_GET(pack) == (float)1.4943014E38F);
    assert(p150_raw_press_GET(pack) == (int32_t) -370899457);
    assert(p150_raw_temp_GET(pack) == (int32_t) -1199840572);
    assert(p150_gyro_cal_x_GET(pack) == (float) -1.5099644E38F);
    assert(p150_mag_ofs_z_GET(pack) == (int16_t)(int16_t) -32160);
    assert(p150_mag_declination_GET(pack) == (float)1.1616059E38F);
    assert(p150_accel_cal_z_GET(pack) == (float)1.076419E38F);
    assert(p150_mag_ofs_y_GET(pack) == (int16_t)(int16_t) -26727);
    assert(p150_gyro_cal_y_GET(pack) == (float)1.0868558E38F);
    assert(p150_gyro_cal_z_GET(pack) == (float)1.8511307E38F);
};


void c_CommunicationChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_mag_ofs_z_GET(pack) == (int16_t)(int16_t)12590);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p151_mag_ofs_x_GET(pack) == (int16_t)(int16_t)27736);
    assert(p151_mag_ofs_y_GET(pack) == (int16_t)(int16_t)32388);
};


void c_CommunicationChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_brkval_GET(pack) == (uint16_t)(uint16_t)35492);
    assert(p152_freemem_GET(pack) == (uint16_t)(uint16_t)16207);
    assert(p152_freemem32_TRY(ph) == (uint32_t)1494387357L);
};


void c_CommunicationChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_adc2_GET(pack) == (uint16_t)(uint16_t)31841);
    assert(p153_adc5_GET(pack) == (uint16_t)(uint16_t)36176);
    assert(p153_adc4_GET(pack) == (uint16_t)(uint16_t)20549);
    assert(p153_adc3_GET(pack) == (uint16_t)(uint16_t)54271);
    assert(p153_adc6_GET(pack) == (uint16_t)(uint16_t)3569);
    assert(p153_adc1_GET(pack) == (uint16_t)(uint16_t)58040);
};


void c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    assert(p154_target_component_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p154_extra_value_GET(pack) == (float)3.706903E37F);
    assert(p154_exposure_type_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p154_target_system_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p154_mode_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p154_engine_cut_off_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p154_extra_param_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p154_command_id_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p154_shutter_speed_GET(pack) == (uint16_t)(uint16_t)19249);
    assert(p154_iso_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p154_aperture_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_CommunicationChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_zoom_pos_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p155_extra_param_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p155_session_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p155_command_id_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p155_extra_value_GET(pack) == (float) -8.57339E37F);
    assert(p155_focus_lock_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p155_shot_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p155_zoom_step_GET(pack) == (int8_t)(int8_t) -52);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)175);
};


void c_CommunicationChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_stab_roll_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p156_mount_mode_GET(pack) == e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RC_TARGETING);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p156_stab_pitch_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p156_stab_yaw_GET(pack) == (uint8_t)(uint8_t)5);
};


void c_CommunicationChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_save_position_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p157_input_a_GET(pack) == (int32_t) -1361309425);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p157_input_b_GET(pack) == (int32_t)1735441738);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p157_input_c_GET(pack) == (int32_t)124889440);
};


void c_CommunicationChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_pointing_c_GET(pack) == (int32_t) -1174442133);
    assert(p158_target_system_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p158_pointing_b_GET(pack) == (int32_t) -1188738090);
    assert(p158_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p158_pointing_a_GET(pack) == (int32_t) -482214499);
};


void c_CommunicationChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    assert(p160_lat_GET(pack) == (float) -5.904557E37F);
    assert(p160_target_component_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p160_lng_GET(pack) == (float)3.2317663E38F);
    assert(p160_count_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p160_target_system_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p160_idx_GET(pack) == (uint8_t)(uint8_t)152);
};


void c_CommunicationChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    assert(p161_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p161_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p161_idx_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_CommunicationChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    assert(p162_breach_status_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p162_breach_type_GET(pack) == e_FENCE_BREACH_FENCE_BREACH_NONE);
    assert(p162_breach_count_GET(pack) == (uint16_t)(uint16_t)46627);
    assert(p162_breach_time_GET(pack) == (uint32_t)1697244309L);
};


void c_CommunicationChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    assert(p163_omegaIy_GET(pack) == (float)3.0565433E38F);
    assert(p163_omegaIx_GET(pack) == (float) -8.995235E37F);
    assert(p163_error_rp_GET(pack) == (float) -2.726368E38F);
    assert(p163_omegaIz_GET(pack) == (float) -2.2814275E38F);
    assert(p163_error_yaw_GET(pack) == (float) -1.2982742E38F);
    assert(p163_renorm_val_GET(pack) == (float)1.1589041E38F);
    assert(p163_accel_weight_GET(pack) == (float)1.4821014E38F);
};


void c_CommunicationChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    assert(p164_xacc_GET(pack) == (float)1.596351E38F);
    assert(p164_zgyro_GET(pack) == (float) -1.4882056E38F);
    assert(p164_zacc_GET(pack) == (float)2.6915311E38F);
    assert(p164_roll_GET(pack) == (float)2.8124936E38F);
    assert(p164_lng_GET(pack) == (int32_t)572870150);
    assert(p164_ygyro_GET(pack) == (float)2.2995001E38F);
    assert(p164_yacc_GET(pack) == (float) -6.403532E37F);
    assert(p164_pitch_GET(pack) == (float)2.4826075E38F);
    assert(p164_lat_GET(pack) == (int32_t) -2034520948);
    assert(p164_xgyro_GET(pack) == (float)2.790566E38F);
    assert(p164_yaw_GET(pack) == (float)1.3144662E38F);
};


void c_CommunicationChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    assert(p165_Vcc_GET(pack) == (uint16_t)(uint16_t)33402);
    assert(p165_I2Cerr_GET(pack) == (uint8_t)(uint8_t)232);
};


void c_CommunicationChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    assert(p166_rssi_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p166_rxerrors_GET(pack) == (uint16_t)(uint16_t)58191);
    assert(p166_fixed__GET(pack) == (uint16_t)(uint16_t)2606);
    assert(p166_noise_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p166_remrssi_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p166_txbuf_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p166_remnoise_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_CommunicationChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    assert(p167_breach_count_GET(pack) == (uint16_t)(uint16_t)17626);
    assert(p167_mods_enabled_GET(pack) == (e_LIMIT_MODULE_LIMIT_GPSLOCK));
    assert(p167_mods_triggered_GET(pack) == (e_LIMIT_MODULE_LIMIT_GEOFENCE |
            e_LIMIT_MODULE_LIMIT_ALTITUDE));
    assert(p167_last_clear_GET(pack) == (uint32_t)4088393930L);
    assert(p167_last_trigger_GET(pack) == (uint32_t)4289879297L);
    assert(p167_last_action_GET(pack) == (uint32_t)1365279422L);
    assert(p167_mods_required_GET(pack) == (e_LIMIT_MODULE_LIMIT_GEOFENCE));
    assert(p167_limits_state_GET(pack) == e_LIMITS_STATE_LIMITS_RECOVERING);
    assert(p167_last_recovery_GET(pack) == (uint32_t)642557098L);
};


void c_CommunicationChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    assert(p168_direction_GET(pack) == (float)1.4033336E38F);
    assert(p168_speed_GET(pack) == (float) -2.9985454E38F);
    assert(p168_speed_z_GET(pack) == (float)4.9718255E37F);
};


void c_CommunicationChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    assert(p169_len_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p169_type_GET(pack) == (uint8_t)(uint8_t)165);
    {
        uint8_t exemplary[] =  {(uint8_t)158, (uint8_t)238, (uint8_t)141, (uint8_t)167, (uint8_t)249, (uint8_t)105, (uint8_t)88, (uint8_t)171, (uint8_t)16, (uint8_t)62, (uint8_t)127, (uint8_t)107, (uint8_t)7, (uint8_t)91, (uint8_t)165, (uint8_t)126} ;
        uint8_t*  sample = p169_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_type_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p170_len_GET(pack) == (uint8_t)(uint8_t)82);
    {
        uint8_t exemplary[] =  {(uint8_t)59, (uint8_t)87, (uint8_t)29, (uint8_t)226, (uint8_t)212, (uint8_t)192, (uint8_t)1, (uint8_t)110, (uint8_t)31, (uint8_t)137, (uint8_t)207, (uint8_t)211, (uint8_t)152, (uint8_t)171, (uint8_t)254, (uint8_t)102, (uint8_t)58, (uint8_t)47, (uint8_t)30, (uint8_t)234, (uint8_t)84, (uint8_t)114, (uint8_t)199, (uint8_t)110, (uint8_t)16, (uint8_t)118, (uint8_t)113, (uint8_t)72, (uint8_t)1, (uint8_t)131, (uint8_t)116, (uint8_t)204} ;
        uint8_t*  sample = p170_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_len_GET(pack) == (uint8_t)(uint8_t)132);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)175, (uint8_t)57, (uint8_t)123, (uint8_t)232, (uint8_t)23, (uint8_t)43, (uint8_t)205, (uint8_t)225, (uint8_t)43, (uint8_t)136, (uint8_t)107, (uint8_t)125, (uint8_t)154, (uint8_t)117, (uint8_t)188, (uint8_t)39, (uint8_t)186, (uint8_t)23, (uint8_t)28, (uint8_t)124, (uint8_t)138, (uint8_t)32, (uint8_t)216, (uint8_t)59, (uint8_t)231, (uint8_t)19, (uint8_t)125, (uint8_t)26, (uint8_t)104, (uint8_t)22, (uint8_t)201, (uint8_t)5, (uint8_t)208, (uint8_t)160, (uint8_t)174, (uint8_t)33, (uint8_t)89, (uint8_t)173, (uint8_t)152, (uint8_t)147, (uint8_t)218, (uint8_t)228, (uint8_t)50, (uint8_t)84, (uint8_t)222, (uint8_t)133, (uint8_t)26, (uint8_t)188, (uint8_t)62, (uint8_t)154, (uint8_t)122, (uint8_t)184, (uint8_t)140, (uint8_t)241, (uint8_t)212, (uint8_t)108, (uint8_t)25, (uint8_t)155, (uint8_t)51, (uint8_t)61, (uint8_t)203, (uint8_t)219, (uint8_t)107} ;
        uint8_t*  sample = p171_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p171_type_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_CommunicationChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)130, (uint8_t)0, (uint8_t)98, (uint8_t)240, (uint8_t)154, (uint8_t)233, (uint8_t)47, (uint8_t)179, (uint8_t)193, (uint8_t)83, (uint8_t)122, (uint8_t)153, (uint8_t)193, (uint8_t)103, (uint8_t)37, (uint8_t)110, (uint8_t)37, (uint8_t)123, (uint8_t)120, (uint8_t)236, (uint8_t)69, (uint8_t)106, (uint8_t)147, (uint8_t)24, (uint8_t)26, (uint8_t)74, (uint8_t)6, (uint8_t)122, (uint8_t)234, (uint8_t)194, (uint8_t)106, (uint8_t)167, (uint8_t)65, (uint8_t)134, (uint8_t)97, (uint8_t)91, (uint8_t)122, (uint8_t)98, (uint8_t)218, (uint8_t)119, (uint8_t)77, (uint8_t)209, (uint8_t)92, (uint8_t)230, (uint8_t)42, (uint8_t)50, (uint8_t)87, (uint8_t)42, (uint8_t)142, (uint8_t)192, (uint8_t)200, (uint8_t)249, (uint8_t)180, (uint8_t)165, (uint8_t)43, (uint8_t)179, (uint8_t)27, (uint8_t)124, (uint8_t)150, (uint8_t)22, (uint8_t)45, (uint8_t)73, (uint8_t)134, (uint8_t)249, (uint8_t)53, (uint8_t)192, (uint8_t)178, (uint8_t)139, (uint8_t)235, (uint8_t)129, (uint8_t)87, (uint8_t)37, (uint8_t)212, (uint8_t)221, (uint8_t)215, (uint8_t)216, (uint8_t)242, (uint8_t)246, (uint8_t)93, (uint8_t)217, (uint8_t)144, (uint8_t)59, (uint8_t)80, (uint8_t)132, (uint8_t)241, (uint8_t)36, (uint8_t)223, (uint8_t)103, (uint8_t)237, (uint8_t)73, (uint8_t)76, (uint8_t)98, (uint8_t)15, (uint8_t)42, (uint8_t)96, (uint8_t)93} ;
        uint8_t*  sample = p172_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p172_len_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p172_type_GET(pack) == (uint8_t)(uint8_t)119);
};


void c_CommunicationChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_voltage_GET(pack) == (float)1.5461225E38F);
    assert(p173_distance_GET(pack) == (float)3.3154503E38F);
};


void c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_Pax_GET(pack) == (float)2.8704682E38F);
    assert(p174_vz_GET(pack) == (float) -1.2430219E37F);
    assert(p174_state_y_GET(pack) == (float) -1.7265774E38F);
    assert(p174_vx_GET(pack) == (float) -2.8254908E36F);
    assert(p174_ratio_GET(pack) == (float) -2.2657522E38F);
    assert(p174_vy_GET(pack) == (float)3.0999807E38F);
    assert(p174_state_x_GET(pack) == (float) -2.4271365E38F);
    assert(p174_Pcz_GET(pack) == (float)3.3781459E38F);
    assert(p174_EAS2TAS_GET(pack) == (float)3.3208326E38F);
    assert(p174_diff_pressure_GET(pack) == (float)1.9219365E37F);
    assert(p174_state_z_GET(pack) == (float) -2.0003048E38F);
    assert(p174_Pby_GET(pack) == (float) -5.4656525E37F);
};


void c_CommunicationChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_count_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p175_break_alt_GET(pack) == (int16_t)(int16_t) -20233);
    assert(p175_idx_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p175_flags_GET(pack) == e_RALLY_FLAGS_LAND_IMMEDIATELY);
    assert(p175_lat_GET(pack) == (int32_t)767257486);
    assert(p175_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p175_lng_GET(pack) == (int32_t)1063137977);
    assert(p175_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p175_alt_GET(pack) == (int16_t)(int16_t) -28009);
    assert(p175_land_dir_GET(pack) == (uint16_t)(uint16_t)57319);
};


void c_CommunicationChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_target_system_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p176_target_component_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p176_idx_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_interference_GET(pack) == (uint16_t)(uint16_t)10022);
    assert(p177_current_GET(pack) == (float) -5.42921E37F);
    assert(p177_throttle_GET(pack) == (uint16_t)(uint16_t)60338);
    assert(p177_CompensationZ_GET(pack) == (float)1.662567E38F);
    assert(p177_CompensationY_GET(pack) == (float)3.3197208E38F);
    assert(p177_CompensationX_GET(pack) == (float) -2.9497733E38F);
};


void c_CommunicationChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_lng_GET(pack) == (int32_t)559719448);
    assert(p178_lat_GET(pack) == (int32_t)565820019);
    assert(p178_yaw_GET(pack) == (float)2.9503133E37F);
    assert(p178_pitch_GET(pack) == (float)1.1175339E38F);
    assert(p178_roll_GET(pack) == (float)1.4585114E38F);
    assert(p178_altitude_GET(pack) == (float)2.270756E38F);
};


void c_CommunicationChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_event_id_GET(pack) == e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR);
    assert(p179_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p179_p4_GET(pack) == (float) -1.0012088E38F);
    assert(p179_p1_GET(pack) == (float)2.3013347E38F);
    assert(p179_p3_GET(pack) == (float)2.5411113E38F);
    assert(p179_time_usec_GET(pack) == (uint64_t)1813203923314458843L);
    assert(p179_p2_GET(pack) == (float)2.1331694E38F);
    assert(p179_cam_idx_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p179_img_idx_GET(pack) == (uint16_t)(uint16_t)7270);
};


void c_CommunicationChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_lng_GET(pack) == (int32_t) -18547997);
    assert(p180_lat_GET(pack) == (int32_t) -2129802835);
    assert(p180_flags_GET(pack) == e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO);
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p180_yaw_GET(pack) == (float)9.023342E37F);
    assert(p180_time_usec_GET(pack) == (uint64_t)1684644520655252709L);
    assert(p180_pitch_GET(pack) == (float)3.1334234E38F);
    assert(p180_roll_GET(pack) == (float)2.742824E37F);
    assert(p180_alt_msl_GET(pack) == (float) -1.2652611E38F);
    assert(p180_img_idx_GET(pack) == (uint16_t)(uint16_t)49924);
    assert(p180_alt_rel_GET(pack) == (float) -3.1635886E38F);
    assert(p180_cam_idx_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p180_foc_len_GET(pack) == (float) -3.1009594E38F);
};


void c_CommunicationChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_current_battery_GET(pack) == (int16_t)(int16_t)17857);
    assert(p181_voltage_GET(pack) == (uint16_t)(uint16_t)8104);
};


void c_CommunicationChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_pitch_GET(pack) == (float)1.5140253E38F);
    assert(p182_v4_GET(pack) == (float) -1.0751439E38F);
    assert(p182_v3_GET(pack) == (float) -3.347101E38F);
    assert(p182_lat_GET(pack) == (int32_t)1724200803);
    assert(p182_lng_GET(pack) == (int32_t)58734568);
    assert(p182_altitude_GET(pack) == (float) -1.130815E38F);
    assert(p182_v2_GET(pack) == (float) -2.3259068E38F);
    assert(p182_yaw_GET(pack) == (float)1.7820107E38F);
    assert(p182_v1_GET(pack) == (float) -1.0328906E38F);
    assert(p182_roll_GET(pack) == (float)2.6145E38F);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_seqno_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP);
    assert(p184_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p184_target_system_GET(pack) == (uint8_t)(uint8_t)221);
    {
        uint8_t exemplary[] =  {(uint8_t)106, (uint8_t)14, (uint8_t)188, (uint8_t)65, (uint8_t)232, (uint8_t)154, (uint8_t)224, (uint8_t)82, (uint8_t)44, (uint8_t)80, (uint8_t)45, (uint8_t)219, (uint8_t)79, (uint8_t)8, (uint8_t)33, (uint8_t)4, (uint8_t)91, (uint8_t)176, (uint8_t)172, (uint8_t)248, (uint8_t)230, (uint8_t)234, (uint8_t)168, (uint8_t)79, (uint8_t)73, (uint8_t)218, (uint8_t)233, (uint8_t)200, (uint8_t)211, (uint8_t)70, (uint8_t)43, (uint8_t)66, (uint8_t)93, (uint8_t)205, (uint8_t)143, (uint8_t)225, (uint8_t)243, (uint8_t)229, (uint8_t)48, (uint8_t)13, (uint8_t)87, (uint8_t)58, (uint8_t)80, (uint8_t)120, (uint8_t)84, (uint8_t)31, (uint8_t)122, (uint8_t)100, (uint8_t)15, (uint8_t)72, (uint8_t)17, (uint8_t)68, (uint8_t)46, (uint8_t)28, (uint8_t)254, (uint8_t)25, (uint8_t)186, (uint8_t)243, (uint8_t)209, (uint8_t)229, (uint8_t)47, (uint8_t)234, (uint8_t)252, (uint8_t)1, (uint8_t)145, (uint8_t)68, (uint8_t)107, (uint8_t)188, (uint8_t)132, (uint8_t)22, (uint8_t)26, (uint8_t)248, (uint8_t)144, (uint8_t)11, (uint8_t)89, (uint8_t)118, (uint8_t)203, (uint8_t)235, (uint8_t)60, (uint8_t)225, (uint8_t)199, (uint8_t)176, (uint8_t)147, (uint8_t)152, (uint8_t)104, (uint8_t)255, (uint8_t)191, (uint8_t)212, (uint8_t)107, (uint8_t)107, (uint8_t)244, (uint8_t)246, (uint8_t)224, (uint8_t)98, (uint8_t)164, (uint8_t)226, (uint8_t)96, (uint8_t)179, (uint8_t)174, (uint8_t)199, (uint8_t)152, (uint8_t)102, (uint8_t)230, (uint8_t)159, (uint8_t)74, (uint8_t)235, (uint8_t)202, (uint8_t)188, (uint8_t)229, (uint8_t)8, (uint8_t)199, (uint8_t)152, (uint8_t)52, (uint8_t)184, (uint8_t)127, (uint8_t)107, (uint8_t)232, (uint8_t)220, (uint8_t)114, (uint8_t)253, (uint8_t)35, (uint8_t)250, (uint8_t)51, (uint8_t)248, (uint8_t)107, (uint8_t)2, (uint8_t)185, (uint8_t)102, (uint8_t)18, (uint8_t)118, (uint8_t)235, (uint8_t)240, (uint8_t)98, (uint8_t)224, (uint8_t)144, (uint8_t)164, (uint8_t)137, (uint8_t)58, (uint8_t)44, (uint8_t)239, (uint8_t)100, (uint8_t)137, (uint8_t)66, (uint8_t)252, (uint8_t)214, (uint8_t)89, (uint8_t)228, (uint8_t)79, (uint8_t)61, (uint8_t)177, (uint8_t)156, (uint8_t)190, (uint8_t)88, (uint8_t)192, (uint8_t)129, (uint8_t)245, (uint8_t)210, (uint8_t)7, (uint8_t)220, (uint8_t)163, (uint8_t)89, (uint8_t)188, (uint8_t)146, (uint8_t)186, (uint8_t)222, (uint8_t)106, (uint8_t)255, (uint8_t)236, (uint8_t)240, (uint8_t)228, (uint8_t)25, (uint8_t)70, (uint8_t)7, (uint8_t)17, (uint8_t)74, (uint8_t)121, (uint8_t)35, (uint8_t)165, (uint8_t)65, (uint8_t)121, (uint8_t)13, (uint8_t)15, (uint8_t)238, (uint8_t)60, (uint8_t)78, (uint8_t)58, (uint8_t)50, (uint8_t)227, (uint8_t)110, (uint8_t)170, (uint8_t)201, (uint8_t)175, (uint8_t)199, (uint8_t)123, (uint8_t)204, (uint8_t)237, (uint8_t)196, (uint8_t)53, (uint8_t)56, (uint8_t)123} ;
        uint8_t*  sample = p184_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 200);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_target_system_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p185_seqno_GET(pack) == (uint32_t)2195765879L);
    assert(p185_status_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK);
    assert(p185_target_component_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_CommunicationChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_instance_GET(pack) == (uint8_t)(uint8_t)172);
    {
        uint8_t exemplary[] =  {(uint8_t)59, (uint8_t)100, (uint8_t)211, (uint8_t)86, (uint8_t)246, (uint8_t)206, (uint8_t)232, (uint8_t)51, (uint8_t)77, (uint8_t)252, (uint8_t)187, (uint8_t)122, (uint8_t)237, (uint8_t)47, (uint8_t)163, (uint8_t)69, (uint8_t)190, (uint8_t)188, (uint8_t)62, (uint8_t)83, (uint8_t)225, (uint8_t)130, (uint8_t)68, (uint8_t)91} ;
        uint8_t*  sample = p186_custom_bytes_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p186_pattern_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p186_target_component_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p186_target_system_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p186_custom_len_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_direction_x_GET(pack) == (float)2.4797787E38F);
    assert(p191_compass_id_GET(pack) == (uint8_t)(uint8_t)6);
    {
        uint8_t exemplary[] =  {(uint8_t)112, (uint8_t)54, (uint8_t)26, (uint8_t)73, (uint8_t)235, (uint8_t)38, (uint8_t)48, (uint8_t)77, (uint8_t)200, (uint8_t)208} ;
        uint8_t*  sample = p191_completion_mask_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p191_direction_y_GET(pack) == (float)7.9344903E37F);
    assert(p191_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START);
    assert(p191_attempt_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p191_direction_z_GET(pack) == (float) -3.3482805E37F);
    assert(p191_cal_mask_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p191_completion_pct_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_CommunicationChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_cal_mask_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p192_autosaved_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p192_offdiag_y_GET(pack) == (float) -1.4144716E38F);
    assert(p192_fitness_GET(pack) == (float) -2.2293881E38F);
    assert(p192_ofs_y_GET(pack) == (float) -1.2385312E38F);
    assert(p192_diag_x_GET(pack) == (float) -3.3576414E38F);
    assert(p192_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_FAILED);
    assert(p192_ofs_z_GET(pack) == (float)6.203774E37F);
    assert(p192_diag_z_GET(pack) == (float)2.0762151E38F);
    assert(p192_compass_id_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p192_diag_y_GET(pack) == (float)2.1729227E38F);
    assert(p192_ofs_x_GET(pack) == (float)2.6810784E38F);
    assert(p192_offdiag_x_GET(pack) == (float)3.039634E38F);
    assert(p192_offdiag_z_GET(pack) == (float) -9.947609E37F);
};


void c_CommunicationChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_pos_horiz_variance_GET(pack) == (float)1.087927E37F);
    assert(p193_velocity_variance_GET(pack) == (float) -2.3364194E38F);
    assert(p193_pos_vert_variance_GET(pack) == (float) -1.492568E38F);
    assert(p193_terrain_alt_variance_GET(pack) == (float) -7.6102476E37F);
    assert(p193_flags_GET(pack) == (e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS |
                                    e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL |
                                    e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL |
                                    e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ));
    assert(p193_compass_variance_GET(pack) == (float) -9.372733E37F);
};


void c_CommunicationChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_achieved_GET(pack) == (float)8.573471E37F);
    assert(p194_P_GET(pack) == (float) -2.933069E36F);
    assert(p194_desired_GET(pack) == (float)2.8384587E38F);
    assert(p194_FF_GET(pack) == (float) -1.5631198E37F);
    assert(p194_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_STEER);
    assert(p194_D_GET(pack) == (float) -2.8108233E38F);
    assert(p194_I_GET(pack) == (float) -1.8125426E38F);
};


void c_CommunicationChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    assert(p200_delta_velocity_y_GET(pack) == (float) -3.1055955E38F);
    assert(p200_delta_angle_z_GET(pack) == (float)1.7512433E38F);
    assert(p200_delta_angle_y_GET(pack) == (float)6.265011E37F);
    assert(p200_joint_el_GET(pack) == (float) -8.3624096E37F);
    assert(p200_delta_angle_x_GET(pack) == (float) -2.5342773E38F);
    assert(p200_delta_velocity_x_GET(pack) == (float)2.3312277E38F);
    assert(p200_target_component_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p200_joint_az_GET(pack) == (float)8.332525E37F);
    assert(p200_delta_time_GET(pack) == (float) -1.852975E37F);
    assert(p200_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p200_delta_velocity_z_GET(pack) == (float) -1.8079251E38F);
    assert(p200_joint_roll_GET(pack) == (float)1.6981836E38F);
};


void c_CommunicationChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p201_demanded_rate_x_GET(pack) == (float) -2.4393044E38F);
    assert(p201_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p201_demanded_rate_y_GET(pack) == (float)2.4033169E38F);
    assert(p201_demanded_rate_z_GET(pack) == (float) -1.3216473E38F);
};


void c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    assert(p214_target_system_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p214_el_torque_cmd_GET(pack) == (int16_t)(int16_t)216);
    assert(p214_az_torque_cmd_GET(pack) == (int16_t)(int16_t) -9174);
    assert(p214_rl_torque_cmd_GET(pack) == (int16_t)(int16_t)12684);
    assert(p214_target_component_GET(pack) == (uint8_t)(uint8_t)22);
};


void c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    assert(p215_status_GET(pack) == e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE);
    assert(p215_flags_GET(pack) == e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING);
    assert(p215_capture_mode_GET(pack) == e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP);
};


void c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    assert(p216_target_component_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p216_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION);
    assert(p216_target_system_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    assert(p217_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_CAPTURE_MODE);
    {
        uint8_t exemplary[] =  {(uint8_t)61, (uint8_t)176, (uint8_t)79, (uint8_t)148} ;
        uint8_t*  sample = p217_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p217_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS);
};


void c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    assert(p218_target_system_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p218_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN);
    assert(p218_target_component_GET(pack) == (uint8_t)(uint8_t)121);
    {
        uint8_t exemplary[] =  {(uint8_t)24, (uint8_t)125, (uint8_t)98, (uint8_t)53} ;
        uint8_t*  sample = p218_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    assert(p219_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_TIME);
    assert(p219_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED);
};


void c_CommunicationChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    assert(p226_rpm1_GET(pack) == (float)2.5405391E37F);
    assert(p226_rpm2_GET(pack) == (float)3.2198364E38F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.6826913E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -9.806025E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -3.0592784E38F);
    assert(p230_hagl_ratio_GET(pack) == (float)6.9205397E37F);
    assert(p230_tas_ratio_GET(pack) == (float)8.0126065E37F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.8920403E36F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -1.41696E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)52909230062959987L);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT));
    assert(p230_mag_ratio_GET(pack) == (float)3.3586106E37F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float) -8.79237E37F);
    assert(p231_var_horiz_GET(pack) == (float)3.9489253E37F);
    assert(p231_time_usec_GET(pack) == (uint64_t)4328196724106380199L);
    assert(p231_wind_z_GET(pack) == (float) -2.6481588E38F);
    assert(p231_wind_y_GET(pack) == (float) -2.4582615E37F);
    assert(p231_vert_accuracy_GET(pack) == (float) -4.3740325E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float)2.0598673E38F);
    assert(p231_var_vert_GET(pack) == (float)1.6474992E38F);
    assert(p231_wind_x_GET(pack) == (float)9.895936E37F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_horiz_accuracy_GET(pack) == (float) -2.1015633E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)4023620504202445699L);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2688334277L);
    assert(p232_vd_GET(pack) == (float)2.5334204E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p232_ve_GET(pack) == (float) -2.5006201E38F);
    assert(p232_speed_accuracy_GET(pack) == (float) -2.6597072E38F);
    assert(p232_hdop_GET(pack) == (float)2.679998E38F);
    assert(p232_vdop_GET(pack) == (float) -2.3450037E38F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
    assert(p232_vn_GET(pack) == (float)2.9899034E38F);
    assert(p232_lon_GET(pack) == (int32_t)219281835);
    assert(p232_alt_GET(pack) == (float)8.0958997E37F);
    assert(p232_vert_accuracy_GET(pack) == (float) -2.7800649E38F);
    assert(p232_lat_GET(pack) == (int32_t)52374527);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)29818);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)88);
    {
        uint8_t exemplary[] =  {(uint8_t)39, (uint8_t)59, (uint8_t)114, (uint8_t)55, (uint8_t)236, (uint8_t)226, (uint8_t)52, (uint8_t)14, (uint8_t)128, (uint8_t)153, (uint8_t)208, (uint8_t)154, (uint8_t)255, (uint8_t)117, (uint8_t)234, (uint8_t)240, (uint8_t)153, (uint8_t)219, (uint8_t)41, (uint8_t)163, (uint8_t)183, (uint8_t)114, (uint8_t)130, (uint8_t)185, (uint8_t)154, (uint8_t)219, (uint8_t)219, (uint8_t)33, (uint8_t)184, (uint8_t)61, (uint8_t)194, (uint8_t)142, (uint8_t)92, (uint8_t)186, (uint8_t)79, (uint8_t)51, (uint8_t)136, (uint8_t)86, (uint8_t)108, (uint8_t)3, (uint8_t)70, (uint8_t)176, (uint8_t)35, (uint8_t)230, (uint8_t)115, (uint8_t)127, (uint8_t)107, (uint8_t)101, (uint8_t)189, (uint8_t)106, (uint8_t)222, (uint8_t)231, (uint8_t)254, (uint8_t)65, (uint8_t)134, (uint8_t)233, (uint8_t)13, (uint8_t)250, (uint8_t)231, (uint8_t)104, (uint8_t)17, (uint8_t)136, (uint8_t)81, (uint8_t)164, (uint8_t)101, (uint8_t)67, (uint8_t)217, (uint8_t)148, (uint8_t)218, (uint8_t)3, (uint8_t)76, (uint8_t)187, (uint8_t)245, (uint8_t)75, (uint8_t)67, (uint8_t)133, (uint8_t)181, (uint8_t)65, (uint8_t)164, (uint8_t)127, (uint8_t)168, (uint8_t)229, (uint8_t)243, (uint8_t)48, (uint8_t)24, (uint8_t)213, (uint8_t)44, (uint8_t)101, (uint8_t)19, (uint8_t)250, (uint8_t)9, (uint8_t)89, (uint8_t)12, (uint8_t)61, (uint8_t)197, (uint8_t)128, (uint8_t)131, (uint8_t)193, (uint8_t)239, (uint8_t)45, (uint8_t)29, (uint8_t)106, (uint8_t)181, (uint8_t)187, (uint8_t)146, (uint8_t)178, (uint8_t)167, (uint8_t)203, (uint8_t)64, (uint8_t)60, (uint8_t)107, (uint8_t)116, (uint8_t)15, (uint8_t)202, (uint8_t)139, (uint8_t)22, (uint8_t)101, (uint8_t)151, (uint8_t)165, (uint8_t)200, (uint8_t)182, (uint8_t)92, (uint8_t)162, (uint8_t)134, (uint8_t)108, (uint8_t)225, (uint8_t)254, (uint8_t)134, (uint8_t)118, (uint8_t)218, (uint8_t)143, (uint8_t)43, (uint8_t)210, (uint8_t)252, (uint8_t)209, (uint8_t)19, (uint8_t)83, (uint8_t)102, (uint8_t)95, (uint8_t)186, (uint8_t)185, (uint8_t)96, (uint8_t)126, (uint8_t)44, (uint8_t)122, (uint8_t)186, (uint8_t)134, (uint8_t)130, (uint8_t)81, (uint8_t)127, (uint8_t)167, (uint8_t)131, (uint8_t)221, (uint8_t)17, (uint8_t)28, (uint8_t)226, (uint8_t)123, (uint8_t)43, (uint8_t)62, (uint8_t)199, (uint8_t)89, (uint8_t)71, (uint8_t)84, (uint8_t)235, (uint8_t)195, (uint8_t)11, (uint8_t)168, (uint8_t)71, (uint8_t)157, (uint8_t)133, (uint8_t)9, (uint8_t)154, (uint8_t)97, (uint8_t)238, (uint8_t)78, (uint8_t)225, (uint8_t)49, (uint8_t)248, (uint8_t)2, (uint8_t)39} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)109);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_custom_mode_GET(pack) == (uint32_t)1505511269L);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)13);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)10029);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p234_latitude_GET(pack) == (int32_t)1778097392);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)15800);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)47491);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -9);
    assert(p234_longitude_GET(pack) == (int32_t) -1135753192);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -68);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED));
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -21078);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -100);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -24214);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -6178);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -5613);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)1377431044L);
    assert(p241_vibration_z_GET(pack) == (float)2.6878312E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)7041652304032582572L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3561449687L);
    assert(p241_vibration_y_GET(pack) == (float)8.534597E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)274702238L);
    assert(p241_vibration_x_GET(pack) == (float) -3.2309302E38F);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_approach_x_GET(pack) == (float)2.0599983E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)2376926908481052774L);
    assert(p242_altitude_GET(pack) == (int32_t)1939014740);
    assert(p242_longitude_GET(pack) == (int32_t)973944531);
    assert(p242_latitude_GET(pack) == (int32_t) -1642709340);
    assert(p242_z_GET(pack) == (float)2.48446E38F);
    {
        float exemplary[] =  {2.3875956E38F, 1.4188326E37F, -7.472671E37F, -2.3242964E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_x_GET(pack) == (float)2.9702757E37F);
    assert(p242_approach_y_GET(pack) == (float) -2.4220353E38F);
    assert(p242_approach_z_GET(pack) == (float)3.365042E38F);
    assert(p242_y_GET(pack) == (float)9.027582E37F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p243_approach_y_GET(pack) == (float) -1.5507856E38F);
    assert(p243_z_GET(pack) == (float)3.0712452E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)9191673220426437305L);
    assert(p243_altitude_GET(pack) == (int32_t) -130085780);
    assert(p243_longitude_GET(pack) == (int32_t) -2095471755);
    assert(p243_approach_x_GET(pack) == (float) -2.0329438E38F);
    assert(p243_latitude_GET(pack) == (int32_t)445609559);
    assert(p243_y_GET(pack) == (float)2.4786137E38F);
    assert(p243_x_GET(pack) == (float) -3.0400724E38F);
    {
        float exemplary[] =  {-1.5019237E37F, -3.076273E38F, 3.1220257E38F, -2.2700622E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_z_GET(pack) == (float) -1.9058167E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)34025);
    assert(p244_interval_us_GET(pack) == (int32_t) -1942720437);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)57926);
    assert(p246_lat_GET(pack) == (int32_t) -519202483);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)61073);
    assert(p246_altitude_GET(pack) == (int32_t) -684295046);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3777895889L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED));
    assert(p246_lon_GET(pack) == (int32_t) -801970908);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)2345);
    assert(p246_callsign_LEN(ph) == 3);
    {
        char16_t * exemplary = u"qai";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)50709);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)3.0938771E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -2.391154E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_id_GET(pack) == (uint32_t)3992920116L);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -1.2880387E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)49486);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)212);
    {
        uint8_t exemplary[] =  {(uint8_t)203, (uint8_t)233, (uint8_t)41, (uint8_t)66, (uint8_t)232, (uint8_t)255, (uint8_t)192, (uint8_t)239, (uint8_t)182, (uint8_t)173, (uint8_t)139, (uint8_t)190, (uint8_t)218, (uint8_t)26, (uint8_t)12, (uint8_t)239, (uint8_t)222, (uint8_t)7, (uint8_t)133, (uint8_t)111, (uint8_t)107, (uint8_t)121, (uint8_t)57, (uint8_t)155, (uint8_t)91, (uint8_t)15, (uint8_t)77, (uint8_t)50, (uint8_t)197, (uint8_t)83, (uint8_t)202, (uint8_t)216, (uint8_t)239, (uint8_t)124, (uint8_t)152, (uint8_t)230, (uint8_t)214, (uint8_t)107, (uint8_t)86, (uint8_t)71, (uint8_t)143, (uint8_t)220, (uint8_t)64, (uint8_t)33, (uint8_t)140, (uint8_t)141, (uint8_t)202, (uint8_t)178, (uint8_t)48, (uint8_t)4, (uint8_t)59, (uint8_t)129, (uint8_t)39, (uint8_t)88, (uint8_t)36, (uint8_t)197, (uint8_t)170, (uint8_t)175, (uint8_t)188, (uint8_t)53, (uint8_t)206, (uint8_t)145, (uint8_t)250, (uint8_t)18, (uint8_t)57, (uint8_t)133, (uint8_t)18, (uint8_t)73, (uint8_t)241, (uint8_t)129, (uint8_t)191, (uint8_t)220, (uint8_t)38, (uint8_t)56, (uint8_t)101, (uint8_t)186, (uint8_t)96, (uint8_t)148, (uint8_t)160, (uint8_t)84, (uint8_t)154, (uint8_t)119, (uint8_t)53, (uint8_t)240, (uint8_t)74, (uint8_t)177, (uint8_t)133, (uint8_t)15, (uint8_t)252, (uint8_t)18, (uint8_t)228, (uint8_t)199, (uint8_t)21, (uint8_t)128, (uint8_t)100, (uint8_t)174, (uint8_t)93, (uint8_t)21, (uint8_t)137, (uint8_t)201, (uint8_t)0, (uint8_t)136, (uint8_t)197, (uint8_t)37, (uint8_t)211, (uint8_t)90, (uint8_t)40, (uint8_t)176, (uint8_t)92, (uint8_t)1, (uint8_t)87, (uint8_t)130, (uint8_t)150, (uint8_t)41, (uint8_t)114, (uint8_t)128, (uint8_t)218, (uint8_t)198, (uint8_t)26, (uint8_t)67, (uint8_t)30, (uint8_t)0, (uint8_t)22, (uint8_t)139, (uint8_t)229, (uint8_t)143, (uint8_t)160, (uint8_t)205, (uint8_t)218, (uint8_t)23, (uint8_t)32, (uint8_t)234, (uint8_t)115, (uint8_t)26, (uint8_t)181, (uint8_t)129, (uint8_t)190, (uint8_t)69, (uint8_t)14, (uint8_t)228, (uint8_t)76, (uint8_t)110, (uint8_t)239, (uint8_t)90, (uint8_t)198, (uint8_t)235, (uint8_t)42, (uint8_t)245, (uint8_t)4, (uint8_t)234, (uint8_t)94, (uint8_t)39, (uint8_t)168, (uint8_t)91, (uint8_t)201, (uint8_t)208, (uint8_t)120, (uint8_t)180, (uint8_t)100, (uint8_t)159, (uint8_t)185, (uint8_t)92, (uint8_t)1, (uint8_t)117, (uint8_t)83, (uint8_t)187, (uint8_t)88, (uint8_t)172, (uint8_t)139, (uint8_t)66, (uint8_t)112, (uint8_t)227, (uint8_t)215, (uint8_t)41, (uint8_t)55, (uint8_t)121, (uint8_t)246, (uint8_t)123, (uint8_t)233, (uint8_t)219, (uint8_t)21, (uint8_t)93, (uint8_t)190, (uint8_t)113, (uint8_t)147, (uint8_t)120, (uint8_t)180, (uint8_t)233, (uint8_t)217, (uint8_t)114, (uint8_t)116, (uint8_t)222, (uint8_t)185, (uint8_t)91, (uint8_t)119, (uint8_t)110, (uint8_t)113, (uint8_t)226, (uint8_t)212, (uint8_t)232, (uint8_t)205, (uint8_t)132, (uint8_t)162, (uint8_t)34, (uint8_t)191, (uint8_t)230, (uint8_t)146, (uint8_t)2, (uint8_t)133, (uint8_t)72, (uint8_t)142, (uint8_t)148, (uint8_t)196, (uint8_t)248, (uint8_t)192, (uint8_t)255, (uint8_t)15, (uint8_t)72, (uint8_t)255, (uint8_t)108, (uint8_t)46, (uint8_t)108, (uint8_t)135, (uint8_t)57, (uint8_t)52, (uint8_t)37, (uint8_t)164, (uint8_t)114, (uint8_t)37, (uint8_t)25, (uint8_t)200, (uint8_t)57, (uint8_t)67, (uint8_t)247, (uint8_t)93, (uint8_t)104, (uint8_t)226, (uint8_t)69, (uint8_t)171, (uint8_t)250, (uint8_t)77, (uint8_t)48, (uint8_t)140, (uint8_t)190, (uint8_t)0, (uint8_t)91, (uint8_t)229, (uint8_t)8, (uint8_t)168} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)26);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)158);
    {
        int8_t exemplary[] =  {(int8_t) -9, (int8_t)19, (int8_t)22, (int8_t)100, (int8_t) -43, (int8_t) -35, (int8_t) -79, (int8_t) -107, (int8_t)46, (int8_t) -9, (int8_t)50, (int8_t) -127, (int8_t)67, (int8_t) -99, (int8_t)22, (int8_t) -72, (int8_t) -80, (int8_t) -2, (int8_t)1, (int8_t) -2, (int8_t) -89, (int8_t)84, (int8_t) -14, (int8_t)36, (int8_t) -54, (int8_t) -84, (int8_t)34, (int8_t) -123, (int8_t)71, (int8_t) -31, (int8_t)114, (int8_t)74} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)56142);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"hQ";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)7688063479593977055L);
    assert(p250_y_GET(pack) == (float)3.1985388E38F);
    assert(p250_x_GET(pack) == (float) -3.0901707E38F);
    assert(p250_z_GET(pack) == (float)2.6637484E37F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)1657850067L);
    assert(p251_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"lplsbsneo";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float)1.1333921E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"siac";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3827153775L);
    assert(p252_value_GET(pack) == (int32_t) -295342900);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 39);
    {
        char16_t * exemplary = u"ecavpprokasxemzajqljhijrlkuvwfdhniczjtd";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 78);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_INFO);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)7.3485016E37F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)4257574236L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)121, (uint8_t)1, (uint8_t)103, (uint8_t)166, (uint8_t)32, (uint8_t)94, (uint8_t)28, (uint8_t)145, (uint8_t)126, (uint8_t)139, (uint8_t)89, (uint8_t)180, (uint8_t)34, (uint8_t)90, (uint8_t)96, (uint8_t)204, (uint8_t)66, (uint8_t)18, (uint8_t)150, (uint8_t)64, (uint8_t)25, (uint8_t)209, (uint8_t)70, (uint8_t)91, (uint8_t)58, (uint8_t)156, (uint8_t)251, (uint8_t)163, (uint8_t)83, (uint8_t)27, (uint8_t)21, (uint8_t)12} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)2788485417723730785L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)179);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1173199757L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)323105698L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)82);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p258_tune_LEN(ph) == 28);
    {
        char16_t * exemplary = u"xkJWrtdabhbdzozwszsjdIjvqhcv";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)197);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_focal_length_GET(pack) == (float)1.9191382E38F);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1520538778L);
    {
        uint8_t exemplary[] =  {(uint8_t)193, (uint8_t)47, (uint8_t)87, (uint8_t)220, (uint8_t)152, (uint8_t)126, (uint8_t)98, (uint8_t)100, (uint8_t)251, (uint8_t)133, (uint8_t)121, (uint8_t)167, (uint8_t)171, (uint8_t)69, (uint8_t)170, (uint8_t)160, (uint8_t)74, (uint8_t)50, (uint8_t)118, (uint8_t)229, (uint8_t)7, (uint8_t)129, (uint8_t)71, (uint8_t)26, (uint8_t)136, (uint8_t)199, (uint8_t)88, (uint8_t)33, (uint8_t)191, (uint8_t)1, (uint8_t)88, (uint8_t)129} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)1, (uint8_t)194, (uint8_t)150, (uint8_t)193, (uint8_t)237, (uint8_t)168, (uint8_t)192, (uint8_t)76, (uint8_t)201, (uint8_t)186, (uint8_t)228, (uint8_t)20, (uint8_t)80, (uint8_t)173, (uint8_t)63, (uint8_t)160, (uint8_t)115, (uint8_t)153, (uint8_t)61, (uint8_t)138, (uint8_t)205, (uint8_t)228, (uint8_t)149, (uint8_t)150, (uint8_t)236, (uint8_t)225, (uint8_t)55, (uint8_t)227, (uint8_t)211, (uint8_t)28, (uint8_t)197, (uint8_t)108} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3985773577L);
    assert(p259_cam_definition_uri_LEN(ph) == 14);
    {
        char16_t * exemplary = u"KelyatfaofWXid";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float)7.7722263E36F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)31462);
    assert(p259_sensor_size_h_GET(pack) == (float)2.4055551E37F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)4156);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)3508);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)402655530L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_used_capacity_GET(pack) == (float) -1.0662128E38F);
    assert(p261_read_speed_GET(pack) == (float) -6.154576E36F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1973718645L);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p261_available_capacity_GET(pack) == (float) -2.689514E38F);
    assert(p261_total_capacity_GET(pack) == (float) -3.4016152E38F);
    assert(p261_write_speed_GET(pack) == (float)1.8458352E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)12);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)4179335354L);
    assert(p262_image_interval_GET(pack) == (float)1.3469536E38F);
    assert(p262_available_capacity_GET(pack) == (float)2.475028E38F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2249165182L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)55);
    assert(p263_alt_GET(pack) == (int32_t) -22072838);
    assert(p263_image_index_GET(pack) == (int32_t) -478551698);
    {
        float exemplary[] =  {-6.0729205E37F, -1.7968335E38F, 2.6211556E38F, -7.389404E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t)434045990);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p263_lat_GET(pack) == (int32_t) -1923781458);
    assert(p263_lon_GET(pack) == (int32_t)467061498);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1043770951L);
    assert(p263_file_url_LEN(ph) == 165);
    {
        char16_t * exemplary = u"qkhrapsQufxTgsakspfanismomtjfeurybJpLuEafmwdnPryovjpovZoiyGtoczoykujdcvsnodsrfcpdbpvrglEPhwhjvRXdimrdrcgprmtttixPvactoddtkmQssVhcemrkymmdThkseHSmokiyvjznsmcVtkiQoimc";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 330);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_utc_GET(pack) == (uint64_t)319974525447732436L);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)4823804155354855948L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2845336579L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)9103901080937652980L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)329063277525439225L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1688149300L);
    assert(p265_yaw_GET(pack) == (float) -4.9692517E37F);
    assert(p265_pitch_GET(pack) == (float)1.949838E38F);
    assert(p265_roll_GET(pack) == (float)1.9474097E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)107);
    {
        uint8_t exemplary[] =  {(uint8_t)190, (uint8_t)147, (uint8_t)94, (uint8_t)150, (uint8_t)47, (uint8_t)203, (uint8_t)22, (uint8_t)223, (uint8_t)170, (uint8_t)154, (uint8_t)9, (uint8_t)226, (uint8_t)137, (uint8_t)170, (uint8_t)90, (uint8_t)224, (uint8_t)157, (uint8_t)141, (uint8_t)66, (uint8_t)135, (uint8_t)206, (uint8_t)164, (uint8_t)139, (uint8_t)244, (uint8_t)157, (uint8_t)213, (uint8_t)59, (uint8_t)220, (uint8_t)82, (uint8_t)54, (uint8_t)214, (uint8_t)237, (uint8_t)60, (uint8_t)196, (uint8_t)202, (uint8_t)204, (uint8_t)205, (uint8_t)163, (uint8_t)55, (uint8_t)187, (uint8_t)130, (uint8_t)193, (uint8_t)121, (uint8_t)3, (uint8_t)195, (uint8_t)164, (uint8_t)128, (uint8_t)232, (uint8_t)96, (uint8_t)241, (uint8_t)203, (uint8_t)58, (uint8_t)148, (uint8_t)207, (uint8_t)66, (uint8_t)156, (uint8_t)150, (uint8_t)136, (uint8_t)230, (uint8_t)170, (uint8_t)134, (uint8_t)150, (uint8_t)130, (uint8_t)163, (uint8_t)3, (uint8_t)181, (uint8_t)148, (uint8_t)90, (uint8_t)223, (uint8_t)133, (uint8_t)57, (uint8_t)206, (uint8_t)16, (uint8_t)12, (uint8_t)109, (uint8_t)18, (uint8_t)79, (uint8_t)89, (uint8_t)16, (uint8_t)137, (uint8_t)86, (uint8_t)169, (uint8_t)21, (uint8_t)50, (uint8_t)43, (uint8_t)211, (uint8_t)179, (uint8_t)12, (uint8_t)22, (uint8_t)171, (uint8_t)126, (uint8_t)111, (uint8_t)126, (uint8_t)162, (uint8_t)45, (uint8_t)74, (uint8_t)109, (uint8_t)46, (uint8_t)37, (uint8_t)149, (uint8_t)43, (uint8_t)125, (uint8_t)153, (uint8_t)23, (uint8_t)198, (uint8_t)80, (uint8_t)214, (uint8_t)136, (uint8_t)220, (uint8_t)50, (uint8_t)151, (uint8_t)190, (uint8_t)172, (uint8_t)164, (uint8_t)169, (uint8_t)187, (uint8_t)30, (uint8_t)236, (uint8_t)88, (uint8_t)110, (uint8_t)60, (uint8_t)120, (uint8_t)5, (uint8_t)26, (uint8_t)134, (uint8_t)209, (uint8_t)82, (uint8_t)31, (uint8_t)236, (uint8_t)220, (uint8_t)192, (uint8_t)62, (uint8_t)80, (uint8_t)135, (uint8_t)240, (uint8_t)117, (uint8_t)166, (uint8_t)234, (uint8_t)13, (uint8_t)142, (uint8_t)197, (uint8_t)71, (uint8_t)5, (uint8_t)2, (uint8_t)46, (uint8_t)116, (uint8_t)114, (uint8_t)8, (uint8_t)246, (uint8_t)111, (uint8_t)160, (uint8_t)247, (uint8_t)34, (uint8_t)102, (uint8_t)10, (uint8_t)234, (uint8_t)53, (uint8_t)98, (uint8_t)176, (uint8_t)54, (uint8_t)55, (uint8_t)138, (uint8_t)20, (uint8_t)68, (uint8_t)56, (uint8_t)3, (uint8_t)218, (uint8_t)35, (uint8_t)26, (uint8_t)203, (uint8_t)71, (uint8_t)162, (uint8_t)213, (uint8_t)190, (uint8_t)252, (uint8_t)77, (uint8_t)187, (uint8_t)1, (uint8_t)1, (uint8_t)52, (uint8_t)94, (uint8_t)200, (uint8_t)240, (uint8_t)232, (uint8_t)220, (uint8_t)167, (uint8_t)35, (uint8_t)163, (uint8_t)231, (uint8_t)0, (uint8_t)74, (uint8_t)202, (uint8_t)178, (uint8_t)238, (uint8_t)53, (uint8_t)25, (uint8_t)119, (uint8_t)60, (uint8_t)88, (uint8_t)159, (uint8_t)196, (uint8_t)179, (uint8_t)6, (uint8_t)23, (uint8_t)86, (uint8_t)163, (uint8_t)137, (uint8_t)103, (uint8_t)191, (uint8_t)254, (uint8_t)201, (uint8_t)54, (uint8_t)202, (uint8_t)218, (uint8_t)57, (uint8_t)2, (uint8_t)24, (uint8_t)225, (uint8_t)206, (uint8_t)190, (uint8_t)160, (uint8_t)100, (uint8_t)182, (uint8_t)220, (uint8_t)211, (uint8_t)172, (uint8_t)30, (uint8_t)237, (uint8_t)13, (uint8_t)54, (uint8_t)192, (uint8_t)101, (uint8_t)195, (uint8_t)238, (uint8_t)150, (uint8_t)41, (uint8_t)12, (uint8_t)65, (uint8_t)10, (uint8_t)113, (uint8_t)214, (uint8_t)190, (uint8_t)110, (uint8_t)7, (uint8_t)210, (uint8_t)193, (uint8_t)27, (uint8_t)92, (uint8_t)153} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)37462);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)198);
    {
        uint8_t exemplary[] =  {(uint8_t)79, (uint8_t)94, (uint8_t)201, (uint8_t)36, (uint8_t)228, (uint8_t)247, (uint8_t)122, (uint8_t)189, (uint8_t)233, (uint8_t)223, (uint8_t)140, (uint8_t)217, (uint8_t)72, (uint8_t)168, (uint8_t)98, (uint8_t)245, (uint8_t)159, (uint8_t)98, (uint8_t)128, (uint8_t)235, (uint8_t)54, (uint8_t)19, (uint8_t)44, (uint8_t)202, (uint8_t)236, (uint8_t)4, (uint8_t)90, (uint8_t)183, (uint8_t)70, (uint8_t)84, (uint8_t)87, (uint8_t)189, (uint8_t)254, (uint8_t)13, (uint8_t)195, (uint8_t)152, (uint8_t)119, (uint8_t)141, (uint8_t)30, (uint8_t)68, (uint8_t)18, (uint8_t)192, (uint8_t)162, (uint8_t)156, (uint8_t)23, (uint8_t)187, (uint8_t)73, (uint8_t)44, (uint8_t)181, (uint8_t)91, (uint8_t)223, (uint8_t)5, (uint8_t)60, (uint8_t)124, (uint8_t)77, (uint8_t)234, (uint8_t)131, (uint8_t)178, (uint8_t)82, (uint8_t)212, (uint8_t)105, (uint8_t)95, (uint8_t)219, (uint8_t)53, (uint8_t)100, (uint8_t)141, (uint8_t)62, (uint8_t)119, (uint8_t)41, (uint8_t)84, (uint8_t)66, (uint8_t)220, (uint8_t)62, (uint8_t)168, (uint8_t)49, (uint8_t)125, (uint8_t)219, (uint8_t)246, (uint8_t)213, (uint8_t)133, (uint8_t)67, (uint8_t)231, (uint8_t)90, (uint8_t)250, (uint8_t)159, (uint8_t)239, (uint8_t)97, (uint8_t)104, (uint8_t)78, (uint8_t)143, (uint8_t)86, (uint8_t)118, (uint8_t)82, (uint8_t)241, (uint8_t)37, (uint8_t)161, (uint8_t)38, (uint8_t)139, (uint8_t)42, (uint8_t)201, (uint8_t)100, (uint8_t)3, (uint8_t)253, (uint8_t)23, (uint8_t)124, (uint8_t)255, (uint8_t)186, (uint8_t)104, (uint8_t)136, (uint8_t)92, (uint8_t)21, (uint8_t)128, (uint8_t)66, (uint8_t)189, (uint8_t)72, (uint8_t)80, (uint8_t)78, (uint8_t)9, (uint8_t)225, (uint8_t)77, (uint8_t)196, (uint8_t)140, (uint8_t)107, (uint8_t)61, (uint8_t)94, (uint8_t)206, (uint8_t)254, (uint8_t)44, (uint8_t)71, (uint8_t)228, (uint8_t)193, (uint8_t)149, (uint8_t)59, (uint8_t)172, (uint8_t)25, (uint8_t)27, (uint8_t)43, (uint8_t)115, (uint8_t)213, (uint8_t)131, (uint8_t)44, (uint8_t)215, (uint8_t)24, (uint8_t)152, (uint8_t)0, (uint8_t)249, (uint8_t)78, (uint8_t)251, (uint8_t)24, (uint8_t)23, (uint8_t)15, (uint8_t)170, (uint8_t)254, (uint8_t)32, (uint8_t)122, (uint8_t)77, (uint8_t)68, (uint8_t)55, (uint8_t)165, (uint8_t)38, (uint8_t)34, (uint8_t)224, (uint8_t)116, (uint8_t)55, (uint8_t)144, (uint8_t)140, (uint8_t)28, (uint8_t)70, (uint8_t)208, (uint8_t)27, (uint8_t)161, (uint8_t)233, (uint8_t)199, (uint8_t)54, (uint8_t)212, (uint8_t)189, (uint8_t)224, (uint8_t)222, (uint8_t)204, (uint8_t)245, (uint8_t)234, (uint8_t)156, (uint8_t)115, (uint8_t)109, (uint8_t)137, (uint8_t)176, (uint8_t)168, (uint8_t)82, (uint8_t)10, (uint8_t)171, (uint8_t)11, (uint8_t)215, (uint8_t)194, (uint8_t)224, (uint8_t)62, (uint8_t)178, (uint8_t)27, (uint8_t)232, (uint8_t)161, (uint8_t)97, (uint8_t)145, (uint8_t)2, (uint8_t)39, (uint8_t)214, (uint8_t)12, (uint8_t)124, (uint8_t)255, (uint8_t)183, (uint8_t)127, (uint8_t)60, (uint8_t)38, (uint8_t)29, (uint8_t)107, (uint8_t)33, (uint8_t)71, (uint8_t)2, (uint8_t)19, (uint8_t)174, (uint8_t)19, (uint8_t)102, (uint8_t)11, (uint8_t)98, (uint8_t)106, (uint8_t)58, (uint8_t)54, (uint8_t)75, (uint8_t)28, (uint8_t)121, (uint8_t)144, (uint8_t)156, (uint8_t)78, (uint8_t)56, (uint8_t)100, (uint8_t)104, (uint8_t)173, (uint8_t)227, (uint8_t)29, (uint8_t)176, (uint8_t)198, (uint8_t)8, (uint8_t)117, (uint8_t)0, (uint8_t)98, (uint8_t)200, (uint8_t)26, (uint8_t)57, (uint8_t)209, (uint8_t)247, (uint8_t)228} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)22846);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)106);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)60317);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)194);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)6829);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p269_uri_LEN(ph) == 50);
    {
        char16_t * exemplary = u"JqycwjatydqhjmmuOqpBqavphjtpnlnbzOpyvvujuxwksqdumj";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 100);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float)3.0073495E38F);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)12961);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)34521);
    assert(p269_bitrate_GET(pack) == (uint32_t)2781234573L);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)26190);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)26029);
    assert(p270_uri_LEN(ph) == 77);
    {
        char16_t * exemplary = u"uwulrpwpftfogytyoeSrMwWkkfwimfhszvmssghbeVkvngfjhnoecabvpitewdFlkOxdnibmvyodx";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 154);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p270_framerate_GET(pack) == (float) -2.927514E38F);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)754);
    assert(p270_bitrate_GET(pack) == (uint32_t)3468219179L);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 30);
    {
        char16_t * exemplary = u"jKlknzbazljcnCzjzDowawGnybbzvb";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 9);
    {
        char16_t * exemplary = u"zwjkqigje";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)10176);
    {
        uint8_t exemplary[] =  {(uint8_t)63, (uint8_t)2, (uint8_t)69, (uint8_t)186, (uint8_t)83, (uint8_t)97, (uint8_t)88, (uint8_t)179} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)48404);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)155, (uint8_t)168, (uint8_t)149, (uint8_t)132, (uint8_t)181, (uint8_t)243, (uint8_t)171} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)8698);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p310_time_usec_GET(pack) == (uint64_t)6425747654001899098L);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)3179375016L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)39174);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_uptime_sec_GET(pack) == (uint32_t)3026369564L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)246);
    {
        uint8_t exemplary[] =  {(uint8_t)228, (uint8_t)87, (uint8_t)159, (uint8_t)81, (uint8_t)181, (uint8_t)125, (uint8_t)169, (uint8_t)90, (uint8_t)153, (uint8_t)166, (uint8_t)134, (uint8_t)26, (uint8_t)30, (uint8_t)206, (uint8_t)112, (uint8_t)137} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3470672937L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p311_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"e";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)5576826338931582641L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)187);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -12556);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p320_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"yjolcQwuwehqhzm";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)226);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)42812);
    assert(p322_param_value_LEN(ph) == 59);
    {
        char16_t * exemplary = u"WrxfKwtidsejccadcufgeuzaazwicdrwmcftcVyisaebgjtnxonulymvhYv";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)19394);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p322_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"pLScxtymfGcqzrog";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p323_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"uChjljRisbgdaxa";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p323_param_value_LEN(ph) == 68);
    {
        char16_t * exemplary = u"ihbTqflvkJjUhjdCxesfkysfgmqyymuUxddIlqkzfrxFlpcnwRlsckmlHvcuthvianVg";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 136);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"cjnvdkjliJUfu";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8);
    assert(p324_param_value_LEN(ph) == 125);
    {
        char16_t * exemplary = u"NuuKHnfglkrdcRgqaGWlccflvhzwxhmhqkuMfqfdQosqjtommmqgmsjglkhYxvrxzchuLofoigvaaoapxhdpnmjlqycljTlnoaxbvnrdwGnmkpgIbkRrvdtwwifxi";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 250);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)39796);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)37679);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p330_time_usec_GET(pack) == (uint64_t)9192465739291422823L);
    {
        uint16_t exemplary[] =  {(uint16_t)15613, (uint16_t)15202, (uint16_t)62238, (uint16_t)7975, (uint16_t)20051, (uint16_t)11230, (uint16_t)10182, (uint16_t)26394, (uint16_t)20684, (uint16_t)51889, (uint16_t)36591, (uint16_t)51880, (uint16_t)22657, (uint16_t)64294, (uint16_t)29696, (uint16_t)49426, (uint16_t)44360, (uint16_t)12899, (uint16_t)65106, (uint16_t)21066, (uint16_t)4291, (uint16_t)46623, (uint16_t)32439, (uint16_t)62127, (uint16_t)23289, (uint16_t)54374, (uint16_t)26111, (uint16_t)28039, (uint16_t)58131, (uint16_t)28081, (uint16_t)33708, (uint16_t)676, (uint16_t)32240, (uint16_t)26951, (uint16_t)12467, (uint16_t)19316, (uint16_t)37183, (uint16_t)42196, (uint16_t)42496, (uint16_t)50110, (uint16_t)34139, (uint16_t)14967, (uint16_t)41809, (uint16_t)19676, (uint16_t)60771, (uint16_t)6909, (uint16_t)40560, (uint16_t)35070, (uint16_t)33799, (uint16_t)41008, (uint16_t)6829, (uint16_t)8821, (uint16_t)29265, (uint16_t)46470, (uint16_t)7380, (uint16_t)58268, (uint16_t)46147, (uint16_t)22927, (uint16_t)26071, (uint16_t)42500, (uint16_t)19571, (uint16_t)17443, (uint16_t)13407, (uint16_t)39583, (uint16_t)17920, (uint16_t)23665, (uint16_t)1340, (uint16_t)54468, (uint16_t)11323, (uint16_t)8394, (uint16_t)34890, (uint16_t)63208} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    assert(p10001_gpsOffsetLon_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
    assert(p10001_ICAO_GET(pack) == (uint32_t)1163262119L);
    assert(p10001_callsign_LEN(ph) == 2);
    {
        char16_t * exemplary = u"gt";
        char16_t * sample = p10001_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p10001_rfSelect_GET(pack) == e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED);
    assert(p10001_stallSpeed_GET(pack) == (uint16_t)(uint16_t)646);
    assert(p10001_gpsOffsetLat_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA);
    assert(p10001_emitterType_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE);
    assert(p10001_aircraftSize_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    assert(p10002_numSats_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p10002_baroAltMSL_GET(pack) == (int32_t) -1954073645);
    assert(p10002_utcTime_GET(pack) == (uint32_t)4007653121L);
    assert(p10002_gpsFix_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D);
    assert(p10002_accuracyHor_GET(pack) == (uint32_t)907226666L);
    assert(p10002_squawk_GET(pack) == (uint16_t)(uint16_t)4755);
    assert(p10002_VelEW_GET(pack) == (int16_t)(int16_t) -1301);
    assert(p10002_gpsLon_GET(pack) == (int32_t) -47792062);
    assert(p10002_gpsAlt_GET(pack) == (int32_t)1733001153);
    assert(p10002_emergencyStatus_GET(pack) == e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY);
    assert(p10002_accuracyVert_GET(pack) == (uint16_t)(uint16_t)49387);
    assert(p10002_velVert_GET(pack) == (int16_t)(int16_t)10302);
    assert(p10002_accuracyVel_GET(pack) == (uint16_t)(uint16_t)8796);
    assert(p10002_gpsLat_GET(pack) == (int32_t) -1602972662);
    assert(p10002_velNS_GET(pack) == (int16_t)(int16_t)886);
    assert(p10002_state_GET(pack) == (e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED |
                                      e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE));
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    assert(p10003_rfHealth_GET(pack) == e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    assert(p11000_target_component_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p11000_bus_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p11000_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11000_count_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p11000_target_system_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p11000_address_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p11000_busname_LEN(ph) == 38);
    {
        char16_t * exemplary = u"exKqaGdmLJordrtxUusxSeuwhecZtgyfjwbtmm";
        char16_t * sample = p11000_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 76);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11000_regstart_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p11000_request_id_GET(pack) == (uint32_t)2422808142L);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    assert(p11001_request_id_GET(pack) == (uint32_t)1531350357L);
    assert(p11001_regstart_GET(pack) == (uint8_t)(uint8_t)154);
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)22, (uint8_t)99, (uint8_t)128, (uint8_t)255, (uint8_t)223, (uint8_t)148, (uint8_t)124, (uint8_t)129, (uint8_t)64, (uint8_t)70, (uint8_t)226, (uint8_t)137, (uint8_t)165, (uint8_t)219, (uint8_t)134, (uint8_t)0, (uint8_t)67, (uint8_t)123, (uint8_t)233, (uint8_t)188, (uint8_t)89, (uint8_t)113, (uint8_t)69, (uint8_t)85, (uint8_t)177, (uint8_t)39, (uint8_t)2, (uint8_t)161, (uint8_t)107, (uint8_t)137, (uint8_t)76, (uint8_t)35, (uint8_t)241, (uint8_t)127, (uint8_t)34, (uint8_t)157, (uint8_t)205, (uint8_t)10, (uint8_t)71, (uint8_t)141, (uint8_t)242, (uint8_t)12, (uint8_t)71, (uint8_t)47, (uint8_t)184, (uint8_t)230, (uint8_t)7, (uint8_t)176, (uint8_t)182, (uint8_t)10, (uint8_t)234, (uint8_t)72, (uint8_t)78, (uint8_t)239, (uint8_t)36, (uint8_t)99, (uint8_t)216, (uint8_t)228, (uint8_t)145, (uint8_t)144, (uint8_t)237, (uint8_t)86, (uint8_t)198, (uint8_t)15, (uint8_t)29, (uint8_t)145, (uint8_t)122, (uint8_t)245, (uint8_t)65, (uint8_t)134, (uint8_t)110, (uint8_t)249, (uint8_t)42, (uint8_t)107, (uint8_t)56, (uint8_t)251, (uint8_t)180, (uint8_t)171, (uint8_t)88, (uint8_t)103, (uint8_t)201, (uint8_t)107, (uint8_t)44, (uint8_t)70, (uint8_t)151, (uint8_t)18, (uint8_t)221, (uint8_t)150, (uint8_t)163, (uint8_t)120, (uint8_t)111, (uint8_t)182, (uint8_t)252, (uint8_t)30, (uint8_t)90, (uint8_t)86, (uint8_t)175, (uint8_t)110, (uint8_t)5, (uint8_t)94, (uint8_t)112, (uint8_t)12, (uint8_t)251, (uint8_t)94, (uint8_t)109, (uint8_t)17, (uint8_t)184, (uint8_t)132, (uint8_t)65, (uint8_t)23, (uint8_t)158, (uint8_t)153, (uint8_t)13, (uint8_t)21, (uint8_t)148, (uint8_t)90, (uint8_t)24, (uint8_t)192, (uint8_t)91, (uint8_t)88, (uint8_t)98, (uint8_t)69, (uint8_t)209, (uint8_t)189, (uint8_t)166, (uint8_t)49, (uint8_t)112} ;
        uint8_t*  sample = p11001_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11001_result_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p11001_count_GET(pack) == (uint8_t)(uint8_t)187);
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    assert(p11002_target_system_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p11002_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C);
    assert(p11002_bus_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p11002_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p11002_busname_LEN(ph) == 13);
    {
        char16_t * exemplary = u"NajoNegbyvtyg";
        char16_t * sample = p11002_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_address_GET(pack) == (uint8_t)(uint8_t)35);
    {
        uint8_t exemplary[] =  {(uint8_t)116, (uint8_t)40, (uint8_t)173, (uint8_t)155, (uint8_t)36, (uint8_t)31, (uint8_t)197, (uint8_t)214, (uint8_t)146, (uint8_t)108, (uint8_t)199, (uint8_t)68, (uint8_t)139, (uint8_t)252, (uint8_t)239, (uint8_t)167, (uint8_t)40, (uint8_t)61, (uint8_t)158, (uint8_t)46, (uint8_t)7, (uint8_t)59, (uint8_t)162, (uint8_t)2, (uint8_t)81, (uint8_t)12, (uint8_t)119, (uint8_t)12, (uint8_t)93, (uint8_t)189, (uint8_t)149, (uint8_t)124, (uint8_t)171, (uint8_t)169, (uint8_t)71, (uint8_t)6, (uint8_t)50, (uint8_t)44, (uint8_t)108, (uint8_t)64, (uint8_t)170, (uint8_t)203, (uint8_t)165, (uint8_t)27, (uint8_t)140, (uint8_t)225, (uint8_t)90, (uint8_t)210, (uint8_t)226, (uint8_t)105, (uint8_t)19, (uint8_t)11, (uint8_t)238, (uint8_t)18, (uint8_t)91, (uint8_t)10, (uint8_t)61, (uint8_t)221, (uint8_t)31, (uint8_t)67, (uint8_t)75, (uint8_t)127, (uint8_t)140, (uint8_t)126, (uint8_t)38, (uint8_t)39, (uint8_t)237, (uint8_t)239, (uint8_t)69, (uint8_t)86, (uint8_t)131, (uint8_t)12, (uint8_t)156, (uint8_t)75, (uint8_t)146, (uint8_t)151, (uint8_t)203, (uint8_t)63, (uint8_t)231, (uint8_t)201, (uint8_t)175, (uint8_t)182, (uint8_t)75, (uint8_t)185, (uint8_t)211, (uint8_t)208, (uint8_t)134, (uint8_t)157, (uint8_t)152, (uint8_t)187, (uint8_t)108, (uint8_t)89, (uint8_t)28, (uint8_t)56, (uint8_t)116, (uint8_t)37, (uint8_t)47, (uint8_t)161, (uint8_t)165, (uint8_t)178, (uint8_t)244, (uint8_t)193, (uint8_t)13, (uint8_t)252, (uint8_t)134, (uint8_t)196, (uint8_t)159, (uint8_t)38, (uint8_t)239, (uint8_t)189, (uint8_t)20, (uint8_t)217, (uint8_t)21, (uint8_t)4, (uint8_t)203, (uint8_t)140, (uint8_t)18, (uint8_t)151, (uint8_t)223, (uint8_t)15, (uint8_t)172, (uint8_t)180, (uint8_t)227, (uint8_t)121, (uint8_t)78, (uint8_t)90, (uint8_t)71, (uint8_t)168} ;
        uint8_t*  sample = p11002_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_regstart_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p11002_request_id_GET(pack) == (uint32_t)4034523335L);
    assert(p11002_count_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    assert(p11003_result_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p11003_request_id_GET(pack) == (uint32_t)3842893079L);
};


void c_CommunicationChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    assert(p11010_sigma_GET(pack) == (float) -5.1889962E36F);
    assert(p11010_omega_dot_GET(pack) == (float)2.4804912E38F);
    assert(p11010_theta_GET(pack) == (float) -2.5446238E38F);
    assert(p11010_error_GET(pack) == (float) -3.2120997E38F);
    assert(p11010_u_GET(pack) == (float) -4.42136E37F);
    assert(p11010_theta_dot_GET(pack) == (float) -2.0725203E38F);
    assert(p11010_achieved_GET(pack) == (float) -1.330407E38F);
    assert(p11010_omega_GET(pack) == (float) -1.5845864E38F);
    assert(p11010_sigma_dot_GET(pack) == (float) -2.0337444E38F);
    assert(p11010_desired_GET(pack) == (float)3.8192728E37F);
    assert(p11010_f_dot_GET(pack) == (float)2.1747361E38F);
    assert(p11010_f_GET(pack) == (float) -1.6611957E38F);
    assert(p11010_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_YAW);
};


void c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    assert(p11011_time_delta_usec_GET(pack) == (uint64_t)4954082534835552085L);
    assert(p11011_time_usec_GET(pack) == (uint64_t)7184237831619573351L);
    {
        float exemplary[] =  {2.133346E38F, 2.9353287E38F, 3.0898836E37F} ;
        float*  sample = p11011_position_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.7128511E38F, 8.882023E37F, 9.645133E37F} ;
        float*  sample = p11011_angle_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11011_confidence_GET(pack) == (float)2.8492916E38F);
};



Pack * c_TEST_Channel_process(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id) /*220*/
        {
            case 0:
                if(pack == NULL) return c_CommunicationChannel_new_HEARTBEAT_0();
                c_TEST_Channel_on_HEARTBEAT_0(&ph, pack);
                break;
            case 1:
                if(pack == NULL) return c_CommunicationChannel_new_SYS_STATUS_1();
                c_TEST_Channel_on_SYS_STATUS_1(&ph, pack);
                break;
            case 2:
                if(pack == NULL) return c_CommunicationChannel_new_SYSTEM_TIME_2();
                c_TEST_Channel_on_SYSTEM_TIME_2(&ph, pack);
                break;
            case 3:
                if(pack == NULL) return c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3();
                c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&ph, pack);
                break;
            case 4:
                if(pack == NULL) return c_CommunicationChannel_new_PING_4();
                c_TEST_Channel_on_PING_4(&ph, pack);
                break;
            case 5:
                if(pack == NULL) return c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5();
                c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&ph, pack);
                break;
            case 6:
                if(pack == NULL) return c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6();
                c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&ph, pack);
                break;
            case 7:
                if(pack == NULL) return c_CommunicationChannel_new_AUTH_KEY_7();
                c_TEST_Channel_on_AUTH_KEY_7(&ph, pack);
                break;
            case 11:
                if(pack == NULL) return c_CommunicationChannel_new_SET_MODE_11();
                c_TEST_Channel_on_SET_MODE_11(&ph, pack);
                break;
            case 20:
                if(pack == NULL) return c_CommunicationChannel_new_PARAM_REQUEST_READ_20();
                c_TEST_Channel_on_PARAM_REQUEST_READ_20(&ph, pack);
                break;
            case 21:
                if(pack == NULL) return c_CommunicationChannel_new_PARAM_REQUEST_LIST_21();
                c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&ph, pack);
                break;
            case 22:
                if(pack == NULL) return c_CommunicationChannel_new_PARAM_VALUE_22();
                c_TEST_Channel_on_PARAM_VALUE_22(&ph, pack);
                break;
            case 23:
                if(pack == NULL) return c_CommunicationChannel_new_PARAM_SET_23();
                c_TEST_Channel_on_PARAM_SET_23(&ph, pack);
                break;
            case 24:
                if(pack == NULL) return c_CommunicationChannel_new_GPS_RAW_INT_24();
                c_TEST_Channel_on_GPS_RAW_INT_24(&ph, pack);
                break;
            case 25:
                if(pack == NULL) return c_CommunicationChannel_new_GPS_STATUS_25();
                c_TEST_Channel_on_GPS_STATUS_25(&ph, pack);
                break;
            case 26:
                if(pack == NULL) return c_CommunicationChannel_new_SCALED_IMU_26();
                c_TEST_Channel_on_SCALED_IMU_26(&ph, pack);
                break;
            case 27:
                if(pack == NULL) return c_CommunicationChannel_new_RAW_IMU_27();
                c_TEST_Channel_on_RAW_IMU_27(&ph, pack);
                break;
            case 28:
                if(pack == NULL) return c_CommunicationChannel_new_RAW_PRESSURE_28();
                c_TEST_Channel_on_RAW_PRESSURE_28(&ph, pack);
                break;
            case 29:
                if(pack == NULL) return c_CommunicationChannel_new_SCALED_PRESSURE_29();
                c_TEST_Channel_on_SCALED_PRESSURE_29(&ph, pack);
                break;
            case 30:
                if(pack == NULL) return c_CommunicationChannel_new_ATTITUDE_30();
                c_TEST_Channel_on_ATTITUDE_30(&ph, pack);
                break;
            case 31:
                if(pack == NULL) return c_CommunicationChannel_new_ATTITUDE_QUATERNION_31();
                c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&ph, pack);
                break;
            case 32:
                if(pack == NULL) return c_CommunicationChannel_new_LOCAL_POSITION_NED_32();
                c_TEST_Channel_on_LOCAL_POSITION_NED_32(&ph, pack);
                break;
            case 33:
                if(pack == NULL) return c_CommunicationChannel_new_GLOBAL_POSITION_INT_33();
                c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&ph, pack);
                break;
            case 34:
                if(pack == NULL) return c_CommunicationChannel_new_RC_CHANNELS_SCALED_34();
                c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&ph, pack);
                break;
            case 35:
                if(pack == NULL) return c_CommunicationChannel_new_RC_CHANNELS_RAW_35();
                c_TEST_Channel_on_RC_CHANNELS_RAW_35(&ph, pack);
                break;
            case 36:
                if(pack == NULL) return c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36();
                c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&ph, pack);
                break;
            case 37:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37();
                c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&ph, pack);
                break;
            case 38:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38();
                c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&ph, pack);
                break;
            case 39:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_ITEM_39();
                c_TEST_Channel_on_MISSION_ITEM_39(&ph, pack);
                break;
            case 40:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_REQUEST_40();
                c_TEST_Channel_on_MISSION_REQUEST_40(&ph, pack);
                break;
            case 41:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_SET_CURRENT_41();
                c_TEST_Channel_on_MISSION_SET_CURRENT_41(&ph, pack);
                break;
            case 42:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_CURRENT_42();
                c_TEST_Channel_on_MISSION_CURRENT_42(&ph, pack);
                break;
            case 43:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_REQUEST_LIST_43();
                c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&ph, pack);
                break;
            case 44:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_COUNT_44();
                c_TEST_Channel_on_MISSION_COUNT_44(&ph, pack);
                break;
            case 45:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_CLEAR_ALL_45();
                c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&ph, pack);
                break;
            case 46:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_ITEM_REACHED_46();
                c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&ph, pack);
                break;
            case 47:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_ACK_47();
                c_TEST_Channel_on_MISSION_ACK_47(&ph, pack);
                break;
            case 48:
                if(pack == NULL) return c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48();
                c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&ph, pack);
                break;
            case 49:
                if(pack == NULL) return c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49();
                c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&ph, pack);
                break;
            case 50:
                if(pack == NULL) return c_CommunicationChannel_new_PARAM_MAP_RC_50();
                c_TEST_Channel_on_PARAM_MAP_RC_50(&ph, pack);
                break;
            case 51:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_REQUEST_INT_51();
                c_TEST_Channel_on_MISSION_REQUEST_INT_51(&ph, pack);
                break;
            case 54:
                if(pack == NULL) return c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54();
                c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&ph, pack);
                break;
            case 55:
                if(pack == NULL) return c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55();
                c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&ph, pack);
                break;
            case 61:
                if(pack == NULL) return c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61();
                c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&ph, pack);
                break;
            case 62:
                if(pack == NULL) return c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62();
                c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&ph, pack);
                break;
            case 63:
                if(pack == NULL) return c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63();
                c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&ph, pack);
                break;
            case 64:
                if(pack == NULL) return c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64();
                c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&ph, pack);
                break;
            case 65:
                if(pack == NULL) return c_CommunicationChannel_new_RC_CHANNELS_65();
                c_TEST_Channel_on_RC_CHANNELS_65(&ph, pack);
                break;
            case 66:
                if(pack == NULL) return c_CommunicationChannel_new_REQUEST_DATA_STREAM_66();
                c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&ph, pack);
                break;
            case 67:
                if(pack == NULL) return c_CommunicationChannel_new_DATA_STREAM_67();
                c_TEST_Channel_on_DATA_STREAM_67(&ph, pack);
                break;
            case 69:
                if(pack == NULL) return c_CommunicationChannel_new_MANUAL_CONTROL_69();
                c_TEST_Channel_on_MANUAL_CONTROL_69(&ph, pack);
                break;
            case 70:
                if(pack == NULL) return c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70();
                c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&ph, pack);
                break;
            case 73:
                if(pack == NULL) return c_CommunicationChannel_new_MISSION_ITEM_INT_73();
                c_TEST_Channel_on_MISSION_ITEM_INT_73(&ph, pack);
                break;
            case 74:
                if(pack == NULL) return c_CommunicationChannel_new_VFR_HUD_74();
                c_TEST_Channel_on_VFR_HUD_74(&ph, pack);
                break;
            case 75:
                if(pack == NULL) return c_CommunicationChannel_new_COMMAND_INT_75();
                c_TEST_Channel_on_COMMAND_INT_75(&ph, pack);
                break;
            case 76:
                if(pack == NULL) return c_CommunicationChannel_new_COMMAND_LONG_76();
                c_TEST_Channel_on_COMMAND_LONG_76(&ph, pack);
                break;
            case 77:
                if(pack == NULL) return c_CommunicationChannel_new_COMMAND_ACK_77();
                c_TEST_Channel_on_COMMAND_ACK_77(&ph, pack);
                break;
            case 81:
                if(pack == NULL) return c_CommunicationChannel_new_MANUAL_SETPOINT_81();
                c_TEST_Channel_on_MANUAL_SETPOINT_81(&ph, pack);
                break;
            case 82:
                if(pack == NULL) return c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82();
                c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&ph, pack);
                break;
            case 83:
                if(pack == NULL) return c_CommunicationChannel_new_ATTITUDE_TARGET_83();
                c_TEST_Channel_on_ATTITUDE_TARGET_83(&ph, pack);
                break;
            case 84:
                if(pack == NULL) return c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84();
                c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&ph, pack);
                break;
            case 86:
                if(pack == NULL) return c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86();
                c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&ph, pack);
                break;
            case 87:
                if(pack == NULL) return c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87();
                c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(&ph, pack);
                break;
            case 89:
                if(pack == NULL) return c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89();
                c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&ph, pack);
                break;
            case 90:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_STATE_90();
                c_TEST_Channel_on_HIL_STATE_90(&ph, pack);
                break;
            case 91:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_CONTROLS_91();
                c_TEST_Channel_on_HIL_CONTROLS_91(&ph, pack);
                break;
            case 92:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92();
                c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(&ph, pack);
                break;
            case 93:
                if(pack == NULL) return c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93();
                c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(&ph, pack);
                break;
            case 100:
                if(pack == NULL) return c_CommunicationChannel_new_OPTICAL_FLOW_100();
                c_TEST_Channel_on_OPTICAL_FLOW_100(&ph, pack);
                break;
            case 101:
                if(pack == NULL) return c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101();
                c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&ph, pack);
                break;
            case 102:
                if(pack == NULL) return c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102();
                c_TEST_Channel_on_VISION_POSITION_ESTIMATE_102(&ph, pack);
                break;
            default:
                assert(0);
                return NULL;
            case PROCESS_CHANNEL_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
                if(RBUF_ISFULL(received_packs)) return pack;
                RBUF_PUT(received_packs, pack)
                return NULL;
            case PROCESS_HOST_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(received_packs)) ? NULL : RBUF_GET(received_packs);
                if(RBUF_ISFULL(sendout_packs)) return pack;
                RBUF_PUT(sendout_packs, pack)
                return NULL;
            case PROCESS_RECEIVED_PACKS:
                LOOP = true;
                goto next_pack;
        }
        free(pack); // dispose pack
        if(!LOOP) return NULL;
next_pack:
        if(RBUF_ISEMPTY(received_packs)) return NULL;
        pack = RBUF_GET(received_packs);
        id = pack->meta->id;
        setPack(pack, &ph);
    }
}
Channel c_TEST_Channel = {.process = c_TEST_Channel_process};


void c_TEST_Channel_process_received() { c_TEST_Channel_process(NULL, PROCESS_RECEIVED_PACKS); }
void c_TEST_Channel_send(Pack* pack) { c_TEST_Channel_process(pack, PROCESS_HOST_REQEST); }
int main()
{
    static uint8_t buff[512];
    static Bounds_Inside PH;
    {
        setPack(c_CommunicationChannel_new_HEARTBEAT_0(), &PH);
        p0_custom_mode_SET((uint32_t)3473256338L, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_BOOT, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_SURFACE_BOAT, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)54030, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)55104, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)34658, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)51245, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -57, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)50589, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)62457, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)10954, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)23946, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)6219, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)911959736L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)8701466209600242140L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_type_mask_SET((uint16_t)(uint16_t)4603, PH.base.pack) ;
        p3_afz_SET((float)1.8253228E38F, PH.base.pack) ;
        p3_x_SET((float) -1.3093658E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2994866487L, PH.base.pack) ;
        p3_y_SET((float)2.9511224E38F, PH.base.pack) ;
        p3_vx_SET((float) -3.107327E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.6429501E38F, PH.base.pack) ;
        p3_vz_SET((float)1.4269583E38F, PH.base.pack) ;
        p3_yaw_SET((float) -2.7133485E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p3_afx_SET((float)1.6213494E38F, PH.base.pack) ;
        p3_vy_SET((float) -2.7590016E38F, PH.base.pack) ;
        p3_z_SET((float)3.318852E38F, PH.base.pack) ;
        p3_afy_SET((float)2.6816858E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)5250370577373848287L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p4_seq_SET((uint32_t)2567245695L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        {
            char16_t* passkey = u"duj";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"emxyyjfyfTexehrvns";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)2696139591L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -13124, PH.base.pack) ;
        {
            char16_t* param_id = u"hnvwlHlzjuwrypvz";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_value_SET((float) -1.899329E38F, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)14066, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)14010, PH.base.pack) ;
        {
            char16_t* param_id = u"co";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_value_SET((float) -9.055052E37F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        {
            char16_t* param_id = u"gokUtggxWrjdX";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_time_usec_SET((uint64_t)4057538721221030602L, PH.base.pack) ;
        p24_alt_SET((int32_t)1615096522, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)11163707L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -278050018, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)351072165L, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)32166, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)2213722939L, &PH) ;
        p24_lon_SET((int32_t) -1905035772, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)3541, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)167634487L, &PH) ;
        p24_lat_SET((int32_t)2079354658, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)8784, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)34073, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)208, (uint8_t)163, (uint8_t)173, (uint8_t)255, (uint8_t)228, (uint8_t)206, (uint8_t)246, (uint8_t)252, (uint8_t)49, (uint8_t)230, (uint8_t)66, (uint8_t)117, (uint8_t)98, (uint8_t)136, (uint8_t)206, (uint8_t)118, (uint8_t)64, (uint8_t)153, (uint8_t)251, (uint8_t)135};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)149, (uint8_t)206, (uint8_t)17, (uint8_t)42, (uint8_t)44, (uint8_t)155, (uint8_t)201, (uint8_t)250, (uint8_t)175, (uint8_t)169, (uint8_t)88, (uint8_t)136, (uint8_t)79, (uint8_t)194, (uint8_t)101, (uint8_t)111, (uint8_t)94, (uint8_t)213, (uint8_t)129, (uint8_t)44};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)250, (uint8_t)170, (uint8_t)119, (uint8_t)113, (uint8_t)202, (uint8_t)212, (uint8_t)111, (uint8_t)236, (uint8_t)62, (uint8_t)114, (uint8_t)135, (uint8_t)40, (uint8_t)134, (uint8_t)203, (uint8_t)20, (uint8_t)194, (uint8_t)229, (uint8_t)53, (uint8_t)233, (uint8_t)225};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)109, (uint8_t)204, (uint8_t)220, (uint8_t)133, (uint8_t)39, (uint8_t)64, (uint8_t)157, (uint8_t)174, (uint8_t)218, (uint8_t)137, (uint8_t)100, (uint8_t)223, (uint8_t)184, (uint8_t)135, (uint8_t)11, (uint8_t)125, (uint8_t)112, (uint8_t)11, (uint8_t)191, (uint8_t)106};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)142, (uint8_t)16, (uint8_t)71, (uint8_t)217, (uint8_t)160, (uint8_t)23, (uint8_t)1, (uint8_t)50, (uint8_t)254, (uint8_t)171, (uint8_t)160, (uint8_t)225, (uint8_t)54, (uint8_t)27, (uint8_t)160, (uint8_t)170, (uint8_t)119, (uint8_t)43, (uint8_t)254, (uint8_t)113};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_zmag_SET((int16_t)(int16_t) -12457, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -17665, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)59307658L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)31814, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -12810, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -23211, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -6739, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -13887, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)31494, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -18089, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t)12613, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -13481, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)24800, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)6240130441531071882L, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)31957, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)2967, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -17407, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)9569, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)9562, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -205, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t)13051, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -7783, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)3868045732865181282L, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)6893, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -2506, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)2.7463825E38F, PH.base.pack) ;
        p29_press_diff_SET((float) -2.9520368E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -31565, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)417424326L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float) -2.1782726E38F, PH.base.pack) ;
        p30_pitch_SET((float) -1.9091804E38F, PH.base.pack) ;
        p30_yawspeed_SET((float) -2.4381418E38F, PH.base.pack) ;
        p30_roll_SET((float)3.4903154E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2063342657L, PH.base.pack) ;
        p30_pitchspeed_SET((float)1.3075602E38F, PH.base.pack) ;
        p30_rollspeed_SET((float) -5.365904E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q2_SET((float)2.9257911E38F, PH.base.pack) ;
        p31_q1_SET((float) -1.793905E38F, PH.base.pack) ;
        p31_q4_SET((float) -3.2202921E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -1.8698631E38F, PH.base.pack) ;
        p31_q3_SET((float) -3.4717445E36F, PH.base.pack) ;
        p31_rollspeed_SET((float) -3.0867312E38F, PH.base.pack) ;
        p31_yawspeed_SET((float)2.3263054E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)4000088348L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_x_SET((float)3.2834475E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)630259066L, PH.base.pack) ;
        p32_z_SET((float) -1.043197E38F, PH.base.pack) ;
        p32_vz_SET((float)1.3874381E38F, PH.base.pack) ;
        p32_vx_SET((float)2.5306764E38F, PH.base.pack) ;
        p32_y_SET((float)4.3581133E37F, PH.base.pack) ;
        p32_vy_SET((float) -3.7692898E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t)147942293, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)3129597245L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)29790, PH.base.pack) ;
        p33_lon_SET((int32_t) -2143268002, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -12754, PH.base.pack) ;
        p33_alt_SET((int32_t) -1506923446, PH.base.pack) ;
        p33_lat_SET((int32_t)1108799583, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)24605, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -5755, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan2_scaled_SET((int16_t)(int16_t) -2528, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1218782226L, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -1597, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -24832, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)3723, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -2942, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -16419, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)10972, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)28366, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan4_raw_SET((uint16_t)(uint16_t)29861, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)1287, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)54373, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)22447, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3893730859L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)34758, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)58617, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)31241, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)7601, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo11_raw_SET((uint16_t)(uint16_t)30567, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)5384, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)60806, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)62437, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)25416, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)53564, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)55770, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)38211, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)27220, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)28926, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)20947, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)50613, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)23140, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)42835, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)3174655675L, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)26964, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)15185, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -5751, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)15441, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t)11237, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)20064, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_autocontinue_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p39_param4_SET((float)7.479992E37F, PH.base.pack) ;
        p39_z_SET((float)9.904857E37F, PH.base.pack) ;
        p39_param2_SET((float)8.695268E37F, PH.base.pack) ;
        p39_param1_SET((float)9.228544E37F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE, PH.base.pack) ;
        p39_y_SET((float) -2.4461456E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)40201, PH.base.pack) ;
        p39_param3_SET((float) -1.674966E38F, PH.base.pack) ;
        p39_x_SET((float) -2.1980869E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)57348, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)41377, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)8153, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)39064, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)38885, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t)543957699, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1055381470, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1390960782, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)6529670338612087669L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_longitude_SET((int32_t) -1259449083, PH.base.pack) ;
        p49_latitude_SET((int32_t) -700907913, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1695486520, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)4890014982723322770L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_scale_SET((float) -2.150964E37F, PH.base.pack) ;
        p50_param_value_max_SET((float) -7.5350323E37F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p50_param_value0_SET((float)1.2636306E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)15940, PH.base.pack) ;
        {
            char16_t* param_id = u"x";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_min_SET((float)1.53327E37F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)29610, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p54_p1y_SET((float)1.5000523E38F, PH.base.pack) ;
        p54_p1z_SET((float) -1.7053513E38F, PH.base.pack) ;
        p54_p2x_SET((float) -1.5536912E38F, PH.base.pack) ;
        p54_p1x_SET((float)8.087403E37F, PH.base.pack) ;
        p54_p2y_SET((float)1.8828015E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p54_p2z_SET((float)1.159999E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1y_SET((float)1.0212652E38F, PH.base.pack) ;
        p55_p1z_SET((float)3.0376926E38F, PH.base.pack) ;
        p55_p1x_SET((float)1.3301561E38F, PH.base.pack) ;
        p55_p2y_SET((float)2.94403E38F, PH.base.pack) ;
        p55_p2z_SET((float) -7.2593945E36F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p55_p2x_SET((float) -1.6229152E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_yawspeed_SET((float) -3.3393105E38F, PH.base.pack) ;
        p61_rollspeed_SET((float) -1.5636592E38F, PH.base.pack) ;
        {
            float q[] =  {1.8319259E38F, 6.2625705E36F, 3.1676512E37F, -3.2698613E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)6868532390058177135L, PH.base.pack) ;
        {
            float covariance[] =  {-2.6542882E38F, 8.768874E37F, 1.7195139E37F, -5.725912E37F, -2.1791634E38F, -2.1836872E38F, -5.1119674E37F, -1.1706831E38F, 3.5431813E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)4.5123123E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_aspd_error_SET((float)2.3301941E38F, PH.base.pack) ;
        p62_alt_error_SET((float)2.7564536E36F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)13535, PH.base.pack) ;
        p62_nav_pitch_SET((float) -4.8372903E37F, PH.base.pack) ;
        p62_nav_roll_SET((float)3.2785442E38F, PH.base.pack) ;
        p62_xtrack_error_SET((float)3.245488E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)12074, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -20447, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_lon_SET((int32_t)1781977607, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)3914575210239425035L, PH.base.pack) ;
        p63_vx_SET((float)7.2206266E37F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1939944674, PH.base.pack) ;
        p63_alt_SET((int32_t)1332501695, PH.base.pack) ;
        p63_vy_SET((float)7.087076E37F, PH.base.pack) ;
        {
            float covariance[] =  {3.042942E38F, -3.3770569E38F, 2.4165187E38F, -3.3222916E38F, 2.0426066E38F, -2.540912E38F, -1.0193723E38F, -3.2027032E38F, -2.7461366E38F, 1.7111667E38F, 1.8360053E38F, -1.7130199E38F, 3.1888832E38F, 2.4502913E38F, 2.1693652E38F, -2.2462255E38F, -1.5726621E38F, 2.3421035E38F, -3.0749274E37F, 7.05773E37F, -3.0093378E38F, -3.1132497E37F, 4.7149955E37F, -3.5916788E37F, -2.4585538E38F, -5.9794475E37F, 2.2707752E38F, 2.8507523E38F, 2.3514846E38F, 2.41959E38F, 4.1102683E37F, -3.3948953E38F, -5.294936E37F, -9.912239E37F, 7.637759E37F, -3.3491613E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float) -1.9182411E38F, PH.base.pack) ;
        p63_lat_SET((int32_t) -1905357514, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)6840861446907552184L, PH.base.pack) ;
        p64_y_SET((float) -1.0210766E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        {
            float covariance[] =  {-2.0267678E38F, -2.3802128E38F, 3.2095587E38F, 4.3625673E37F, 1.5825956E38F, -1.1901752E38F, 9.88296E37F, 3.2841426E38F, 2.9126341E38F, 7.047598E37F, -3.0045E38F, -1.0342931E38F, 1.4822627E38F, -5.1415944E37F, 5.489218E37F, 1.4470436E37F, -4.9143655E37F, -3.3971472E37F, 1.6171797E38F, 2.514227E38F, 2.968204E38F, -2.5292633E38F, -3.0479397E38F, -8.280108E37F, -1.5879017E38F, -1.345013E38F, 3.9642332E37F, -1.2919841E38F, 8.873542E37F, -4.4770748E36F, -2.8778032E38F, -1.2368249E38F, -1.3501561E37F, 7.1637694E37F, -2.126477E38F, 2.2818585E38F, -3.2283083E38F, 2.215962E38F, 1.8835815E38F, -2.0684796E38F, -6.7636877E37F, -2.3651245E38F, 1.2578159E38F, 1.0837243E38F, -2.5993979E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vz_SET((float) -3.4533913E37F, PH.base.pack) ;
        p64_vx_SET((float) -9.838845E37F, PH.base.pack) ;
        p64_ax_SET((float) -2.8027907E38F, PH.base.pack) ;
        p64_z_SET((float)1.2428261E38F, PH.base.pack) ;
        p64_az_SET((float) -2.7447445E38F, PH.base.pack) ;
        p64_ay_SET((float) -3.3245656E38F, PH.base.pack) ;
        p64_x_SET((float)8.714836E37F, PH.base.pack) ;
        p64_vy_SET((float)4.748064E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan8_raw_SET((uint16_t)(uint16_t)47448, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)10363, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)27436, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)63369, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)2743, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)43606, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)39009, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)55156, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)61555, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)1777, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)34108, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)21005, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)1358418158L, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)58471, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)2544, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)13946, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)27705, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)44773, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)11404, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)60430, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)17412, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_r_SET((int16_t)(int16_t) -17393, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -28557, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)14166, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)52292, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)24978, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan5_raw_SET((uint16_t)(uint16_t)3561, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)17084, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)26327, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)64019, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)57080, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)21745, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)30826, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)38391, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p73_x_SET((int32_t)1833981712, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)51436, PH.base.pack) ;
        p73_param3_SET((float)1.6314174E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p73_z_SET((float)1.4716172E37F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_param2_SET((float)2.8786773E38F, PH.base.pack) ;
        p73_param1_SET((float) -5.015129E37F, PH.base.pack) ;
        p73_y_SET((int32_t) -1825103608, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_USER_4, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p73_param4_SET((float) -2.5691168E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float) -2.3945303E38F, PH.base.pack) ;
        p74_alt_SET((float)5.055715E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)35668, PH.base.pack) ;
        p74_groundspeed_SET((float) -3.6161726E37F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)1466, PH.base.pack) ;
        p74_climb_SET((float) -2.1007745E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_REPOSITION, PH.base.pack) ;
        p75_param2_SET((float)8.548283E37F, PH.base.pack) ;
        p75_x_SET((int32_t) -1066873593, PH.base.pack) ;
        p75_param3_SET((float)1.3842496E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p75_param1_SET((float) -3.399694E38F, PH.base.pack) ;
        p75_param4_SET((float) -5.0740276E37F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p75_y_SET((int32_t) -1165759413, PH.base.pack) ;
        p75_z_SET((float)2.3041381E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p76_param4_SET((float)1.2580275E38F, PH.base.pack) ;
        p76_param1_SET((float) -1.7039703E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p76_param2_SET((float)9.984129E37F, PH.base.pack) ;
        p76_param7_SET((float)1.3754551E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p76_param3_SET((float) -2.9528657E38F, PH.base.pack) ;
        p76_param6_SET((float) -2.2581682E38F, PH.base.pack) ;
        p76_param5_SET((float)1.7800285E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)63, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_STORAGE_FORMAT, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)197, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)86, &PH) ;
        p77_result_param2_SET((int32_t)1919772252, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p81_yaw_SET((float) -1.1193933E38F, PH.base.pack) ;
        p81_roll_SET((float) -3.240329E38F, PH.base.pack) ;
        p81_thrust_SET((float)1.6786203E38F, PH.base.pack) ;
        p81_pitch_SET((float) -1.9361337E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3121835409L, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)447063262L, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -1.776839E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)2.9441723E37F, PH.base.pack) ;
        {
            float q[] =  {-1.2861756E38F, -1.0440092E38F, -2.081663E38F, -3.183554E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float)1.4478203E37F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p82_thrust_SET((float)6.45079E37F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float)4.7352667E37F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)3.2967063E38F, PH.base.pack) ;
        p83_thrust_SET((float) -3.166687E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)2.5878473E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)3535735512L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        {
            float q[] =  {1.016169E38F, 1.2416461E38F, -1.8213924E38F, -1.5204962E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_yaw_rate_SET((float) -1.3329793E38F, PH.base.pack) ;
        p84_x_SET((float)2.281713E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p84_afx_SET((float)2.0011033E38F, PH.base.pack) ;
        p84_z_SET((float)2.4646164E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p84_vx_SET((float)3.3870748E38F, PH.base.pack) ;
        p84_y_SET((float) -2.5538324E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)7433, PH.base.pack) ;
        p84_vy_SET((float)2.5219446E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3080430978L, PH.base.pack) ;
        p84_afz_SET((float)3.3071728E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p84_vz_SET((float) -1.3270502E38F, PH.base.pack) ;
        p84_yaw_SET((float) -2.5397098E38F, PH.base.pack) ;
        p84_afy_SET((float)4.5636907E37F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p86_vy_SET((float)2.7367303E38F, PH.base.pack) ;
        p86_vz_SET((float)2.3918451E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -506630299, PH.base.pack) ;
        p86_afz_SET((float)3.0521136E38F, PH.base.pack) ;
        p86_afx_SET((float) -2.8067245E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p86_lon_int_SET((int32_t)963049746, PH.base.pack) ;
        p86_alt_SET((float)2.7520544E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -4.5022705E37F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)6621, PH.base.pack) ;
        p86_afy_SET((float) -2.6273503E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)434876613L, PH.base.pack) ;
        p86_yaw_SET((float) -9.825652E36F, PH.base.pack) ;
        p86_vx_SET((float) -8.947105E37F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afx_SET((float)1.7120013E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1530578360, PH.base.pack) ;
        p87_yaw_rate_SET((float) -2.8086967E38F, PH.base.pack) ;
        p87_alt_SET((float)5.9577296E37F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p87_afz_SET((float)1.3004249E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2929938786L, PH.base.pack) ;
        p87_yaw_SET((float)2.4187005E38F, PH.base.pack) ;
        p87_afy_SET((float)3.3855057E38F, PH.base.pack) ;
        p87_vy_SET((float) -2.1146448E38F, PH.base.pack) ;
        p87_vx_SET((float) -1.3773908E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -469102545, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)498, PH.base.pack) ;
        p87_vz_SET((float) -1.2772831E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_pitch_SET((float) -6.7218086E37F, PH.base.pack) ;
        p89_x_SET((float)1.6617693E37F, PH.base.pack) ;
        p89_z_SET((float)2.5593036E38F, PH.base.pack) ;
        p89_y_SET((float)2.4958393E38F, PH.base.pack) ;
        p89_roll_SET((float) -1.8976786E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)517236725L, PH.base.pack) ;
        p89_yaw_SET((float) -5.588522E36F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_vz_SET((int16_t)(int16_t) -9783, PH.base.pack) ;
        p90_yawspeed_SET((float)2.012785E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)1308806021531935658L, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)10854, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)787, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)17978, PH.base.pack) ;
        p90_rollspeed_SET((float) -1.0990084E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -24503, PH.base.pack) ;
        p90_roll_SET((float) -3.1415244E38F, PH.base.pack) ;
        p90_lat_SET((int32_t)1778208770, PH.base.pack) ;
        p90_yaw_SET((float) -2.3987036E38F, PH.base.pack) ;
        p90_lon_SET((int32_t)1289865568, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.8417E37F, PH.base.pack) ;
        p90_pitch_SET((float) -3.2463781E38F, PH.base.pack) ;
        p90_alt_SET((int32_t)138637301, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)16326, PH.base.pack) ;
        c_TEST_Channel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_throttle_SET((float) -9.636375E37F, PH.base.pack) ;
        p91_aux3_SET((float) -3.0406445E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)1502598631686122471L, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p91_yaw_rudder_SET((float)2.9800314E38F, PH.base.pack) ;
        p91_aux4_SET((float) -1.7767685E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)3.3754272E38F, PH.base.pack) ;
        p91_aux1_SET((float) -1.7561705E38F, PH.base.pack) ;
        p91_aux2_SET((float)2.7127415E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.163045E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan10_raw_SET((uint16_t)(uint16_t)35803, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)14977, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)57784, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)27269981483190511L, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)2533, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)7001, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)36827, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)12291, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)31166, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)65306, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)34393, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)11724, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)59902, PH.base.pack) ;
        c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)5732493282806625187L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)2427409631316157394L, PH.base.pack) ;
        {
            float controls[] =  {1.296122E38F, 2.4771765E38F, -1.6323735E38F, -2.5396027E38F, -5.649296E37F, 1.0329847E38F, 2.0473287E38F, -2.6651287E37F, -3.2771508E38F, 1.3150501E38F, -2.6064369E38F, -2.2323063E37F, -9.631881E37F, 3.3706468E38F, -1.3839112E38F, -2.1172913E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_ground_distance_SET((float) -2.5283796E38F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -1.0340761E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)6454181654021257243L, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)5.702404E37F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -8227, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -5.582826E37F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)20204, PH.base.pack) ;
        p100_flow_rate_y_SET((float)2.887878E38F, &PH) ;
        c_TEST_Channel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float)8.558883E37F, PH.base.pack) ;
        p101_z_SET((float) -2.6450434E38F, PH.base.pack) ;
        p101_x_SET((float)2.452712E38F, PH.base.pack) ;
        p101_pitch_SET((float) -2.974933E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)5016536245288174239L, PH.base.pack) ;
        p101_yaw_SET((float)1.4119027E38F, PH.base.pack) ;
        p101_roll_SET((float) -1.7423957E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float) -2.4305862E38F, PH.base.pack) ;
        p102_yaw_SET((float) -8.1296416E37F, PH.base.pack) ;
        p102_z_SET((float)1.1526458E38F, PH.base.pack) ;
        p102_roll_SET((float)5.517476E37F, PH.base.pack) ;
        p102_x_SET((float) -1.7929413E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)8809574321192565503L, PH.base.pack) ;
        p102_pitch_SET((float) -2.7903943E38F, PH.base.pack) ;
        c_TEST_Channel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_usec_SET((uint64_t)6384236830692217058L, PH.base.pack) ;
        p103_y_SET((float)3.083999E38F, PH.base.pack) ;
        p103_x_SET((float) -3.865643E37F, PH.base.pack) ;
        p103_z_SET((float) -2.1738731E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float) -3.2674218E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)7930840519941980549L, PH.base.pack) ;
        p104_pitch_SET((float)2.819721E37F, PH.base.pack) ;
        p104_yaw_SET((float) -1.0449101E38F, PH.base.pack) ;
        p104_x_SET((float) -8.4217686E37F, PH.base.pack) ;
        p104_y_SET((float)2.9631375E38F, PH.base.pack) ;
        p104_roll_SET((float) -1.1464653E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_pressure_alt_SET((float) -3.1362463E38F, PH.base.pack) ;
        p105_zacc_SET((float) -2.1361917E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -5.4022067E35F, PH.base.pack) ;
        p105_zmag_SET((float) -2.0786385E38F, PH.base.pack) ;
        p105_temperature_SET((float)3.18487E38F, PH.base.pack) ;
        p105_xgyro_SET((float)3.0346586E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -3.0594374E38F, PH.base.pack) ;
        p105_ymag_SET((float) -4.0989366E37F, PH.base.pack) ;
        p105_xmag_SET((float) -9.415653E37F, PH.base.pack) ;
        p105_xacc_SET((float) -2.845407E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -8.1727214E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -1.0328859E38F, PH.base.pack) ;
        p105_yacc_SET((float) -2.2729306E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8709509946926204667L, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)28749, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_y_SET((float) -3.10678E38F, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.9263296E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p106_distance_SET((float) -2.2105303E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)20114, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.1844881E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)4863102767190122976L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)2.2989442E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3877446271L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)2.554222E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3391285960L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_ygyro_SET((float)1.1860863E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)4846370867762031452L, PH.base.pack) ;
        p107_temperature_SET((float)2.8750119E38F, PH.base.pack) ;
        p107_yacc_SET((float) -2.3809523E38F, PH.base.pack) ;
        p107_ymag_SET((float) -2.2505765E38F, PH.base.pack) ;
        p107_zmag_SET((float)3.0965272E38F, PH.base.pack) ;
        p107_xacc_SET((float) -2.8537178E38F, PH.base.pack) ;
        p107_xmag_SET((float) -2.790117E38F, PH.base.pack) ;
        p107_xgyro_SET((float)6.737639E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float)2.0428548E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.6731007E37F, PH.base.pack) ;
        p107_pressure_alt_SET((float)1.5149414E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)1702696425L, PH.base.pack) ;
        p107_zacc_SET((float)8.915135E37F, PH.base.pack) ;
        p107_zgyro_SET((float)1.4572054E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_roll_SET((float)5.346685E36F, PH.base.pack) ;
        p108_xgyro_SET((float)1.329684E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.7448996E38F, PH.base.pack) ;
        p108_q3_SET((float) -7.1383905E37F, PH.base.pack) ;
        p108_alt_SET((float) -1.3344527E37F, PH.base.pack) ;
        p108_zacc_SET((float) -6.396129E37F, PH.base.pack) ;
        p108_yacc_SET((float) -2.2321696E38F, PH.base.pack) ;
        p108_vn_SET((float) -3.4288547E37F, PH.base.pack) ;
        p108_pitch_SET((float)9.372106E37F, PH.base.pack) ;
        p108_q1_SET((float) -7.9086926E37F, PH.base.pack) ;
        p108_ygyro_SET((float) -1.0662154E38F, PH.base.pack) ;
        p108_q2_SET((float) -1.4183924E38F, PH.base.pack) ;
        p108_q4_SET((float)1.5308777E38F, PH.base.pack) ;
        p108_lat_SET((float)2.0770287E38F, PH.base.pack) ;
        p108_zgyro_SET((float)9.722182E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.6691852E38F, PH.base.pack) ;
        p108_vd_SET((float) -2.5532493E38F, PH.base.pack) ;
        p108_yaw_SET((float) -2.1164978E38F, PH.base.pack) ;
        p108_ve_SET((float) -3.1755037E38F, PH.base.pack) ;
        p108_lon_SET((float) -1.0745593E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.2982923E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_txbuf_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)1903, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)40437, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_network_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)14, (uint8_t)141, (uint8_t)177, (uint8_t)106, (uint8_t)135, (uint8_t)45, (uint8_t)156, (uint8_t)131, (uint8_t)111, (uint8_t)91, (uint8_t)25, (uint8_t)142, (uint8_t)67, (uint8_t)85, (uint8_t)67, (uint8_t)87, (uint8_t)152, (uint8_t)196, (uint8_t)177, (uint8_t)33, (uint8_t)38, (uint8_t)70, (uint8_t)102, (uint8_t)29, (uint8_t)110, (uint8_t)178, (uint8_t)16, (uint8_t)142, (uint8_t)25, (uint8_t)182, (uint8_t)102, (uint8_t)4, (uint8_t)248, (uint8_t)57, (uint8_t)233, (uint8_t)251, (uint8_t)73, (uint8_t)177, (uint8_t)144, (uint8_t)64, (uint8_t)200, (uint8_t)94, (uint8_t)234, (uint8_t)174, (uint8_t)38, (uint8_t)53, (uint8_t)71, (uint8_t)244, (uint8_t)57, (uint8_t)38, (uint8_t)3, (uint8_t)136, (uint8_t)77, (uint8_t)239, (uint8_t)56, (uint8_t)16, (uint8_t)102, (uint8_t)59, (uint8_t)26, (uint8_t)251, (uint8_t)66, (uint8_t)156, (uint8_t)186, (uint8_t)28, (uint8_t)6, (uint8_t)187, (uint8_t)81, (uint8_t)225, (uint8_t)146, (uint8_t)248, (uint8_t)177, (uint8_t)117, (uint8_t)59, (uint8_t)198, (uint8_t)35, (uint8_t)136, (uint8_t)180, (uint8_t)224, (uint8_t)198, (uint8_t)188, (uint8_t)185, (uint8_t)68, (uint8_t)124, (uint8_t)37, (uint8_t)250, (uint8_t)217, (uint8_t)127, (uint8_t)84, (uint8_t)64, (uint8_t)237, (uint8_t)10, (uint8_t)50, (uint8_t)233, (uint8_t)178, (uint8_t)82, (uint8_t)168, (uint8_t)101, (uint8_t)103, (uint8_t)174, (uint8_t)222, (uint8_t)83, (uint8_t)181, (uint8_t)135, (uint8_t)118, (uint8_t)200, (uint8_t)89, (uint8_t)47, (uint8_t)161, (uint8_t)15, (uint8_t)172, (uint8_t)156, (uint8_t)154, (uint8_t)2, (uint8_t)100, (uint8_t)85, (uint8_t)47, (uint8_t)116, (uint8_t)94, (uint8_t)149, (uint8_t)117, (uint8_t)156, (uint8_t)151, (uint8_t)45, (uint8_t)148, (uint8_t)198, (uint8_t)177, (uint8_t)20, (uint8_t)189, (uint8_t)14, (uint8_t)172, (uint8_t)132, (uint8_t)245, (uint8_t)162, (uint8_t)247, (uint8_t)39, (uint8_t)167, (uint8_t)91, (uint8_t)96, (uint8_t)9, (uint8_t)211, (uint8_t)17, (uint8_t)47, (uint8_t)247, (uint8_t)52, (uint8_t)103, (uint8_t)226, (uint8_t)53, (uint8_t)191, (uint8_t)27, (uint8_t)183, (uint8_t)253, (uint8_t)154, (uint8_t)117, (uint8_t)56, (uint8_t)27, (uint8_t)87, (uint8_t)153, (uint8_t)208, (uint8_t)15, (uint8_t)189, (uint8_t)111, (uint8_t)73, (uint8_t)246, (uint8_t)101, (uint8_t)4, (uint8_t)106, (uint8_t)241, (uint8_t)159, (uint8_t)165, (uint8_t)133, (uint8_t)95, (uint8_t)145, (uint8_t)84, (uint8_t)108, (uint8_t)3, (uint8_t)70, (uint8_t)198, (uint8_t)217, (uint8_t)108, (uint8_t)112, (uint8_t)148, (uint8_t)0, (uint8_t)221, (uint8_t)56, (uint8_t)159, (uint8_t)82, (uint8_t)148, (uint8_t)162, (uint8_t)180, (uint8_t)135, (uint8_t)3, (uint8_t)115, (uint8_t)119, (uint8_t)230, (uint8_t)184, (uint8_t)235, (uint8_t)61, (uint8_t)238, (uint8_t)190, (uint8_t)54, (uint8_t)95, (uint8_t)148, (uint8_t)63, (uint8_t)98, (uint8_t)232, (uint8_t)102, (uint8_t)178, (uint8_t)57, (uint8_t)41, (uint8_t)195, (uint8_t)197, (uint8_t)214, (uint8_t)73, (uint8_t)15, (uint8_t)20, (uint8_t)19, (uint8_t)63, (uint8_t)220, (uint8_t)206, (uint8_t)7, (uint8_t)174, (uint8_t)14, (uint8_t)241, (uint8_t)132, (uint8_t)192, (uint8_t)203, (uint8_t)139, (uint8_t)157, (uint8_t)238, (uint8_t)169, (uint8_t)225, (uint8_t)167, (uint8_t)102, (uint8_t)142, (uint8_t)76, (uint8_t)149, (uint8_t)175, (uint8_t)136, (uint8_t)231, (uint8_t)223, (uint8_t)155, (uint8_t)91, (uint8_t)168, (uint8_t)9, (uint8_t)69, (uint8_t)37, (uint8_t)86, (uint8_t)173, (uint8_t)117, (uint8_t)129, (uint8_t)170};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)342863465457903769L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -6303449974993382477L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)223074054L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)1564453917102649671L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t) -25221, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)55261, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)41159, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)16296, PH.base.pack) ;
        p113_lon_SET((int32_t)884344387, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)7360602235232762631L, PH.base.pack) ;
        p113_alt_SET((int32_t) -457058146, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)51727, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -31513, PH.base.pack) ;
        p113_lat_SET((int32_t)1691933438, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -13602, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_temperature_SET((int16_t)(int16_t) -17913, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)7942756989323883226L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.8738557E38F, PH.base.pack) ;
        p114_distance_SET((float)2.0696244E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1182563282L, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.6808253E38F, PH.base.pack) ;
        p114_integrated_x_SET((float)2.3649883E37F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)234345247L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.1219105E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)1.805269E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_time_usec_SET((uint64_t)8388180451266151954L, PH.base.pack) ;
        p115_pitchspeed_SET((float) -1.5326824E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.0995912E38F, 7.1376233E37F, 2.4338732E37F, 6.5911437E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_alt_SET((int32_t)1970435894, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)30684, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)27660, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -31835, PH.base.pack) ;
        p115_lat_SET((int32_t) -1312834695, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)4411, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.2386933E38F, PH.base.pack) ;
        p115_lon_SET((int32_t)2098267362, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)18536, PH.base.pack) ;
        p115_rollspeed_SET((float) -3.2549426E38F, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)25066, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)42170, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -18282, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zmag_SET((int16_t)(int16_t)32575, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)9328, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)17877, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)12267, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)22040, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)2839, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2072618281L, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)19283, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -4468, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)13211, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)41931, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)3219, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)256833865L, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)298310947L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)20264, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)15114, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)47429, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)25426, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p119_ofs_SET((uint32_t)569797457L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p119_count_SET((uint32_t)2191085204L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)19106, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)202, (uint8_t)103, (uint8_t)23, (uint8_t)27, (uint8_t)179, (uint8_t)112, (uint8_t)214, (uint8_t)217, (uint8_t)239, (uint8_t)174, (uint8_t)213, (uint8_t)243, (uint8_t)196, (uint8_t)32, (uint8_t)3, (uint8_t)142, (uint8_t)23, (uint8_t)12, (uint8_t)0, (uint8_t)238, (uint8_t)15, (uint8_t)108, (uint8_t)244, (uint8_t)60, (uint8_t)20, (uint8_t)230, (uint8_t)107, (uint8_t)157, (uint8_t)219, (uint8_t)86, (uint8_t)224, (uint8_t)111, (uint8_t)174, (uint8_t)67, (uint8_t)248, (uint8_t)230, (uint8_t)175, (uint8_t)236, (uint8_t)107, (uint8_t)133, (uint8_t)143, (uint8_t)164, (uint8_t)97, (uint8_t)82, (uint8_t)143, (uint8_t)184, (uint8_t)66, (uint8_t)71, (uint8_t)146, (uint8_t)149, (uint8_t)56, (uint8_t)131, (uint8_t)212, (uint8_t)38, (uint8_t)216, (uint8_t)58, (uint8_t)97, (uint8_t)84, (uint8_t)222, (uint8_t)184, (uint8_t)31, (uint8_t)113, (uint8_t)125, (uint8_t)228, (uint8_t)96, (uint8_t)95, (uint8_t)19, (uint8_t)110, (uint8_t)186, (uint8_t)44, (uint8_t)14, (uint8_t)30, (uint8_t)198, (uint8_t)6, (uint8_t)49, (uint8_t)132, (uint8_t)213, (uint8_t)113, (uint8_t)229, (uint8_t)229, (uint8_t)29, (uint8_t)170, (uint8_t)62, (uint8_t)165, (uint8_t)83, (uint8_t)168, (uint8_t)85, (uint8_t)230, (uint8_t)23, (uint8_t)37};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_count_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p120_ofs_SET((uint32_t)1434441270L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)81, (uint8_t)98, (uint8_t)85, (uint8_t)74, (uint8_t)164, (uint8_t)116, (uint8_t)235, (uint8_t)225, (uint8_t)194, (uint8_t)52, (uint8_t)56, (uint8_t)78, (uint8_t)99, (uint8_t)243, (uint8_t)78, (uint8_t)161, (uint8_t)253, (uint8_t)130, (uint8_t)105, (uint8_t)246, (uint8_t)134, (uint8_t)28, (uint8_t)44, (uint8_t)244, (uint8_t)175, (uint8_t)238, (uint8_t)21, (uint8_t)31, (uint8_t)215, (uint8_t)135, (uint8_t)213, (uint8_t)214, (uint8_t)215, (uint8_t)103, (uint8_t)233, (uint8_t)114, (uint8_t)30, (uint8_t)50, (uint8_t)158, (uint8_t)17, (uint8_t)2, (uint8_t)217, (uint8_t)136, (uint8_t)43, (uint8_t)147, (uint8_t)215, (uint8_t)248, (uint8_t)233, (uint8_t)74, (uint8_t)12, (uint8_t)190, (uint8_t)44, (uint8_t)65, (uint8_t)10, (uint8_t)100, (uint8_t)245, (uint8_t)88, (uint8_t)168, (uint8_t)50, (uint8_t)196, (uint8_t)78, (uint8_t)89, (uint8_t)232, (uint8_t)228, (uint8_t)135, (uint8_t)207, (uint8_t)26, (uint8_t)81, (uint8_t)142, (uint8_t)202, (uint8_t)59, (uint8_t)110, (uint8_t)183, (uint8_t)32, (uint8_t)122, (uint8_t)225, (uint8_t)143, (uint8_t)133, (uint8_t)109, (uint8_t)233, (uint8_t)61, (uint8_t)10, (uint8_t)158, (uint8_t)170, (uint8_t)161, (uint8_t)61, (uint8_t)152, (uint8_t)193, (uint8_t)113, (uint8_t)181, (uint8_t)88, (uint8_t)246, (uint8_t)36, (uint8_t)208, (uint8_t)54, (uint8_t)68, (uint8_t)92, (uint8_t)105, (uint8_t)165, (uint8_t)222, (uint8_t)162, (uint8_t)178, (uint8_t)40, (uint8_t)16, (uint8_t)179, (uint8_t)115, (uint8_t)64, (uint8_t)175, (uint8_t)140, (uint8_t)77};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_vel_SET((uint16_t)(uint16_t)11717, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)562287537L, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_lat_SET((int32_t)347602116, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)15633, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)13147, PH.base.pack) ;
        p124_lon_SET((int32_t)1490068487, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)12483, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)552671631494735314L, PH.base.pack) ;
        p124_alt_SET((int32_t)518241545, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)24071, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)59550, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)31708, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)146, (uint8_t)177, (uint8_t)195, (uint8_t)94, (uint8_t)106, (uint8_t)176, (uint8_t)255, (uint8_t)31, (uint8_t)19, (uint8_t)55, (uint8_t)68, (uint8_t)183, (uint8_t)8, (uint8_t)98, (uint8_t)75, (uint8_t)18, (uint8_t)19, (uint8_t)107, (uint8_t)155, (uint8_t)69, (uint8_t)176, (uint8_t)56, (uint8_t)99, (uint8_t)51, (uint8_t)79, (uint8_t)8, (uint8_t)232, (uint8_t)177, (uint8_t)18, (uint8_t)96, (uint8_t)21, (uint8_t)3, (uint8_t)162, (uint8_t)59, (uint8_t)95, (uint8_t)50, (uint8_t)164, (uint8_t)15, (uint8_t)205, (uint8_t)123, (uint8_t)98, (uint8_t)116, (uint8_t)110, (uint8_t)175, (uint8_t)152, (uint8_t)83, (uint8_t)15, (uint8_t)86, (uint8_t)37, (uint8_t)203, (uint8_t)179, (uint8_t)45, (uint8_t)90, (uint8_t)120, (uint8_t)115, (uint8_t)26, (uint8_t)63, (uint8_t)153, (uint8_t)211, (uint8_t)48, (uint8_t)38, (uint8_t)22, (uint8_t)95, (uint8_t)40, (uint8_t)93, (uint8_t)196, (uint8_t)83, (uint8_t)154, (uint8_t)163, (uint8_t)25};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_count_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1317965482L, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_rtk_rate_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -131308683, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)1553950476, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)3677915637L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1457971662, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)3450257308L, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1661680039, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p127_tow_SET((uint32_t)1809941862L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)17772, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_tow_SET((uint32_t)773927872L, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)825159956, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)226486067, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)64867, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1299914287, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3384311445L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -85722918, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)2617604077L, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_zgyro_SET((int16_t)(int16_t)7065, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)27953, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -17174, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -26083, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -999, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -16064, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -14683, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)7552, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1839688710L, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -18120, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_width_SET((uint16_t)(uint16_t)46677, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p130_size_SET((uint32_t)1575004946L, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)47650, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)14419, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)209, (uint8_t)193, (uint8_t)219, (uint8_t)172, (uint8_t)165, (uint8_t)68, (uint8_t)167, (uint8_t)175, (uint8_t)85, (uint8_t)17, (uint8_t)234, (uint8_t)17, (uint8_t)36, (uint8_t)85, (uint8_t)15, (uint8_t)146, (uint8_t)155, (uint8_t)4, (uint8_t)151, (uint8_t)44, (uint8_t)215, (uint8_t)65, (uint8_t)123, (uint8_t)48, (uint8_t)85, (uint8_t)235, (uint8_t)71, (uint8_t)151, (uint8_t)174, (uint8_t)239, (uint8_t)239, (uint8_t)95, (uint8_t)99, (uint8_t)191, (uint8_t)114, (uint8_t)197, (uint8_t)57, (uint8_t)255, (uint8_t)131, (uint8_t)229, (uint8_t)30, (uint8_t)184, (uint8_t)97, (uint8_t)213, (uint8_t)42, (uint8_t)121, (uint8_t)174, (uint8_t)133, (uint8_t)99, (uint8_t)116, (uint8_t)219, (uint8_t)160, (uint8_t)103, (uint8_t)46, (uint8_t)134, (uint8_t)59, (uint8_t)126, (uint8_t)207, (uint8_t)58, (uint8_t)90, (uint8_t)29, (uint8_t)198, (uint8_t)35, (uint8_t)159, (uint8_t)167, (uint8_t)198, (uint8_t)206, (uint8_t)222, (uint8_t)66, (uint8_t)72, (uint8_t)20, (uint8_t)121, (uint8_t)169, (uint8_t)201, (uint8_t)118, (uint8_t)11, (uint8_t)12, (uint8_t)116, (uint8_t)56, (uint8_t)91, (uint8_t)1, (uint8_t)46, (uint8_t)13, (uint8_t)129, (uint8_t)66, (uint8_t)11, (uint8_t)124, (uint8_t)111, (uint8_t)124, (uint8_t)181, (uint8_t)4, (uint8_t)18, (uint8_t)47, (uint8_t)103, (uint8_t)201, (uint8_t)253, (uint8_t)255, (uint8_t)146, (uint8_t)75, (uint8_t)224, (uint8_t)203, (uint8_t)66, (uint8_t)54, (uint8_t)75, (uint8_t)247, (uint8_t)250, (uint8_t)45, (uint8_t)3, (uint8_t)173, (uint8_t)104, (uint8_t)94, (uint8_t)32, (uint8_t)61, (uint8_t)2, (uint8_t)164, (uint8_t)176, (uint8_t)6, (uint8_t)105, (uint8_t)43, (uint8_t)136, (uint8_t)228, (uint8_t)108, (uint8_t)181, (uint8_t)157, (uint8_t)218, (uint8_t)226, (uint8_t)122, (uint8_t)4, (uint8_t)195, (uint8_t)18, (uint8_t)184, (uint8_t)166, (uint8_t)170, (uint8_t)112, (uint8_t)137, (uint8_t)40, (uint8_t)10, (uint8_t)184, (uint8_t)122, (uint8_t)100, (uint8_t)94, (uint8_t)231, (uint8_t)59, (uint8_t)213, (uint8_t)85, (uint8_t)74, (uint8_t)185, (uint8_t)227, (uint8_t)13, (uint8_t)41, (uint8_t)36, (uint8_t)44, (uint8_t)138, (uint8_t)210, (uint8_t)163, (uint8_t)129, (uint8_t)110, (uint8_t)248, (uint8_t)197, (uint8_t)152, (uint8_t)241, (uint8_t)13, (uint8_t)182, (uint8_t)63, (uint8_t)87, (uint8_t)193, (uint8_t)234, (uint8_t)115, (uint8_t)251, (uint8_t)233, (uint8_t)246, (uint8_t)13, (uint8_t)239, (uint8_t)19, (uint8_t)148, (uint8_t)46, (uint8_t)95, (uint8_t)205, (uint8_t)112, (uint8_t)175, (uint8_t)184, (uint8_t)253, (uint8_t)29, (uint8_t)20, (uint8_t)229, (uint8_t)222, (uint8_t)237, (uint8_t)138, (uint8_t)108, (uint8_t)167, (uint8_t)231, (uint8_t)221, (uint8_t)104, (uint8_t)217, (uint8_t)25, (uint8_t)68, (uint8_t)36, (uint8_t)215, (uint8_t)158, (uint8_t)107, (uint8_t)166, (uint8_t)253, (uint8_t)99, (uint8_t)205, (uint8_t)60, (uint8_t)96, (uint8_t)115, (uint8_t)236, (uint8_t)236, (uint8_t)55, (uint8_t)149, (uint8_t)224, (uint8_t)9, (uint8_t)114, (uint8_t)74, (uint8_t)240, (uint8_t)94, (uint8_t)87, (uint8_t)166, (uint8_t)189, (uint8_t)36, (uint8_t)117, (uint8_t)182, (uint8_t)178, (uint8_t)46, (uint8_t)208, (uint8_t)128, (uint8_t)176, (uint8_t)202, (uint8_t)235, (uint8_t)18, (uint8_t)230, (uint8_t)133, (uint8_t)69, (uint8_t)17, (uint8_t)138, (uint8_t)1, (uint8_t)117, (uint8_t)8, (uint8_t)39, (uint8_t)179, (uint8_t)121, (uint8_t)241, (uint8_t)15, (uint8_t)253, (uint8_t)230, (uint8_t)1, (uint8_t)182, (uint8_t)102, (uint8_t)51, (uint8_t)104, (uint8_t)152, (uint8_t)153};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)673, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3273661576L, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)45432, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)1763, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)49697, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)24730, PH.base.pack) ;
        p133_mask_SET((uint64_t)8551005329862089722L, PH.base.pack) ;
        p133_lon_SET((int32_t) -853417661, PH.base.pack) ;
        p133_lat_SET((int32_t)843643676, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)48477, PH.base.pack) ;
        p134_lat_SET((int32_t)1281321427, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -19426, (int16_t)6192, (int16_t)29372, (int16_t) -32541, (int16_t) -11856, (int16_t)21362, (int16_t) -13975, (int16_t)14772, (int16_t)5461, (int16_t)4862, (int16_t)17822, (int16_t)21158, (int16_t)4734, (int16_t) -10135, (int16_t)15255, (int16_t) -24904};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lon_SET((int32_t)932913780, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -1647561783, PH.base.pack) ;
        p135_lat_SET((int32_t)977341911, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)23777, PH.base.pack) ;
        p136_current_height_SET((float) -5.2644924E36F, PH.base.pack) ;
        p136_lat_SET((int32_t)925947156, PH.base.pack) ;
        p136_terrain_height_SET((float)2.8410405E38F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)8604, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)4186, PH.base.pack) ;
        p136_lon_SET((int32_t) -1034020909, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float)2.6068143E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -2.5149456E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)4016560535L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -16166, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_z_SET((float)2.0361552E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)5980536357930566149L, PH.base.pack) ;
        p138_y_SET((float)3.1084097E38F, PH.base.pack) ;
        {
            float q[] =  {-1.2252261E38F, -8.7575376E36F, 1.681753E38F, -2.8685343E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -2.8895607E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {7.8464384E36F, 1.4914905E38F, -1.2211145E38F, 6.6566985E37F, 2.5477546E38F, -1.7821658E38F, 3.2474553E38F, -1.780408E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)3536411804479647248L, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-1.1984695E38F, -3.1091695E38F, -8.958287E37F, 1.9829749E38F, 3.032142E38F, -3.3821354E38F, -1.2694561E38F, 2.3702679E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)4586872990326143893L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float)9.183482E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)3.026209E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)124878507215970304L, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -3.3409832E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -1.5380464E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.0001361E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -2.4132964E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)119, (uint8_t)128, (uint8_t)194, (uint8_t)253, (uint8_t)65, (uint8_t)35, (uint8_t)183, (uint8_t)181, (uint8_t)179, (uint8_t)199, (uint8_t)60, (uint8_t)146, (uint8_t)76, (uint8_t)175, (uint8_t)64, (uint8_t)169, (uint8_t)116, (uint8_t)164, (uint8_t)8, (uint8_t)118, (uint8_t)208, (uint8_t)109, (uint8_t)5, (uint8_t)13, (uint8_t)97, (uint8_t)155, (uint8_t)186, (uint8_t)145, (uint8_t)231, (uint8_t)47, (uint8_t)30, (uint8_t)77, (uint8_t)15, (uint8_t)194, (uint8_t)211, (uint8_t)132, (uint8_t)85, (uint8_t)183, (uint8_t)61, (uint8_t)45, (uint8_t)157, (uint8_t)23, (uint8_t)13, (uint8_t)148, (uint8_t)31, (uint8_t)78, (uint8_t)164, (uint8_t)244, (uint8_t)162, (uint8_t)255, (uint8_t)107, (uint8_t)220, (uint8_t)152, (uint8_t)8, (uint8_t)3, (uint8_t)10, (uint8_t)239, (uint8_t)18, (uint8_t)138, (uint8_t)153, (uint8_t)187, (uint8_t)159, (uint8_t)235, (uint8_t)65, (uint8_t)191, (uint8_t)214, (uint8_t)209, (uint8_t)110, (uint8_t)135, (uint8_t)206, (uint8_t)215, (uint8_t)50, (uint8_t)217, (uint8_t)185, (uint8_t)226, (uint8_t)10, (uint8_t)143, (uint8_t)205, (uint8_t)8, (uint8_t)113, (uint8_t)239, (uint8_t)209, (uint8_t)50, (uint8_t)15, (uint8_t)245, (uint8_t)116, (uint8_t)180, (uint8_t)198, (uint8_t)242, (uint8_t)224, (uint8_t)240, (uint8_t)104, (uint8_t)3, (uint8_t)102, (uint8_t)161, (uint8_t)239, (uint8_t)117, (uint8_t)89, (uint8_t)252, (uint8_t)73, (uint8_t)243, (uint8_t)65, (uint8_t)233, (uint8_t)36, (uint8_t)14, (uint8_t)247, (uint8_t)120, (uint8_t)2, (uint8_t)121, (uint8_t)239, (uint8_t)73, (uint8_t)176, (uint8_t)126, (uint8_t)147, (uint8_t)248, (uint8_t)73, (uint8_t)15, (uint8_t)158, (uint8_t)55, (uint8_t)200};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)35, (uint8_t)138, (uint8_t)36, (uint8_t)161, (uint8_t)196, (uint8_t)182, (uint8_t)12, (uint8_t)124, (uint8_t)164, (uint8_t)216, (uint8_t)53, (uint8_t)139, (uint8_t)118, (uint8_t)227, (uint8_t)145, (uint8_t)132, (uint8_t)86, (uint8_t)203, (uint8_t)158, (uint8_t)211, (uint8_t)125, (uint8_t)185, (uint8_t)194, (uint8_t)182, (uint8_t)18, (uint8_t)8, (uint8_t)111, (uint8_t)218, (uint8_t)11, (uint8_t)196, (uint8_t)251, (uint8_t)235, (uint8_t)216, (uint8_t)242, (uint8_t)202, (uint8_t)192, (uint8_t)212, (uint8_t)115, (uint8_t)53, (uint8_t)121, (uint8_t)73, (uint8_t)39, (uint8_t)222, (uint8_t)151, (uint8_t)29, (uint8_t)124, (uint8_t)143, (uint8_t)159, (uint8_t)203, (uint8_t)89, (uint8_t)176, (uint8_t)160, (uint8_t)205, (uint8_t)106, (uint8_t)165, (uint8_t)135, (uint8_t)160, (uint8_t)96, (uint8_t)61, (uint8_t)252, (uint8_t)247, (uint8_t)76, (uint8_t)231, (uint8_t)123, (uint8_t)40, (uint8_t)187, (uint8_t)106, (uint8_t)16, (uint8_t)220, (uint8_t)255, (uint8_t)179, (uint8_t)99, (uint8_t)199, (uint8_t)18, (uint8_t)66, (uint8_t)109, (uint8_t)182, (uint8_t)186, (uint8_t)151, (uint8_t)204, (uint8_t)69, (uint8_t)4, (uint8_t)59, (uint8_t)69, (uint8_t)251, (uint8_t)67, (uint8_t)251, (uint8_t)147, (uint8_t)136, (uint8_t)171, (uint8_t)187, (uint8_t)212, (uint8_t)56, (uint8_t)101, (uint8_t)243, (uint8_t)12, (uint8_t)35, (uint8_t)221, (uint8_t)203, (uint8_t)195, (uint8_t)41, (uint8_t)158, (uint8_t)58, (uint8_t)200, (uint8_t)201, (uint8_t)179, (uint8_t)141, (uint8_t)127, (uint8_t)26, (uint8_t)14, (uint8_t)109, (uint8_t)158, (uint8_t)13, (uint8_t)26, (uint8_t)76, (uint8_t)39, (uint8_t)99, (uint8_t)21, (uint8_t)196, (uint8_t)144};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_time_boot_ms_SET((uint32_t)1233816172L, PH.base.pack) ;
        p143_press_diff_SET((float)7.2793573E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -20398, PH.base.pack) ;
        p143_press_abs_SET((float)2.471792E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float acc[] =  {3.5062812E37F, -3.125209E38F, -1.6939905E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {-2.039113E38F, 3.3312225E38F, -6.524356E37F, 1.1358256E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-2.7560253E38F, -1.1068467E38F, 2.2030895E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)6381942527891513783L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)6210441172642704568L, PH.base.pack) ;
        p144_lon_SET((int32_t) -1392718340, PH.base.pack) ;
        {
            float position_cov[] =  {-6.1877585E37F, -2.7693991E38F, 2.7404085E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-9.908583E37F, 2.4752292E38F, -2.201051E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)2.6886861E38F, PH.base.pack) ;
        p144_lat_SET((int32_t)111284836, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_acc_SET((float) -9.802328E37F, PH.base.pack) ;
        p146_z_acc_SET((float)1.183897E38F, PH.base.pack) ;
        p146_y_vel_SET((float)1.4983169E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -6.202711E37F, PH.base.pack) ;
        {
            float q[] =  {-2.4813242E38F, 2.747353E38F, -1.7955605E38F, 9.409315E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_pos_SET((float) -6.5846315E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {-2.3730711E38F, -4.7420715E37F, -1.080617E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_pos_SET((float)4.654246E37F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -1.290356E38F, PH.base.pack) ;
        p146_airspeed_SET((float)3.0321874E38F, PH.base.pack) ;
        p146_y_pos_SET((float)1.8418087E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {6.573743E37F, 1.2661057E38F, 7.5094425E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_pitch_rate_SET((float) -1.4999511E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)7736542273191355681L, PH.base.pack) ;
        p146_z_vel_SET((float) -2.252626E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.9667828E37F, PH.base.pack) ;
        p146_x_acc_SET((float) -2.7117196E37F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -32165, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)40, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -3266, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -262913710, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -1038579729, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)25852, (uint16_t)42655, (uint16_t)36426, (uint16_t)25650, (uint16_t)17407, (uint16_t)49223, (uint16_t)48561, (uint16_t)19317, (uint16_t)52270, (uint16_t)47251};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_uid_SET((uint64_t)2187919626460316104L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2808021338L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)19692, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)130, (uint8_t)150, (uint8_t)218, (uint8_t)3, (uint8_t)231, (uint8_t)156, (uint8_t)246, (uint8_t)176};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION), PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)55, (uint8_t)56, (uint8_t)183, (uint8_t)27, (uint8_t)30, (uint8_t)13, (uint8_t)12, (uint8_t)124};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)47399782L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)42280, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)184, (uint8_t)114, (uint8_t)1, (uint8_t)38, (uint8_t)120, (uint8_t)154, (uint8_t)181, (uint8_t)39, (uint8_t)143, (uint8_t)24, (uint8_t)19, (uint8_t)243, (uint8_t)73, (uint8_t)61, (uint8_t)174, (uint8_t)71, (uint8_t)213, (uint8_t)12};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)185, (uint8_t)220, (uint8_t)240, (uint8_t)122, (uint8_t)76, (uint8_t)156, (uint8_t)148, (uint8_t)157};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)3534720312L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3525153175L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_size_y_SET((float)1.5738029E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_distance_SET((float)2.2381584E38F, PH.base.pack) ;
        p149_size_x_SET((float)1.9795392E38F, PH.base.pack) ;
        p149_z_SET((float) -2.3352164E38F, &PH) ;
        p149_time_usec_SET((uint64_t)6322294710075288096L, PH.base.pack) ;
        {
            float q[] =  {-3.0973243E38F, -9.132675E37F, 1.5047419E38F, 3.670054E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)158, &PH) ;
        p149_angle_y_SET((float)1.4247271E38F, PH.base.pack) ;
        p149_y_SET((float) -1.431919E38F, &PH) ;
        p149_angle_x_SET((float) -3.517707E36F, PH.base.pack) ;
        p149_x_SET((float)3.0786864E38F, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_OFFSETS_150(), &PH);
        p150_mag_ofs_x_SET((int16_t)(int16_t)17117, PH.base.pack) ;
        p150_gyro_cal_y_SET((float)1.0868558E38F, PH.base.pack) ;
        p150_gyro_cal_x_SET((float) -1.5099644E38F, PH.base.pack) ;
        p150_mag_ofs_z_SET((int16_t)(int16_t) -32160, PH.base.pack) ;
        p150_gyro_cal_z_SET((float)1.8511307E38F, PH.base.pack) ;
        p150_mag_ofs_y_SET((int16_t)(int16_t) -26727, PH.base.pack) ;
        p150_accel_cal_y_SET((float)1.5554018E38F, PH.base.pack) ;
        p150_accel_cal_z_SET((float)1.076419E38F, PH.base.pack) ;
        p150_raw_press_SET((int32_t) -370899457, PH.base.pack) ;
        p150_raw_temp_SET((int32_t) -1199840572, PH.base.pack) ;
        p150_accel_cal_x_SET((float)1.4943014E38F, PH.base.pack) ;
        p150_mag_declination_SET((float)1.1616059E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_OFFSETS_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_MAG_OFFSETS_151(), &PH);
        p151_mag_ofs_y_SET((int16_t)(int16_t)32388, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p151_mag_ofs_z_SET((int16_t)(int16_t)12590, PH.base.pack) ;
        p151_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p151_mag_ofs_x_SET((int16_t)(int16_t)27736, PH.base.pack) ;
        c_CommunicationChannel_on_SET_MAG_OFFSETS_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMINFO_152(), &PH);
        p152_freemem_SET((uint16_t)(uint16_t)16207, PH.base.pack) ;
        p152_brkval_SET((uint16_t)(uint16_t)35492, PH.base.pack) ;
        p152_freemem32_SET((uint32_t)1494387357L, &PH) ;
        c_CommunicationChannel_on_MEMINFO_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AP_ADC_153(), &PH);
        p153_adc2_SET((uint16_t)(uint16_t)31841, PH.base.pack) ;
        p153_adc1_SET((uint16_t)(uint16_t)58040, PH.base.pack) ;
        p153_adc4_SET((uint16_t)(uint16_t)20549, PH.base.pack) ;
        p153_adc3_SET((uint16_t)(uint16_t)54271, PH.base.pack) ;
        p153_adc6_SET((uint16_t)(uint16_t)3569, PH.base.pack) ;
        p153_adc5_SET((uint16_t)(uint16_t)36176, PH.base.pack) ;
        c_CommunicationChannel_on_AP_ADC_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONFIGURE_154(), &PH);
        p154_extra_value_SET((float)3.706903E37F, PH.base.pack) ;
        p154_engine_cut_off_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p154_mode_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p154_iso_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p154_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p154_exposure_type_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p154_extra_param_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p154_aperture_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p154_shutter_speed_SET((uint16_t)(uint16_t)19249, PH.base.pack) ;
        p154_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p154_command_id_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONTROL_155(), &PH);
        p155_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p155_zoom_step_SET((int8_t)(int8_t) -52, PH.base.pack) ;
        p155_extra_param_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p155_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p155_extra_value_SET((float) -8.57339E37F, PH.base.pack) ;
        p155_focus_lock_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p155_session_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p155_command_id_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p155_shot_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p155_zoom_pos_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONTROL_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONFIGURE_156(), &PH);
        p156_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_RC_TARGETING, PH.base.pack) ;
        p156_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p156_stab_pitch_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p156_stab_yaw_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p156_stab_roll_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONFIGURE_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONTROL_157(), &PH);
        p157_input_c_SET((int32_t)124889440, PH.base.pack) ;
        p157_input_b_SET((int32_t)1735441738, PH.base.pack) ;
        p157_input_a_SET((int32_t) -1361309425, PH.base.pack) ;
        p157_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p157_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p157_save_position_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONTROL_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_STATUS_158(), &PH);
        p158_pointing_b_SET((int32_t) -1188738090, PH.base.pack) ;
        p158_pointing_c_SET((int32_t) -1174442133, PH.base.pack) ;
        p158_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p158_pointing_a_SET((int32_t) -482214499, PH.base.pack) ;
        p158_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_STATUS_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_POINT_160(), &PH);
        p160_count_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p160_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p160_idx_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p160_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p160_lat_SET((float) -5.904557E37F, PH.base.pack) ;
        p160_lng_SET((float)3.2317663E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_POINT_160(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_FETCH_POINT_161(), &PH);
        p161_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p161_idx_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p161_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_FETCH_POINT_161(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_STATUS_162(), &PH);
        p162_breach_time_SET((uint32_t)1697244309L, PH.base.pack) ;
        p162_breach_count_SET((uint16_t)(uint16_t)46627, PH.base.pack) ;
        p162_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_NONE, PH.base.pack) ;
        p162_breach_status_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_STATUS_162(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS_163(), &PH);
        p163_renorm_val_SET((float)1.1589041E38F, PH.base.pack) ;
        p163_accel_weight_SET((float)1.4821014E38F, PH.base.pack) ;
        p163_error_yaw_SET((float) -1.2982742E38F, PH.base.pack) ;
        p163_omegaIy_SET((float)3.0565433E38F, PH.base.pack) ;
        p163_omegaIz_SET((float) -2.2814275E38F, PH.base.pack) ;
        p163_error_rp_SET((float) -2.726368E38F, PH.base.pack) ;
        p163_omegaIx_SET((float) -8.995235E37F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS_163(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SIMSTATE_164(), &PH);
        p164_xgyro_SET((float)2.790566E38F, PH.base.pack) ;
        p164_xacc_SET((float)1.596351E38F, PH.base.pack) ;
        p164_pitch_SET((float)2.4826075E38F, PH.base.pack) ;
        p164_yacc_SET((float) -6.403532E37F, PH.base.pack) ;
        p164_lng_SET((int32_t)572870150, PH.base.pack) ;
        p164_roll_SET((float)2.8124936E38F, PH.base.pack) ;
        p164_yaw_SET((float)1.3144662E38F, PH.base.pack) ;
        p164_zacc_SET((float)2.6915311E38F, PH.base.pack) ;
        p164_ygyro_SET((float)2.2995001E38F, PH.base.pack) ;
        p164_zgyro_SET((float) -1.4882056E38F, PH.base.pack) ;
        p164_lat_SET((int32_t) -2034520948, PH.base.pack) ;
        c_CommunicationChannel_on_SIMSTATE_164(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HWSTATUS_165(), &PH);
        p165_I2Cerr_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p165_Vcc_SET((uint16_t)(uint16_t)33402, PH.base.pack) ;
        c_CommunicationChannel_on_HWSTATUS_165(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RADIO_166(), &PH);
        p166_noise_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p166_rssi_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p166_rxerrors_SET((uint16_t)(uint16_t)58191, PH.base.pack) ;
        p166_txbuf_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p166_remnoise_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p166_fixed__SET((uint16_t)(uint16_t)2606, PH.base.pack) ;
        p166_remrssi_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_166(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LIMITS_STATUS_167(), &PH);
        p167_last_recovery_SET((uint32_t)642557098L, PH.base.pack) ;
        p167_last_trigger_SET((uint32_t)4289879297L, PH.base.pack) ;
        p167_limits_state_SET(e_LIMITS_STATE_LIMITS_RECOVERING, PH.base.pack) ;
        p167_last_action_SET((uint32_t)1365279422L, PH.base.pack) ;
        p167_breach_count_SET((uint16_t)(uint16_t)17626, PH.base.pack) ;
        p167_last_clear_SET((uint32_t)4088393930L, PH.base.pack) ;
        p167_mods_triggered_SET((e_LIMIT_MODULE_LIMIT_GEOFENCE |
                                 e_LIMIT_MODULE_LIMIT_ALTITUDE), PH.base.pack) ;
        p167_mods_required_SET((e_LIMIT_MODULE_LIMIT_GEOFENCE), PH.base.pack) ;
        p167_mods_enabled_SET((e_LIMIT_MODULE_LIMIT_GPSLOCK), PH.base.pack) ;
        c_CommunicationChannel_on_LIMITS_STATUS_167(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_168(), &PH);
        p168_speed_z_SET((float)4.9718255E37F, PH.base.pack) ;
        p168_speed_SET((float) -2.9985454E38F, PH.base.pack) ;
        p168_direction_SET((float)1.4033336E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_168(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA16_169(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)158, (uint8_t)238, (uint8_t)141, (uint8_t)167, (uint8_t)249, (uint8_t)105, (uint8_t)88, (uint8_t)171, (uint8_t)16, (uint8_t)62, (uint8_t)127, (uint8_t)107, (uint8_t)7, (uint8_t)91, (uint8_t)165, (uint8_t)126};
            p169_data__SET(&data_, 0, PH.base.pack) ;
        }
        p169_len_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p169_type_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        c_CommunicationChannel_on_DATA16_169(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA32_170(), &PH);
        p170_type_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p170_len_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)59, (uint8_t)87, (uint8_t)29, (uint8_t)226, (uint8_t)212, (uint8_t)192, (uint8_t)1, (uint8_t)110, (uint8_t)31, (uint8_t)137, (uint8_t)207, (uint8_t)211, (uint8_t)152, (uint8_t)171, (uint8_t)254, (uint8_t)102, (uint8_t)58, (uint8_t)47, (uint8_t)30, (uint8_t)234, (uint8_t)84, (uint8_t)114, (uint8_t)199, (uint8_t)110, (uint8_t)16, (uint8_t)118, (uint8_t)113, (uint8_t)72, (uint8_t)1, (uint8_t)131, (uint8_t)116, (uint8_t)204};
            p170_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DATA32_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA64_171(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)214, (uint8_t)175, (uint8_t)57, (uint8_t)123, (uint8_t)232, (uint8_t)23, (uint8_t)43, (uint8_t)205, (uint8_t)225, (uint8_t)43, (uint8_t)136, (uint8_t)107, (uint8_t)125, (uint8_t)154, (uint8_t)117, (uint8_t)188, (uint8_t)39, (uint8_t)186, (uint8_t)23, (uint8_t)28, (uint8_t)124, (uint8_t)138, (uint8_t)32, (uint8_t)216, (uint8_t)59, (uint8_t)231, (uint8_t)19, (uint8_t)125, (uint8_t)26, (uint8_t)104, (uint8_t)22, (uint8_t)201, (uint8_t)5, (uint8_t)208, (uint8_t)160, (uint8_t)174, (uint8_t)33, (uint8_t)89, (uint8_t)173, (uint8_t)152, (uint8_t)147, (uint8_t)218, (uint8_t)228, (uint8_t)50, (uint8_t)84, (uint8_t)222, (uint8_t)133, (uint8_t)26, (uint8_t)188, (uint8_t)62, (uint8_t)154, (uint8_t)122, (uint8_t)184, (uint8_t)140, (uint8_t)241, (uint8_t)212, (uint8_t)108, (uint8_t)25, (uint8_t)155, (uint8_t)51, (uint8_t)61, (uint8_t)203, (uint8_t)219, (uint8_t)107};
            p171_data__SET(&data_, 0, PH.base.pack) ;
        }
        p171_len_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p171_type_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_CommunicationChannel_on_DATA64_171(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA96_172(), &PH);
        p172_len_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p172_type_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)130, (uint8_t)0, (uint8_t)98, (uint8_t)240, (uint8_t)154, (uint8_t)233, (uint8_t)47, (uint8_t)179, (uint8_t)193, (uint8_t)83, (uint8_t)122, (uint8_t)153, (uint8_t)193, (uint8_t)103, (uint8_t)37, (uint8_t)110, (uint8_t)37, (uint8_t)123, (uint8_t)120, (uint8_t)236, (uint8_t)69, (uint8_t)106, (uint8_t)147, (uint8_t)24, (uint8_t)26, (uint8_t)74, (uint8_t)6, (uint8_t)122, (uint8_t)234, (uint8_t)194, (uint8_t)106, (uint8_t)167, (uint8_t)65, (uint8_t)134, (uint8_t)97, (uint8_t)91, (uint8_t)122, (uint8_t)98, (uint8_t)218, (uint8_t)119, (uint8_t)77, (uint8_t)209, (uint8_t)92, (uint8_t)230, (uint8_t)42, (uint8_t)50, (uint8_t)87, (uint8_t)42, (uint8_t)142, (uint8_t)192, (uint8_t)200, (uint8_t)249, (uint8_t)180, (uint8_t)165, (uint8_t)43, (uint8_t)179, (uint8_t)27, (uint8_t)124, (uint8_t)150, (uint8_t)22, (uint8_t)45, (uint8_t)73, (uint8_t)134, (uint8_t)249, (uint8_t)53, (uint8_t)192, (uint8_t)178, (uint8_t)139, (uint8_t)235, (uint8_t)129, (uint8_t)87, (uint8_t)37, (uint8_t)212, (uint8_t)221, (uint8_t)215, (uint8_t)216, (uint8_t)242, (uint8_t)246, (uint8_t)93, (uint8_t)217, (uint8_t)144, (uint8_t)59, (uint8_t)80, (uint8_t)132, (uint8_t)241, (uint8_t)36, (uint8_t)223, (uint8_t)103, (uint8_t)237, (uint8_t)73, (uint8_t)76, (uint8_t)98, (uint8_t)15, (uint8_t)42, (uint8_t)96, (uint8_t)93};
            p172_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DATA96_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RANGEFINDER_173(), &PH);
        p173_distance_SET((float)3.3154503E38F, PH.base.pack) ;
        p173_voltage_SET((float)1.5461225E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RANGEFINDER_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AIRSPEED_AUTOCAL_174(), &PH);
        p174_vx_SET((float) -2.8254908E36F, PH.base.pack) ;
        p174_Pcz_SET((float)3.3781459E38F, PH.base.pack) ;
        p174_state_z_SET((float) -2.0003048E38F, PH.base.pack) ;
        p174_Pby_SET((float) -5.4656525E37F, PH.base.pack) ;
        p174_Pax_SET((float)2.8704682E38F, PH.base.pack) ;
        p174_state_x_SET((float) -2.4271365E38F, PH.base.pack) ;
        p174_vy_SET((float)3.0999807E38F, PH.base.pack) ;
        p174_vz_SET((float) -1.2430219E37F, PH.base.pack) ;
        p174_EAS2TAS_SET((float)3.3208326E38F, PH.base.pack) ;
        p174_state_y_SET((float) -1.7265774E38F, PH.base.pack) ;
        p174_ratio_SET((float) -2.2657522E38F, PH.base.pack) ;
        p174_diff_pressure_SET((float)1.9219365E37F, PH.base.pack) ;
        c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_POINT_175(), &PH);
        p175_flags_SET(e_RALLY_FLAGS_LAND_IMMEDIATELY, PH.base.pack) ;
        p175_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p175_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p175_count_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p175_idx_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p175_land_dir_SET((uint16_t)(uint16_t)57319, PH.base.pack) ;
        p175_break_alt_SET((int16_t)(int16_t) -20233, PH.base.pack) ;
        p175_alt_SET((int16_t)(int16_t) -28009, PH.base.pack) ;
        p175_lng_SET((int32_t)1063137977, PH.base.pack) ;
        p175_lat_SET((int32_t)767257486, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_POINT_175(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_FETCH_POINT_176(), &PH);
        p176_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p176_idx_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p176_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_FETCH_POINT_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COMPASSMOT_STATUS_177(), &PH);
        p177_current_SET((float) -5.42921E37F, PH.base.pack) ;
        p177_CompensationX_SET((float) -2.9497733E38F, PH.base.pack) ;
        p177_throttle_SET((uint16_t)(uint16_t)60338, PH.base.pack) ;
        p177_CompensationZ_SET((float)1.662567E38F, PH.base.pack) ;
        p177_CompensationY_SET((float)3.3197208E38F, PH.base.pack) ;
        p177_interference_SET((uint16_t)(uint16_t)10022, PH.base.pack) ;
        c_CommunicationChannel_on_COMPASSMOT_STATUS_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS2_178(), &PH);
        p178_lat_SET((int32_t)565820019, PH.base.pack) ;
        p178_yaw_SET((float)2.9503133E37F, PH.base.pack) ;
        p178_pitch_SET((float)1.1175339E38F, PH.base.pack) ;
        p178_lng_SET((int32_t)559719448, PH.base.pack) ;
        p178_altitude_SET((float)2.270756E38F, PH.base.pack) ;
        p178_roll_SET((float)1.4585114E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS2_178(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_STATUS_179(), &PH);
        p179_p1_SET((float)2.3013347E38F, PH.base.pack) ;
        p179_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_ERROR, PH.base.pack) ;
        p179_time_usec_SET((uint64_t)1813203923314458843L, PH.base.pack) ;
        p179_p2_SET((float)2.1331694E38F, PH.base.pack) ;
        p179_p3_SET((float)2.5411113E38F, PH.base.pack) ;
        p179_p4_SET((float) -1.0012088E38F, PH.base.pack) ;
        p179_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p179_img_idx_SET((uint16_t)(uint16_t)7270, PH.base.pack) ;
        p179_cam_idx_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_STATUS_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_FEEDBACK_180(), &PH);
        p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_PHOTO, PH.base.pack) ;
        p180_lat_SET((int32_t) -2129802835, PH.base.pack) ;
        p180_pitch_SET((float)3.1334234E38F, PH.base.pack) ;
        p180_img_idx_SET((uint16_t)(uint16_t)49924, PH.base.pack) ;
        p180_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p180_lng_SET((int32_t) -18547997, PH.base.pack) ;
        p180_roll_SET((float)2.742824E37F, PH.base.pack) ;
        p180_foc_len_SET((float) -3.1009594E38F, PH.base.pack) ;
        p180_yaw_SET((float)9.023342E37F, PH.base.pack) ;
        p180_alt_rel_SET((float) -3.1635886E38F, PH.base.pack) ;
        p180_time_usec_SET((uint64_t)1684644520655252709L, PH.base.pack) ;
        p180_alt_msl_SET((float) -1.2652611E38F, PH.base.pack) ;
        p180_cam_idx_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_FEEDBACK_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY2_181(), &PH);
        p181_voltage_SET((uint16_t)(uint16_t)8104, PH.base.pack) ;
        p181_current_battery_SET((int16_t)(int16_t)17857, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY2_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS3_182(), &PH);
        p182_lat_SET((int32_t)1724200803, PH.base.pack) ;
        p182_v1_SET((float) -1.0328906E38F, PH.base.pack) ;
        p182_v4_SET((float) -1.0751439E38F, PH.base.pack) ;
        p182_pitch_SET((float)1.5140253E38F, PH.base.pack) ;
        p182_v2_SET((float) -2.3259068E38F, PH.base.pack) ;
        p182_v3_SET((float) -3.347101E38F, PH.base.pack) ;
        p182_lng_SET((int32_t)58734568, PH.base.pack) ;
        p182_altitude_SET((float) -1.130815E38F, PH.base.pack) ;
        p182_yaw_SET((float)1.7820107E38F, PH.base.pack) ;
        p182_roll_SET((float)2.6145E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS3_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_REQUEST_183(), &PH);
        p183_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p183_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_DATA_BLOCK_184(), &PH);
        p184_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p184_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)106, (uint8_t)14, (uint8_t)188, (uint8_t)65, (uint8_t)232, (uint8_t)154, (uint8_t)224, (uint8_t)82, (uint8_t)44, (uint8_t)80, (uint8_t)45, (uint8_t)219, (uint8_t)79, (uint8_t)8, (uint8_t)33, (uint8_t)4, (uint8_t)91, (uint8_t)176, (uint8_t)172, (uint8_t)248, (uint8_t)230, (uint8_t)234, (uint8_t)168, (uint8_t)79, (uint8_t)73, (uint8_t)218, (uint8_t)233, (uint8_t)200, (uint8_t)211, (uint8_t)70, (uint8_t)43, (uint8_t)66, (uint8_t)93, (uint8_t)205, (uint8_t)143, (uint8_t)225, (uint8_t)243, (uint8_t)229, (uint8_t)48, (uint8_t)13, (uint8_t)87, (uint8_t)58, (uint8_t)80, (uint8_t)120, (uint8_t)84, (uint8_t)31, (uint8_t)122, (uint8_t)100, (uint8_t)15, (uint8_t)72, (uint8_t)17, (uint8_t)68, (uint8_t)46, (uint8_t)28, (uint8_t)254, (uint8_t)25, (uint8_t)186, (uint8_t)243, (uint8_t)209, (uint8_t)229, (uint8_t)47, (uint8_t)234, (uint8_t)252, (uint8_t)1, (uint8_t)145, (uint8_t)68, (uint8_t)107, (uint8_t)188, (uint8_t)132, (uint8_t)22, (uint8_t)26, (uint8_t)248, (uint8_t)144, (uint8_t)11, (uint8_t)89, (uint8_t)118, (uint8_t)203, (uint8_t)235, (uint8_t)60, (uint8_t)225, (uint8_t)199, (uint8_t)176, (uint8_t)147, (uint8_t)152, (uint8_t)104, (uint8_t)255, (uint8_t)191, (uint8_t)212, (uint8_t)107, (uint8_t)107, (uint8_t)244, (uint8_t)246, (uint8_t)224, (uint8_t)98, (uint8_t)164, (uint8_t)226, (uint8_t)96, (uint8_t)179, (uint8_t)174, (uint8_t)199, (uint8_t)152, (uint8_t)102, (uint8_t)230, (uint8_t)159, (uint8_t)74, (uint8_t)235, (uint8_t)202, (uint8_t)188, (uint8_t)229, (uint8_t)8, (uint8_t)199, (uint8_t)152, (uint8_t)52, (uint8_t)184, (uint8_t)127, (uint8_t)107, (uint8_t)232, (uint8_t)220, (uint8_t)114, (uint8_t)253, (uint8_t)35, (uint8_t)250, (uint8_t)51, (uint8_t)248, (uint8_t)107, (uint8_t)2, (uint8_t)185, (uint8_t)102, (uint8_t)18, (uint8_t)118, (uint8_t)235, (uint8_t)240, (uint8_t)98, (uint8_t)224, (uint8_t)144, (uint8_t)164, (uint8_t)137, (uint8_t)58, (uint8_t)44, (uint8_t)239, (uint8_t)100, (uint8_t)137, (uint8_t)66, (uint8_t)252, (uint8_t)214, (uint8_t)89, (uint8_t)228, (uint8_t)79, (uint8_t)61, (uint8_t)177, (uint8_t)156, (uint8_t)190, (uint8_t)88, (uint8_t)192, (uint8_t)129, (uint8_t)245, (uint8_t)210, (uint8_t)7, (uint8_t)220, (uint8_t)163, (uint8_t)89, (uint8_t)188, (uint8_t)146, (uint8_t)186, (uint8_t)222, (uint8_t)106, (uint8_t)255, (uint8_t)236, (uint8_t)240, (uint8_t)228, (uint8_t)25, (uint8_t)70, (uint8_t)7, (uint8_t)17, (uint8_t)74, (uint8_t)121, (uint8_t)35, (uint8_t)165, (uint8_t)65, (uint8_t)121, (uint8_t)13, (uint8_t)15, (uint8_t)238, (uint8_t)60, (uint8_t)78, (uint8_t)58, (uint8_t)50, (uint8_t)227, (uint8_t)110, (uint8_t)170, (uint8_t)201, (uint8_t)175, (uint8_t)199, (uint8_t)123, (uint8_t)204, (uint8_t)237, (uint8_t)196, (uint8_t)53, (uint8_t)56, (uint8_t)123};
            p184_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_BLOCK_STATUS_185(), &PH);
        p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, PH.base.pack) ;
        p185_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p185_seqno_SET((uint32_t)2195765879L, PH.base.pack) ;
        p185_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LED_CONTROL_186(), &PH);
        p186_instance_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        {
            uint8_t custom_bytes[] =  {(uint8_t)59, (uint8_t)100, (uint8_t)211, (uint8_t)86, (uint8_t)246, (uint8_t)206, (uint8_t)232, (uint8_t)51, (uint8_t)77, (uint8_t)252, (uint8_t)187, (uint8_t)122, (uint8_t)237, (uint8_t)47, (uint8_t)163, (uint8_t)69, (uint8_t)190, (uint8_t)188, (uint8_t)62, (uint8_t)83, (uint8_t)225, (uint8_t)130, (uint8_t)68, (uint8_t)91};
            p186_custom_bytes_SET(&custom_bytes, 0, PH.base.pack) ;
        }
        p186_pattern_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p186_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p186_custom_len_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p186_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_CommunicationChannel_on_LED_CONTROL_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_PROGRESS_191(), &PH);
        p191_direction_y_SET((float)7.9344903E37F, PH.base.pack) ;
        p191_attempt_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p191_cal_mask_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p191_direction_z_SET((float) -3.3482805E37F, PH.base.pack) ;
        {
            uint8_t completion_mask[] =  {(uint8_t)112, (uint8_t)54, (uint8_t)26, (uint8_t)73, (uint8_t)235, (uint8_t)38, (uint8_t)48, (uint8_t)77, (uint8_t)200, (uint8_t)208};
            p191_completion_mask_SET(&completion_mask, 0, PH.base.pack) ;
        }
        p191_direction_x_SET((float)2.4797787E38F, PH.base.pack) ;
        p191_compass_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p191_completion_pct_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p191_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_WAITING_TO_START, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_REPORT_192(), &PH);
        p192_ofs_z_SET((float)6.203774E37F, PH.base.pack) ;
        p192_fitness_SET((float) -2.2293881E38F, PH.base.pack) ;
        p192_diag_x_SET((float) -3.3576414E38F, PH.base.pack) ;
        p192_offdiag_z_SET((float) -9.947609E37F, PH.base.pack) ;
        p192_ofs_x_SET((float)2.6810784E38F, PH.base.pack) ;
        p192_diag_y_SET((float)2.1729227E38F, PH.base.pack) ;
        p192_offdiag_x_SET((float)3.039634E38F, PH.base.pack) ;
        p192_ofs_y_SET((float) -1.2385312E38F, PH.base.pack) ;
        p192_cal_mask_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p192_diag_z_SET((float)2.0762151E38F, PH.base.pack) ;
        p192_autosaved_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p192_compass_id_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p192_offdiag_y_SET((float) -1.4144716E38F, PH.base.pack) ;
        p192_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_FAILED, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_REPORT_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EKF_STATUS_REPORT_193(), &PH);
        p193_pos_vert_variance_SET((float) -1.492568E38F, PH.base.pack) ;
        p193_terrain_alt_variance_SET((float) -7.6102476E37F, PH.base.pack) ;
        p193_compass_variance_SET((float) -9.372733E37F, PH.base.pack) ;
        p193_pos_horiz_variance_SET((float)1.087927E37F, PH.base.pack) ;
        p193_flags_SET((e_EKF_STATUS_FLAGS_EKF_POS_VERT_ABS |
                        e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL |
                        e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL |
                        e_EKF_STATUS_FLAGS_EKF_VELOCITY_HORIZ), PH.base.pack) ;
        p193_velocity_variance_SET((float) -2.3364194E38F, PH.base.pack) ;
        c_CommunicationChannel_on_EKF_STATUS_REPORT_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PID_TUNING_194(), &PH);
        p194_FF_SET((float) -1.5631198E37F, PH.base.pack) ;
        p194_desired_SET((float)2.8384587E38F, PH.base.pack) ;
        p194_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_STEER, PH.base.pack) ;
        p194_D_SET((float) -2.8108233E38F, PH.base.pack) ;
        p194_I_SET((float) -1.8125426E38F, PH.base.pack) ;
        p194_P_SET((float) -2.933069E36F, PH.base.pack) ;
        p194_achieved_SET((float)8.573471E37F, PH.base.pack) ;
        c_CommunicationChannel_on_PID_TUNING_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_REPORT_200(), &PH);
        p200_delta_angle_y_SET((float)6.265011E37F, PH.base.pack) ;
        p200_delta_angle_x_SET((float) -2.5342773E38F, PH.base.pack) ;
        p200_delta_velocity_x_SET((float)2.3312277E38F, PH.base.pack) ;
        p200_delta_angle_z_SET((float)1.7512433E38F, PH.base.pack) ;
        p200_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p200_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p200_delta_time_SET((float) -1.852975E37F, PH.base.pack) ;
        p200_delta_velocity_z_SET((float) -1.8079251E38F, PH.base.pack) ;
        p200_delta_velocity_y_SET((float) -3.1055955E38F, PH.base.pack) ;
        p200_joint_az_SET((float)8.332525E37F, PH.base.pack) ;
        p200_joint_roll_SET((float)1.6981836E38F, PH.base.pack) ;
        p200_joint_el_SET((float) -8.3624096E37F, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_REPORT_200(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_CONTROL_201(), &PH);
        p201_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p201_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p201_demanded_rate_y_SET((float)2.4033169E38F, PH.base.pack) ;
        p201_demanded_rate_x_SET((float) -2.4393044E38F, PH.base.pack) ;
        p201_demanded_rate_z_SET((float) -1.3216473E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_CONTROL_201(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_TORQUE_CMD_REPORT_214(), &PH);
        p214_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p214_az_torque_cmd_SET((int16_t)(int16_t) -9174, PH.base.pack) ;
        p214_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p214_rl_torque_cmd_SET((int16_t)(int16_t)12684, PH.base.pack) ;
        p214_el_torque_cmd_SET((int16_t)(int16_t)216, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_HEARTBEAT_215(), &PH);
        p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, PH.base.pack) ;
        p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_SETUP, PH.base.pack) ;
        p215_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_REQUEST_216(), &PH);
        p216_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PHOTO_RESOLUTION, PH.base.pack) ;
        p216_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p216_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_RESPONSE_217(), &PH);
        {
            uint8_t value[] =  {(uint8_t)61, (uint8_t)176, (uint8_t)79, (uint8_t)148};
            p217_value_SET(&value, 0, PH.base.pack) ;
        }
        p217_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_CAPTURE_MODE, PH.base.pack) ;
        p217_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_REQUEST_218(), &PH);
        p218_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN, PH.base.pack) ;
        p218_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p218_target_component_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        {
            uint8_t value[] =  {(uint8_t)24, (uint8_t)125, (uint8_t)98, (uint8_t)53};
            p218_value_SET(&value, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_RESPONSE_219(), &PH);
        p219_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_TIME, PH.base.pack) ;
        p219_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_FAILED, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RPM_226(), &PH);
        p226_rpm1_SET((float)2.5405391E37F, PH.base.pack) ;
        p226_rpm2_SET((float)3.2198364E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RPM_226(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_ratio_SET((float) -1.41696E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)6.9205397E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.6826913E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -3.0592784E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)52909230062959987L, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT), PH.base.pack) ;
        p230_mag_ratio_SET((float)3.3586106E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.8920403E36F, PH.base.pack) ;
        p230_tas_ratio_SET((float)8.0126065E37F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -9.806025E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_time_usec_SET((uint64_t)4328196724106380199L, PH.base.pack) ;
        p231_wind_z_SET((float) -2.6481588E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -4.3740325E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -8.79237E37F, PH.base.pack) ;
        p231_var_horiz_SET((float)3.9489253E37F, PH.base.pack) ;
        p231_wind_y_SET((float) -2.4582615E37F, PH.base.pack) ;
        p231_var_vert_SET((float)1.6474992E38F, PH.base.pack) ;
        p231_wind_x_SET((float)9.895936E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.0598673E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_horiz_accuracy_SET((float) -2.1015633E38F, PH.base.pack) ;
        p232_vn_SET((float)2.9899034E38F, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY), PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2688334277L, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -2.6597072E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)4023620504202445699L, PH.base.pack) ;
        p232_ve_SET((float) -2.5006201E38F, PH.base.pack) ;
        p232_vd_SET((float)2.5334204E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)29818, PH.base.pack) ;
        p232_vdop_SET((float) -2.3450037E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -2.7800649E38F, PH.base.pack) ;
        p232_lat_SET((int32_t)52374527, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p232_lon_SET((int32_t)219281835, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p232_alt_SET((float)8.0958997E37F, PH.base.pack) ;
        p232_hdop_SET((float)2.679998E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)39, (uint8_t)59, (uint8_t)114, (uint8_t)55, (uint8_t)236, (uint8_t)226, (uint8_t)52, (uint8_t)14, (uint8_t)128, (uint8_t)153, (uint8_t)208, (uint8_t)154, (uint8_t)255, (uint8_t)117, (uint8_t)234, (uint8_t)240, (uint8_t)153, (uint8_t)219, (uint8_t)41, (uint8_t)163, (uint8_t)183, (uint8_t)114, (uint8_t)130, (uint8_t)185, (uint8_t)154, (uint8_t)219, (uint8_t)219, (uint8_t)33, (uint8_t)184, (uint8_t)61, (uint8_t)194, (uint8_t)142, (uint8_t)92, (uint8_t)186, (uint8_t)79, (uint8_t)51, (uint8_t)136, (uint8_t)86, (uint8_t)108, (uint8_t)3, (uint8_t)70, (uint8_t)176, (uint8_t)35, (uint8_t)230, (uint8_t)115, (uint8_t)127, (uint8_t)107, (uint8_t)101, (uint8_t)189, (uint8_t)106, (uint8_t)222, (uint8_t)231, (uint8_t)254, (uint8_t)65, (uint8_t)134, (uint8_t)233, (uint8_t)13, (uint8_t)250, (uint8_t)231, (uint8_t)104, (uint8_t)17, (uint8_t)136, (uint8_t)81, (uint8_t)164, (uint8_t)101, (uint8_t)67, (uint8_t)217, (uint8_t)148, (uint8_t)218, (uint8_t)3, (uint8_t)76, (uint8_t)187, (uint8_t)245, (uint8_t)75, (uint8_t)67, (uint8_t)133, (uint8_t)181, (uint8_t)65, (uint8_t)164, (uint8_t)127, (uint8_t)168, (uint8_t)229, (uint8_t)243, (uint8_t)48, (uint8_t)24, (uint8_t)213, (uint8_t)44, (uint8_t)101, (uint8_t)19, (uint8_t)250, (uint8_t)9, (uint8_t)89, (uint8_t)12, (uint8_t)61, (uint8_t)197, (uint8_t)128, (uint8_t)131, (uint8_t)193, (uint8_t)239, (uint8_t)45, (uint8_t)29, (uint8_t)106, (uint8_t)181, (uint8_t)187, (uint8_t)146, (uint8_t)178, (uint8_t)167, (uint8_t)203, (uint8_t)64, (uint8_t)60, (uint8_t)107, (uint8_t)116, (uint8_t)15, (uint8_t)202, (uint8_t)139, (uint8_t)22, (uint8_t)101, (uint8_t)151, (uint8_t)165, (uint8_t)200, (uint8_t)182, (uint8_t)92, (uint8_t)162, (uint8_t)134, (uint8_t)108, (uint8_t)225, (uint8_t)254, (uint8_t)134, (uint8_t)118, (uint8_t)218, (uint8_t)143, (uint8_t)43, (uint8_t)210, (uint8_t)252, (uint8_t)209, (uint8_t)19, (uint8_t)83, (uint8_t)102, (uint8_t)95, (uint8_t)186, (uint8_t)185, (uint8_t)96, (uint8_t)126, (uint8_t)44, (uint8_t)122, (uint8_t)186, (uint8_t)134, (uint8_t)130, (uint8_t)81, (uint8_t)127, (uint8_t)167, (uint8_t)131, (uint8_t)221, (uint8_t)17, (uint8_t)28, (uint8_t)226, (uint8_t)123, (uint8_t)43, (uint8_t)62, (uint8_t)199, (uint8_t)89, (uint8_t)71, (uint8_t)84, (uint8_t)235, (uint8_t)195, (uint8_t)11, (uint8_t)168, (uint8_t)71, (uint8_t)157, (uint8_t)133, (uint8_t)9, (uint8_t)154, (uint8_t)97, (uint8_t)238, (uint8_t)78, (uint8_t)225, (uint8_t)49, (uint8_t)248, (uint8_t)2, (uint8_t)39};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
        p234_throttle_SET((int8_t)(int8_t) -100, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -5613, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)13, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)47491, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)10029, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -24214, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1135753192, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -21078, PH.base.pack) ;
        p234_latitude_SET((int32_t)1778097392, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -68, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1505511269L, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)15800, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -6178, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -9, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED), PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)7041652304032582572L, PH.base.pack) ;
        p241_vibration_z_SET((float)2.6878312E38F, PH.base.pack) ;
        p241_vibration_y_SET((float)8.534597E37F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)274702238L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1377431044L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3561449687L, PH.base.pack) ;
        p241_vibration_x_SET((float) -3.2309302E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
        p242_altitude_SET((int32_t)1939014740, PH.base.pack) ;
        p242_approach_z_SET((float)3.365042E38F, PH.base.pack) ;
        p242_z_SET((float)2.48446E38F, PH.base.pack) ;
        p242_approach_x_SET((float)2.0599983E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1642709340, PH.base.pack) ;
        p242_longitude_SET((int32_t)973944531, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)2376926908481052774L, &PH) ;
        p242_y_SET((float)9.027582E37F, PH.base.pack) ;
        p242_approach_y_SET((float) -2.4220353E38F, PH.base.pack) ;
        p242_x_SET((float)2.9702757E37F, PH.base.pack) ;
        {
            float q[] =  {2.3875956E38F, 1.4188326E37F, -7.472671E37F, -2.3242964E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_x_SET((float) -3.0400724E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)9191673220426437305L, &PH) ;
        p243_approach_y_SET((float) -1.5507856E38F, PH.base.pack) ;
        {
            float q[] =  {-1.5019237E37F, -3.076273E38F, 3.1220257E38F, -2.2700622E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_latitude_SET((int32_t)445609559, PH.base.pack) ;
        p243_approach_x_SET((float) -2.0329438E38F, PH.base.pack) ;
        p243_z_SET((float)3.0712452E38F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.9058167E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -2095471755, PH.base.pack) ;
        p243_altitude_SET((int32_t) -130085780, PH.base.pack) ;
        p243_y_SET((float)2.4786137E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1942720437, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)34025, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_lat_SET((int32_t) -519202483, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)57926, PH.base.pack) ;
        p246_lon_SET((int32_t) -801970908, PH.base.pack) ;
        {
            char16_t* callsign = u"qai";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_squawk_SET((uint16_t)(uint16_t)50709, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3777895889L, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED), PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)2345, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p246_altitude_SET((int32_t) -684295046, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)61073, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
        p247_time_to_minimum_delta_SET((float) -1.2880387E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_id_SET((uint32_t)3992920116L, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)3.0938771E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -2.391154E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_system_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)49486, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)203, (uint8_t)233, (uint8_t)41, (uint8_t)66, (uint8_t)232, (uint8_t)255, (uint8_t)192, (uint8_t)239, (uint8_t)182, (uint8_t)173, (uint8_t)139, (uint8_t)190, (uint8_t)218, (uint8_t)26, (uint8_t)12, (uint8_t)239, (uint8_t)222, (uint8_t)7, (uint8_t)133, (uint8_t)111, (uint8_t)107, (uint8_t)121, (uint8_t)57, (uint8_t)155, (uint8_t)91, (uint8_t)15, (uint8_t)77, (uint8_t)50, (uint8_t)197, (uint8_t)83, (uint8_t)202, (uint8_t)216, (uint8_t)239, (uint8_t)124, (uint8_t)152, (uint8_t)230, (uint8_t)214, (uint8_t)107, (uint8_t)86, (uint8_t)71, (uint8_t)143, (uint8_t)220, (uint8_t)64, (uint8_t)33, (uint8_t)140, (uint8_t)141, (uint8_t)202, (uint8_t)178, (uint8_t)48, (uint8_t)4, (uint8_t)59, (uint8_t)129, (uint8_t)39, (uint8_t)88, (uint8_t)36, (uint8_t)197, (uint8_t)170, (uint8_t)175, (uint8_t)188, (uint8_t)53, (uint8_t)206, (uint8_t)145, (uint8_t)250, (uint8_t)18, (uint8_t)57, (uint8_t)133, (uint8_t)18, (uint8_t)73, (uint8_t)241, (uint8_t)129, (uint8_t)191, (uint8_t)220, (uint8_t)38, (uint8_t)56, (uint8_t)101, (uint8_t)186, (uint8_t)96, (uint8_t)148, (uint8_t)160, (uint8_t)84, (uint8_t)154, (uint8_t)119, (uint8_t)53, (uint8_t)240, (uint8_t)74, (uint8_t)177, (uint8_t)133, (uint8_t)15, (uint8_t)252, (uint8_t)18, (uint8_t)228, (uint8_t)199, (uint8_t)21, (uint8_t)128, (uint8_t)100, (uint8_t)174, (uint8_t)93, (uint8_t)21, (uint8_t)137, (uint8_t)201, (uint8_t)0, (uint8_t)136, (uint8_t)197, (uint8_t)37, (uint8_t)211, (uint8_t)90, (uint8_t)40, (uint8_t)176, (uint8_t)92, (uint8_t)1, (uint8_t)87, (uint8_t)130, (uint8_t)150, (uint8_t)41, (uint8_t)114, (uint8_t)128, (uint8_t)218, (uint8_t)198, (uint8_t)26, (uint8_t)67, (uint8_t)30, (uint8_t)0, (uint8_t)22, (uint8_t)139, (uint8_t)229, (uint8_t)143, (uint8_t)160, (uint8_t)205, (uint8_t)218, (uint8_t)23, (uint8_t)32, (uint8_t)234, (uint8_t)115, (uint8_t)26, (uint8_t)181, (uint8_t)129, (uint8_t)190, (uint8_t)69, (uint8_t)14, (uint8_t)228, (uint8_t)76, (uint8_t)110, (uint8_t)239, (uint8_t)90, (uint8_t)198, (uint8_t)235, (uint8_t)42, (uint8_t)245, (uint8_t)4, (uint8_t)234, (uint8_t)94, (uint8_t)39, (uint8_t)168, (uint8_t)91, (uint8_t)201, (uint8_t)208, (uint8_t)120, (uint8_t)180, (uint8_t)100, (uint8_t)159, (uint8_t)185, (uint8_t)92, (uint8_t)1, (uint8_t)117, (uint8_t)83, (uint8_t)187, (uint8_t)88, (uint8_t)172, (uint8_t)139, (uint8_t)66, (uint8_t)112, (uint8_t)227, (uint8_t)215, (uint8_t)41, (uint8_t)55, (uint8_t)121, (uint8_t)246, (uint8_t)123, (uint8_t)233, (uint8_t)219, (uint8_t)21, (uint8_t)93, (uint8_t)190, (uint8_t)113, (uint8_t)147, (uint8_t)120, (uint8_t)180, (uint8_t)233, (uint8_t)217, (uint8_t)114, (uint8_t)116, (uint8_t)222, (uint8_t)185, (uint8_t)91, (uint8_t)119, (uint8_t)110, (uint8_t)113, (uint8_t)226, (uint8_t)212, (uint8_t)232, (uint8_t)205, (uint8_t)132, (uint8_t)162, (uint8_t)34, (uint8_t)191, (uint8_t)230, (uint8_t)146, (uint8_t)2, (uint8_t)133, (uint8_t)72, (uint8_t)142, (uint8_t)148, (uint8_t)196, (uint8_t)248, (uint8_t)192, (uint8_t)255, (uint8_t)15, (uint8_t)72, (uint8_t)255, (uint8_t)108, (uint8_t)46, (uint8_t)108, (uint8_t)135, (uint8_t)57, (uint8_t)52, (uint8_t)37, (uint8_t)164, (uint8_t)114, (uint8_t)37, (uint8_t)25, (uint8_t)200, (uint8_t)57, (uint8_t)67, (uint8_t)247, (uint8_t)93, (uint8_t)104, (uint8_t)226, (uint8_t)69, (uint8_t)171, (uint8_t)250, (uint8_t)77, (uint8_t)48, (uint8_t)140, (uint8_t)190, (uint8_t)0, (uint8_t)91, (uint8_t)229, (uint8_t)8, (uint8_t)168};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)56142, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -9, (int8_t)19, (int8_t)22, (int8_t)100, (int8_t) -43, (int8_t) -35, (int8_t) -79, (int8_t) -107, (int8_t)46, (int8_t) -9, (int8_t)50, (int8_t) -127, (int8_t)67, (int8_t) -99, (int8_t)22, (int8_t) -72, (int8_t) -80, (int8_t) -2, (int8_t)1, (int8_t) -2, (int8_t) -89, (int8_t)84, (int8_t) -14, (int8_t)36, (int8_t) -54, (int8_t) -84, (int8_t)34, (int8_t) -123, (int8_t)71, (int8_t) -31, (int8_t)114, (int8_t)74};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float) -3.0901707E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)7688063479593977055L, PH.base.pack) ;
        p250_z_SET((float)2.6637484E37F, PH.base.pack) ;
        {
            char16_t* name = u"hQ";
            p250_name_SET_(name, &PH) ;
        }
        p250_y_SET((float)3.1985388E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"lplsbsneo";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float)1.1333921E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)1657850067L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)3827153775L, PH.base.pack) ;
        p252_value_SET((int32_t) -295342900, PH.base.pack) ;
        {
            char16_t* name = u"siac";
            p252_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"ecavpprokasxemzajqljhijrlkuvwfdhniczjtd";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p254_value_SET((float)7.3485016E37F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)4257574236L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)2788485417723730785L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)121, (uint8_t)1, (uint8_t)103, (uint8_t)166, (uint8_t)32, (uint8_t)94, (uint8_t)28, (uint8_t)145, (uint8_t)126, (uint8_t)139, (uint8_t)89, (uint8_t)180, (uint8_t)34, (uint8_t)90, (uint8_t)96, (uint8_t)204, (uint8_t)66, (uint8_t)18, (uint8_t)150, (uint8_t)64, (uint8_t)25, (uint8_t)209, (uint8_t)70, (uint8_t)91, (uint8_t)58, (uint8_t)156, (uint8_t)251, (uint8_t)163, (uint8_t)83, (uint8_t)27, (uint8_t)21, (uint8_t)12};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)1173199757L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)323105698L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        {
            char16_t* tune = u"xkJWrtdabhbdzozwszsjdIjvqhcv";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_focal_length_SET((float)1.9191382E38F, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)3985773577L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"KelyatfaofWXid";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)3508, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)193, (uint8_t)47, (uint8_t)87, (uint8_t)220, (uint8_t)152, (uint8_t)126, (uint8_t)98, (uint8_t)100, (uint8_t)251, (uint8_t)133, (uint8_t)121, (uint8_t)167, (uint8_t)171, (uint8_t)69, (uint8_t)170, (uint8_t)160, (uint8_t)74, (uint8_t)50, (uint8_t)118, (uint8_t)229, (uint8_t)7, (uint8_t)129, (uint8_t)71, (uint8_t)26, (uint8_t)136, (uint8_t)199, (uint8_t)88, (uint8_t)33, (uint8_t)191, (uint8_t)1, (uint8_t)88, (uint8_t)129};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_v_SET((uint16_t)(uint16_t)31462, PH.base.pack) ;
        p259_sensor_size_h_SET((float)2.4055551E37F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)4156, PH.base.pack) ;
        p259_sensor_size_v_SET((float)7.7722263E36F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)1, (uint8_t)194, (uint8_t)150, (uint8_t)193, (uint8_t)237, (uint8_t)168, (uint8_t)192, (uint8_t)76, (uint8_t)201, (uint8_t)186, (uint8_t)228, (uint8_t)20, (uint8_t)80, (uint8_t)173, (uint8_t)63, (uint8_t)160, (uint8_t)115, (uint8_t)153, (uint8_t)61, (uint8_t)138, (uint8_t)205, (uint8_t)228, (uint8_t)149, (uint8_t)150, (uint8_t)236, (uint8_t)225, (uint8_t)55, (uint8_t)227, (uint8_t)211, (uint8_t)28, (uint8_t)197, (uint8_t)108};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO), PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)1520538778L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)402655530L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_status_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p261_write_speed_SET((float)1.8458352E38F, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.0662128E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1973718645L, PH.base.pack) ;
        p261_read_speed_SET((float) -6.154576E36F, PH.base.pack) ;
        p261_total_capacity_SET((float) -3.4016152E38F, PH.base.pack) ;
        p261_available_capacity_SET((float) -2.689514E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_time_boot_ms_SET((uint32_t)4179335354L, PH.base.pack) ;
        p262_image_interval_SET((float)1.3469536E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2249165182L, PH.base.pack) ;
        p262_available_capacity_SET((float)2.475028E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lon_SET((int32_t)467061498, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1043770951L, PH.base.pack) ;
        {
            float q[] =  {-6.0729205E37F, -1.7968335E38F, 2.6211556E38F, -7.389404E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_alt_SET((int32_t) -22072838, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)319974525447732436L, PH.base.pack) ;
        {
            char16_t* file_url = u"qkhrapsQufxTgsakspfanismomtjfeurybJpLuEafmwdnPryovjpovZoiyGtoczoykujdcvsnodsrfcpdbpvrglEPhwhjvRXdimrdrcgprmtttixPvactoddtkmQssVhcemrkymmdThkseHSmokiyvjznsmcVtkiQoimc";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_lat_SET((int32_t) -1923781458, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p263_image_index_SET((int32_t) -478551698, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)55, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)434045990, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)4823804155354855948L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2845336579L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)9103901080937652980L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)329063277525439225L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float) -4.9692517E37F, PH.base.pack) ;
        p265_roll_SET((float)1.9474097E38F, PH.base.pack) ;
        p265_pitch_SET((float)1.949838E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1688149300L, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)190, (uint8_t)147, (uint8_t)94, (uint8_t)150, (uint8_t)47, (uint8_t)203, (uint8_t)22, (uint8_t)223, (uint8_t)170, (uint8_t)154, (uint8_t)9, (uint8_t)226, (uint8_t)137, (uint8_t)170, (uint8_t)90, (uint8_t)224, (uint8_t)157, (uint8_t)141, (uint8_t)66, (uint8_t)135, (uint8_t)206, (uint8_t)164, (uint8_t)139, (uint8_t)244, (uint8_t)157, (uint8_t)213, (uint8_t)59, (uint8_t)220, (uint8_t)82, (uint8_t)54, (uint8_t)214, (uint8_t)237, (uint8_t)60, (uint8_t)196, (uint8_t)202, (uint8_t)204, (uint8_t)205, (uint8_t)163, (uint8_t)55, (uint8_t)187, (uint8_t)130, (uint8_t)193, (uint8_t)121, (uint8_t)3, (uint8_t)195, (uint8_t)164, (uint8_t)128, (uint8_t)232, (uint8_t)96, (uint8_t)241, (uint8_t)203, (uint8_t)58, (uint8_t)148, (uint8_t)207, (uint8_t)66, (uint8_t)156, (uint8_t)150, (uint8_t)136, (uint8_t)230, (uint8_t)170, (uint8_t)134, (uint8_t)150, (uint8_t)130, (uint8_t)163, (uint8_t)3, (uint8_t)181, (uint8_t)148, (uint8_t)90, (uint8_t)223, (uint8_t)133, (uint8_t)57, (uint8_t)206, (uint8_t)16, (uint8_t)12, (uint8_t)109, (uint8_t)18, (uint8_t)79, (uint8_t)89, (uint8_t)16, (uint8_t)137, (uint8_t)86, (uint8_t)169, (uint8_t)21, (uint8_t)50, (uint8_t)43, (uint8_t)211, (uint8_t)179, (uint8_t)12, (uint8_t)22, (uint8_t)171, (uint8_t)126, (uint8_t)111, (uint8_t)126, (uint8_t)162, (uint8_t)45, (uint8_t)74, (uint8_t)109, (uint8_t)46, (uint8_t)37, (uint8_t)149, (uint8_t)43, (uint8_t)125, (uint8_t)153, (uint8_t)23, (uint8_t)198, (uint8_t)80, (uint8_t)214, (uint8_t)136, (uint8_t)220, (uint8_t)50, (uint8_t)151, (uint8_t)190, (uint8_t)172, (uint8_t)164, (uint8_t)169, (uint8_t)187, (uint8_t)30, (uint8_t)236, (uint8_t)88, (uint8_t)110, (uint8_t)60, (uint8_t)120, (uint8_t)5, (uint8_t)26, (uint8_t)134, (uint8_t)209, (uint8_t)82, (uint8_t)31, (uint8_t)236, (uint8_t)220, (uint8_t)192, (uint8_t)62, (uint8_t)80, (uint8_t)135, (uint8_t)240, (uint8_t)117, (uint8_t)166, (uint8_t)234, (uint8_t)13, (uint8_t)142, (uint8_t)197, (uint8_t)71, (uint8_t)5, (uint8_t)2, (uint8_t)46, (uint8_t)116, (uint8_t)114, (uint8_t)8, (uint8_t)246, (uint8_t)111, (uint8_t)160, (uint8_t)247, (uint8_t)34, (uint8_t)102, (uint8_t)10, (uint8_t)234, (uint8_t)53, (uint8_t)98, (uint8_t)176, (uint8_t)54, (uint8_t)55, (uint8_t)138, (uint8_t)20, (uint8_t)68, (uint8_t)56, (uint8_t)3, (uint8_t)218, (uint8_t)35, (uint8_t)26, (uint8_t)203, (uint8_t)71, (uint8_t)162, (uint8_t)213, (uint8_t)190, (uint8_t)252, (uint8_t)77, (uint8_t)187, (uint8_t)1, (uint8_t)1, (uint8_t)52, (uint8_t)94, (uint8_t)200, (uint8_t)240, (uint8_t)232, (uint8_t)220, (uint8_t)167, (uint8_t)35, (uint8_t)163, (uint8_t)231, (uint8_t)0, (uint8_t)74, (uint8_t)202, (uint8_t)178, (uint8_t)238, (uint8_t)53, (uint8_t)25, (uint8_t)119, (uint8_t)60, (uint8_t)88, (uint8_t)159, (uint8_t)196, (uint8_t)179, (uint8_t)6, (uint8_t)23, (uint8_t)86, (uint8_t)163, (uint8_t)137, (uint8_t)103, (uint8_t)191, (uint8_t)254, (uint8_t)201, (uint8_t)54, (uint8_t)202, (uint8_t)218, (uint8_t)57, (uint8_t)2, (uint8_t)24, (uint8_t)225, (uint8_t)206, (uint8_t)190, (uint8_t)160, (uint8_t)100, (uint8_t)182, (uint8_t)220, (uint8_t)211, (uint8_t)172, (uint8_t)30, (uint8_t)237, (uint8_t)13, (uint8_t)54, (uint8_t)192, (uint8_t)101, (uint8_t)195, (uint8_t)238, (uint8_t)150, (uint8_t)41, (uint8_t)12, (uint8_t)65, (uint8_t)10, (uint8_t)113, (uint8_t)214, (uint8_t)190, (uint8_t)110, (uint8_t)7, (uint8_t)210, (uint8_t)193, (uint8_t)27, (uint8_t)92, (uint8_t)153};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)37462, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)79, (uint8_t)94, (uint8_t)201, (uint8_t)36, (uint8_t)228, (uint8_t)247, (uint8_t)122, (uint8_t)189, (uint8_t)233, (uint8_t)223, (uint8_t)140, (uint8_t)217, (uint8_t)72, (uint8_t)168, (uint8_t)98, (uint8_t)245, (uint8_t)159, (uint8_t)98, (uint8_t)128, (uint8_t)235, (uint8_t)54, (uint8_t)19, (uint8_t)44, (uint8_t)202, (uint8_t)236, (uint8_t)4, (uint8_t)90, (uint8_t)183, (uint8_t)70, (uint8_t)84, (uint8_t)87, (uint8_t)189, (uint8_t)254, (uint8_t)13, (uint8_t)195, (uint8_t)152, (uint8_t)119, (uint8_t)141, (uint8_t)30, (uint8_t)68, (uint8_t)18, (uint8_t)192, (uint8_t)162, (uint8_t)156, (uint8_t)23, (uint8_t)187, (uint8_t)73, (uint8_t)44, (uint8_t)181, (uint8_t)91, (uint8_t)223, (uint8_t)5, (uint8_t)60, (uint8_t)124, (uint8_t)77, (uint8_t)234, (uint8_t)131, (uint8_t)178, (uint8_t)82, (uint8_t)212, (uint8_t)105, (uint8_t)95, (uint8_t)219, (uint8_t)53, (uint8_t)100, (uint8_t)141, (uint8_t)62, (uint8_t)119, (uint8_t)41, (uint8_t)84, (uint8_t)66, (uint8_t)220, (uint8_t)62, (uint8_t)168, (uint8_t)49, (uint8_t)125, (uint8_t)219, (uint8_t)246, (uint8_t)213, (uint8_t)133, (uint8_t)67, (uint8_t)231, (uint8_t)90, (uint8_t)250, (uint8_t)159, (uint8_t)239, (uint8_t)97, (uint8_t)104, (uint8_t)78, (uint8_t)143, (uint8_t)86, (uint8_t)118, (uint8_t)82, (uint8_t)241, (uint8_t)37, (uint8_t)161, (uint8_t)38, (uint8_t)139, (uint8_t)42, (uint8_t)201, (uint8_t)100, (uint8_t)3, (uint8_t)253, (uint8_t)23, (uint8_t)124, (uint8_t)255, (uint8_t)186, (uint8_t)104, (uint8_t)136, (uint8_t)92, (uint8_t)21, (uint8_t)128, (uint8_t)66, (uint8_t)189, (uint8_t)72, (uint8_t)80, (uint8_t)78, (uint8_t)9, (uint8_t)225, (uint8_t)77, (uint8_t)196, (uint8_t)140, (uint8_t)107, (uint8_t)61, (uint8_t)94, (uint8_t)206, (uint8_t)254, (uint8_t)44, (uint8_t)71, (uint8_t)228, (uint8_t)193, (uint8_t)149, (uint8_t)59, (uint8_t)172, (uint8_t)25, (uint8_t)27, (uint8_t)43, (uint8_t)115, (uint8_t)213, (uint8_t)131, (uint8_t)44, (uint8_t)215, (uint8_t)24, (uint8_t)152, (uint8_t)0, (uint8_t)249, (uint8_t)78, (uint8_t)251, (uint8_t)24, (uint8_t)23, (uint8_t)15, (uint8_t)170, (uint8_t)254, (uint8_t)32, (uint8_t)122, (uint8_t)77, (uint8_t)68, (uint8_t)55, (uint8_t)165, (uint8_t)38, (uint8_t)34, (uint8_t)224, (uint8_t)116, (uint8_t)55, (uint8_t)144, (uint8_t)140, (uint8_t)28, (uint8_t)70, (uint8_t)208, (uint8_t)27, (uint8_t)161, (uint8_t)233, (uint8_t)199, (uint8_t)54, (uint8_t)212, (uint8_t)189, (uint8_t)224, (uint8_t)222, (uint8_t)204, (uint8_t)245, (uint8_t)234, (uint8_t)156, (uint8_t)115, (uint8_t)109, (uint8_t)137, (uint8_t)176, (uint8_t)168, (uint8_t)82, (uint8_t)10, (uint8_t)171, (uint8_t)11, (uint8_t)215, (uint8_t)194, (uint8_t)224, (uint8_t)62, (uint8_t)178, (uint8_t)27, (uint8_t)232, (uint8_t)161, (uint8_t)97, (uint8_t)145, (uint8_t)2, (uint8_t)39, (uint8_t)214, (uint8_t)12, (uint8_t)124, (uint8_t)255, (uint8_t)183, (uint8_t)127, (uint8_t)60, (uint8_t)38, (uint8_t)29, (uint8_t)107, (uint8_t)33, (uint8_t)71, (uint8_t)2, (uint8_t)19, (uint8_t)174, (uint8_t)19, (uint8_t)102, (uint8_t)11, (uint8_t)98, (uint8_t)106, (uint8_t)58, (uint8_t)54, (uint8_t)75, (uint8_t)28, (uint8_t)121, (uint8_t)144, (uint8_t)156, (uint8_t)78, (uint8_t)56, (uint8_t)100, (uint8_t)104, (uint8_t)173, (uint8_t)227, (uint8_t)29, (uint8_t)176, (uint8_t)198, (uint8_t)8, (uint8_t)117, (uint8_t)0, (uint8_t)98, (uint8_t)200, (uint8_t)26, (uint8_t)57, (uint8_t)209, (uint8_t)247, (uint8_t)228};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)22846, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)60317, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_resolution_h_SET((uint16_t)(uint16_t)6829, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        {
            char16_t* uri = u"JqycwjatydqhjmmuOqpBqavphjtpnlnbzOpyvvujuxwksqdumj";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_v_SET((uint16_t)(uint16_t)12961, PH.base.pack) ;
        p269_framerate_SET((float)3.0073495E38F, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)2781234573L, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)34521, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_bitrate_SET((uint32_t)3468219179L, PH.base.pack) ;
        p270_framerate_SET((float) -2.927514E38F, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)26029, PH.base.pack) ;
        {
            char16_t* uri = u"uwulrpwpftfogytyoeSrMwWkkfwimfhszvmssghbeVkvngfjhnoecabvpitewdFlkOxdnibmvyodx";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)26190, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)754, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"zwjkqigje";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"jKlknzbazljcnCzjzDowawGnybbzvb";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)10176, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)8698, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)63, (uint8_t)2, (uint8_t)69, (uint8_t)186, (uint8_t)83, (uint8_t)97, (uint8_t)88, (uint8_t)179};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)48404, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)174, (uint8_t)155, (uint8_t)168, (uint8_t)149, (uint8_t)132, (uint8_t)181, (uint8_t)243, (uint8_t)171};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)3179375016L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)39174, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)6425747654001899098L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_time_usec_SET((uint64_t)5576826338931582641L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3470672937L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)3026369564L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)228, (uint8_t)87, (uint8_t)159, (uint8_t)81, (uint8_t)181, (uint8_t)125, (uint8_t)169, (uint8_t)90, (uint8_t)153, (uint8_t)166, (uint8_t)134, (uint8_t)26, (uint8_t)30, (uint8_t)206, (uint8_t)112, (uint8_t)137};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        {
            char16_t* name = u"e";
            p311_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        {
            char16_t* param_id = u"yjolcQwuwehqhzm";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t) -12556, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)42812, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)19394, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_value = u"WrxfKwtidsejccadcufgeuzaazwicdrwmcftcVyisaebgjtnxonulymvhYv";
            p322_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"pLScxtymfGcqzrog";
            p322_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_value = u"ihbTqflvkJjUhjdCxesfkysfgmqyymuUxddIlqkzfrxFlpcnwRlsckmlHvcuthvianVg";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        {
            char16_t* param_id = u"uChjljRisbgdaxa";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
        {
            char16_t* param_id = u"cjnvdkjliJUfu";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_FAILED, PH.base.pack) ;
        {
            char16_t* param_value = u"NuuKHnfglkrdcRgqaGWlccflvhzwxhmhqkuMfqfdQosqjtommmqgmsjglkhYxvrxzchuLofoigvaaoapxhdpnmjlqycljTlnoaxbvnrdwGnmkpgIbkRrvdtwwifxi";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)9192465739291422823L, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)37679, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)15613, (uint16_t)15202, (uint16_t)62238, (uint16_t)7975, (uint16_t)20051, (uint16_t)11230, (uint16_t)10182, (uint16_t)26394, (uint16_t)20684, (uint16_t)51889, (uint16_t)36591, (uint16_t)51880, (uint16_t)22657, (uint16_t)64294, (uint16_t)29696, (uint16_t)49426, (uint16_t)44360, (uint16_t)12899, (uint16_t)65106, (uint16_t)21066, (uint16_t)4291, (uint16_t)46623, (uint16_t)32439, (uint16_t)62127, (uint16_t)23289, (uint16_t)54374, (uint16_t)26111, (uint16_t)28039, (uint16_t)58131, (uint16_t)28081, (uint16_t)33708, (uint16_t)676, (uint16_t)32240, (uint16_t)26951, (uint16_t)12467, (uint16_t)19316, (uint16_t)37183, (uint16_t)42196, (uint16_t)42496, (uint16_t)50110, (uint16_t)34139, (uint16_t)14967, (uint16_t)41809, (uint16_t)19676, (uint16_t)60771, (uint16_t)6909, (uint16_t)40560, (uint16_t)35070, (uint16_t)33799, (uint16_t)41008, (uint16_t)6829, (uint16_t)8821, (uint16_t)29265, (uint16_t)46470, (uint16_t)7380, (uint16_t)58268, (uint16_t)46147, (uint16_t)22927, (uint16_t)26071, (uint16_t)42500, (uint16_t)19571, (uint16_t)17443, (uint16_t)13407, (uint16_t)39583, (uint16_t)17920, (uint16_t)23665, (uint16_t)1340, (uint16_t)54468, (uint16_t)11323, (uint16_t)8394, (uint16_t)34890, (uint16_t)63208};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_max_distance_SET((uint16_t)(uint16_t)39796, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
        p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED, PH.base.pack) ;
        p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA, PH.base.pack) ;
        p10001_ICAO_SET((uint32_t)1163262119L, PH.base.pack) ;
        {
            char16_t* callsign = u"gt";
            p10001_callsign_SET_(callsign, &PH) ;
        }
        p10001_stallSpeed_SET((uint16_t)(uint16_t)646, PH.base.pack) ;
        p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, PH.base.pack) ;
        p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, PH.base.pack) ;
        p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
        p10002_VelEW_SET((int16_t)(int16_t) -1301, PH.base.pack) ;
        p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D, PH.base.pack) ;
        p10002_velNS_SET((int16_t)(int16_t)886, PH.base.pack) ;
        p10002_numSats_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p10002_accuracyVert_SET((uint16_t)(uint16_t)49387, PH.base.pack) ;
        p10002_gpsLat_SET((int32_t) -1602972662, PH.base.pack) ;
        p10002_gpsAlt_SET((int32_t)1733001153, PH.base.pack) ;
        p10002_velVert_SET((int16_t)(int16_t)10302, PH.base.pack) ;
        p10002_accuracyHor_SET((uint32_t)907226666L, PH.base.pack) ;
        p10002_accuracyVel_SET((uint16_t)(uint16_t)8796, PH.base.pack) ;
        p10002_utcTime_SET((uint32_t)4007653121L, PH.base.pack) ;
        p10002_baroAltMSL_SET((int32_t) -1954073645, PH.base.pack) ;
        p10002_state_SET((e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED |
                          e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE), PH.base.pack) ;
        p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY, PH.base.pack) ;
        p10002_squawk_SET((uint16_t)(uint16_t)4755, PH.base.pack) ;
        p10002_gpsLon_SET((int32_t) -47792062, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(), &PH);
        p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_READ_11000(), &PH);
        p11000_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        p11000_request_id_SET((uint32_t)2422808142L, PH.base.pack) ;
        p11000_regstart_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p11000_count_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        {
            char16_t* busname = u"exKqaGdmLJordrtxUusxSeuwhecZtgyfjwbtmm";
            p11000_busname_SET_(busname, &PH) ;
        }
        p11000_address_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p11000_target_component_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p11000_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p11000_bus_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_READ_11000(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_READ_REPLY_11001(), &PH);
        p11001_count_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p11001_request_id_SET((uint32_t)1531350357L, PH.base.pack) ;
        p11001_regstart_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p11001_result_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)179, (uint8_t)22, (uint8_t)99, (uint8_t)128, (uint8_t)255, (uint8_t)223, (uint8_t)148, (uint8_t)124, (uint8_t)129, (uint8_t)64, (uint8_t)70, (uint8_t)226, (uint8_t)137, (uint8_t)165, (uint8_t)219, (uint8_t)134, (uint8_t)0, (uint8_t)67, (uint8_t)123, (uint8_t)233, (uint8_t)188, (uint8_t)89, (uint8_t)113, (uint8_t)69, (uint8_t)85, (uint8_t)177, (uint8_t)39, (uint8_t)2, (uint8_t)161, (uint8_t)107, (uint8_t)137, (uint8_t)76, (uint8_t)35, (uint8_t)241, (uint8_t)127, (uint8_t)34, (uint8_t)157, (uint8_t)205, (uint8_t)10, (uint8_t)71, (uint8_t)141, (uint8_t)242, (uint8_t)12, (uint8_t)71, (uint8_t)47, (uint8_t)184, (uint8_t)230, (uint8_t)7, (uint8_t)176, (uint8_t)182, (uint8_t)10, (uint8_t)234, (uint8_t)72, (uint8_t)78, (uint8_t)239, (uint8_t)36, (uint8_t)99, (uint8_t)216, (uint8_t)228, (uint8_t)145, (uint8_t)144, (uint8_t)237, (uint8_t)86, (uint8_t)198, (uint8_t)15, (uint8_t)29, (uint8_t)145, (uint8_t)122, (uint8_t)245, (uint8_t)65, (uint8_t)134, (uint8_t)110, (uint8_t)249, (uint8_t)42, (uint8_t)107, (uint8_t)56, (uint8_t)251, (uint8_t)180, (uint8_t)171, (uint8_t)88, (uint8_t)103, (uint8_t)201, (uint8_t)107, (uint8_t)44, (uint8_t)70, (uint8_t)151, (uint8_t)18, (uint8_t)221, (uint8_t)150, (uint8_t)163, (uint8_t)120, (uint8_t)111, (uint8_t)182, (uint8_t)252, (uint8_t)30, (uint8_t)90, (uint8_t)86, (uint8_t)175, (uint8_t)110, (uint8_t)5, (uint8_t)94, (uint8_t)112, (uint8_t)12, (uint8_t)251, (uint8_t)94, (uint8_t)109, (uint8_t)17, (uint8_t)184, (uint8_t)132, (uint8_t)65, (uint8_t)23, (uint8_t)158, (uint8_t)153, (uint8_t)13, (uint8_t)21, (uint8_t)148, (uint8_t)90, (uint8_t)24, (uint8_t)192, (uint8_t)91, (uint8_t)88, (uint8_t)98, (uint8_t)69, (uint8_t)209, (uint8_t)189, (uint8_t)166, (uint8_t)49, (uint8_t)112};
            p11001_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_11002(), &PH);
        p11002_address_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)116, (uint8_t)40, (uint8_t)173, (uint8_t)155, (uint8_t)36, (uint8_t)31, (uint8_t)197, (uint8_t)214, (uint8_t)146, (uint8_t)108, (uint8_t)199, (uint8_t)68, (uint8_t)139, (uint8_t)252, (uint8_t)239, (uint8_t)167, (uint8_t)40, (uint8_t)61, (uint8_t)158, (uint8_t)46, (uint8_t)7, (uint8_t)59, (uint8_t)162, (uint8_t)2, (uint8_t)81, (uint8_t)12, (uint8_t)119, (uint8_t)12, (uint8_t)93, (uint8_t)189, (uint8_t)149, (uint8_t)124, (uint8_t)171, (uint8_t)169, (uint8_t)71, (uint8_t)6, (uint8_t)50, (uint8_t)44, (uint8_t)108, (uint8_t)64, (uint8_t)170, (uint8_t)203, (uint8_t)165, (uint8_t)27, (uint8_t)140, (uint8_t)225, (uint8_t)90, (uint8_t)210, (uint8_t)226, (uint8_t)105, (uint8_t)19, (uint8_t)11, (uint8_t)238, (uint8_t)18, (uint8_t)91, (uint8_t)10, (uint8_t)61, (uint8_t)221, (uint8_t)31, (uint8_t)67, (uint8_t)75, (uint8_t)127, (uint8_t)140, (uint8_t)126, (uint8_t)38, (uint8_t)39, (uint8_t)237, (uint8_t)239, (uint8_t)69, (uint8_t)86, (uint8_t)131, (uint8_t)12, (uint8_t)156, (uint8_t)75, (uint8_t)146, (uint8_t)151, (uint8_t)203, (uint8_t)63, (uint8_t)231, (uint8_t)201, (uint8_t)175, (uint8_t)182, (uint8_t)75, (uint8_t)185, (uint8_t)211, (uint8_t)208, (uint8_t)134, (uint8_t)157, (uint8_t)152, (uint8_t)187, (uint8_t)108, (uint8_t)89, (uint8_t)28, (uint8_t)56, (uint8_t)116, (uint8_t)37, (uint8_t)47, (uint8_t)161, (uint8_t)165, (uint8_t)178, (uint8_t)244, (uint8_t)193, (uint8_t)13, (uint8_t)252, (uint8_t)134, (uint8_t)196, (uint8_t)159, (uint8_t)38, (uint8_t)239, (uint8_t)189, (uint8_t)20, (uint8_t)217, (uint8_t)21, (uint8_t)4, (uint8_t)203, (uint8_t)140, (uint8_t)18, (uint8_t)151, (uint8_t)223, (uint8_t)15, (uint8_t)172, (uint8_t)180, (uint8_t)227, (uint8_t)121, (uint8_t)78, (uint8_t)90, (uint8_t)71, (uint8_t)168};
            p11002_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11002_request_id_SET((uint32_t)4034523335L, PH.base.pack) ;
        p11002_count_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p11002_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p11002_target_system_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p11002_bus_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        {
            char16_t* busname = u"NajoNegbyvtyg";
            p11002_busname_SET_(busname, &PH) ;
        }
        p11002_regstart_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p11002_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_I2C, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_REPLY_11003(), &PH);
        p11003_request_id_SET((uint32_t)3842893079L, PH.base.pack) ;
        p11003_result_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADAP_TUNING_11010(), &PH);
        p11010_achieved_SET((float) -1.330407E38F, PH.base.pack) ;
        p11010_theta_SET((float) -2.5446238E38F, PH.base.pack) ;
        p11010_f_dot_SET((float)2.1747361E38F, PH.base.pack) ;
        p11010_theta_dot_SET((float) -2.0725203E38F, PH.base.pack) ;
        p11010_sigma_SET((float) -5.1889962E36F, PH.base.pack) ;
        p11010_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_YAW, PH.base.pack) ;
        p11010_desired_SET((float)3.8192728E37F, PH.base.pack) ;
        p11010_omega_dot_SET((float)2.4804912E38F, PH.base.pack) ;
        p11010_omega_SET((float) -1.5845864E38F, PH.base.pack) ;
        p11010_sigma_dot_SET((float) -2.0337444E38F, PH.base.pack) ;
        p11010_f_SET((float) -1.6611957E38F, PH.base.pack) ;
        p11010_error_SET((float) -3.2120997E38F, PH.base.pack) ;
        p11010_u_SET((float) -4.42136E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ADAP_TUNING_11010(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VISION_POSITION_DELTA_11011(), &PH);
        p11011_time_delta_usec_SET((uint64_t)4954082534835552085L, PH.base.pack) ;
        p11011_time_usec_SET((uint64_t)7184237831619573351L, PH.base.pack) ;
        {
            float position_delta[] =  {2.133346E38F, 2.9353287E38F, 3.0898836E37F};
            p11011_position_delta_SET(&position_delta, 0, PH.base.pack) ;
        }
        p11011_confidence_SET((float)2.8492916E38F, PH.base.pack) ;
        {
            float angle_delta[] =  {1.7128511E38F, 8.882023E37F, 9.645133E37F};
            p11011_angle_delta_SET(&angle_delta, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


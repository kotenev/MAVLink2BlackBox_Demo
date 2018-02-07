
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
    return  _en__f(get_bits(data, 40, 4));
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
    return  _en__k(get_bits(data, 48, 3));
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
    return  _en__k(get_bits(data, 48, 3));
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
    return  _en__x(get_bits(data, 276, 8));
}
INLINER e_MAV_MISSION_TYPE p39_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    return  _en__k(get_bits(data, 284, 3));
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
    return  _en__k(get_bits(data, 32, 3));
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
    return  _en__k(get_bits(data, 16, 3));
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
    return  _en__k(get_bits(data, 32, 3));
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
    return  _en__k(get_bits(data, 16, 3));
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
    return  _en__k(get_bits(data, 20, 3));
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
    return  _en__k(get_bits(data, 32, 3));
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
    return  _en__x(get_bits(data, 276, 8));
}
INLINER e_MAV_MISSION_TYPE p73_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    return  _en__k(get_bits(data, 284, 3));
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
    return  _en__x(get_bits(data, 260, 8));
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
    return  _en__x(get_bits(data, 248, 8));
}
INLINER e_MAV_CMD p77_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    return  _en__x(get_bits(data, 0, 8));
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
    return  _en__f(get_bits(data, 328, 4));
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
    return  _en__f(get_bits(data, 640, 4));
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
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CALIBRATING);
    assert(p0_custom_mode_GET(pack) == (uint32_t)1507917181L);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_GENERIC);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL));
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)5397);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)22957);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)29702);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -4938);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)41243);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)59800);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)58618);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)56053);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)34);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)49310);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)9113139124641270543L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)3235677747L);
};


void c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_yaw_rate_GET(pack) == (float)1.8300678E38F);
    assert(p3_vz_GET(pack) == (float) -1.3551482E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)14708040L);
    assert(p3_vx_GET(pack) == (float) -2.632605E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)59169);
    assert(p3_y_GET(pack) == (float) -4.3472445E37F);
    assert(p3_z_GET(pack) == (float) -3.2615399E38F);
    assert(p3_yaw_GET(pack) == (float) -3.6022302E37F);
    assert(p3_afz_GET(pack) == (float) -2.3980548E38F);
    assert(p3_afy_GET(pack) == (float) -2.1962967E38F);
    assert(p3_afx_GET(pack) == (float) -1.8687326E38F);
    assert(p3_vy_GET(pack) == (float) -8.177676E37F);
    assert(p3_x_GET(pack) == (float) -2.6014653E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)3880396579108295343L);
    assert(p4_seq_GET(pack) == (uint32_t)3152826195L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p5_passkey_LEN(ph) == 24);
    {
        char16_t * exemplary = u"ojgdkqjvudnqxvzusptuYpxh";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)203);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 31);
    {
        char16_t * exemplary = u"demprzujwspbngqirjbdqpgbpnasruu";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 62);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1490703464L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p20_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"bfcBvlxxgsynkbF";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -26042);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)213);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p22_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"mvwbdfz";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)31342);
    assert(p22_param_value_GET(pack) == (float) -3.2911002E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)54714);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float) -2.1598708E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"xefozgvu";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p24_time_usec_GET(pack) == (uint64_t)812081883450896751L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)32838);
    assert(p24_lat_GET(pack) == (int32_t)563736395);
    assert(p24_lon_GET(pack) == (int32_t)1409349401);
    assert(p24_v_acc_TRY(ph) == (uint32_t)2425708132L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)41018);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1491266432L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1012811980L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -184536551);
    assert(p24_h_acc_TRY(ph) == (uint32_t)2325586932L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p24_alt_GET(pack) == (int32_t) -990944789);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)45854);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)55627);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)180, (uint8_t)139, (uint8_t)77, (uint8_t)197, (uint8_t)181, (uint8_t)145, (uint8_t)55, (uint8_t)6, (uint8_t)250, (uint8_t)54, (uint8_t)164, (uint8_t)249, (uint8_t)47, (uint8_t)41, (uint8_t)65, (uint8_t)55, (uint8_t)70, (uint8_t)231, (uint8_t)151, (uint8_t)30} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)70, (uint8_t)219, (uint8_t)102, (uint8_t)10, (uint8_t)247, (uint8_t)4, (uint8_t)200, (uint8_t)215, (uint8_t)91, (uint8_t)13, (uint8_t)14, (uint8_t)57, (uint8_t)191, (uint8_t)58, (uint8_t)116, (uint8_t)73, (uint8_t)217, (uint8_t)32, (uint8_t)173, (uint8_t)211} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)66, (uint8_t)7, (uint8_t)82, (uint8_t)36, (uint8_t)9, (uint8_t)133, (uint8_t)52, (uint8_t)206, (uint8_t)217, (uint8_t)175, (uint8_t)203, (uint8_t)180, (uint8_t)121, (uint8_t)65, (uint8_t)2, (uint8_t)62, (uint8_t)215, (uint8_t)246, (uint8_t)140, (uint8_t)230} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)161);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)35, (uint8_t)28, (uint8_t)20, (uint8_t)204, (uint8_t)158, (uint8_t)95, (uint8_t)44, (uint8_t)202, (uint8_t)191, (uint8_t)81, (uint8_t)69, (uint8_t)146, (uint8_t)71, (uint8_t)128, (uint8_t)152, (uint8_t)63, (uint8_t)58, (uint8_t)252, (uint8_t)38} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)217, (uint8_t)242, (uint8_t)28, (uint8_t)9, (uint8_t)226, (uint8_t)36, (uint8_t)232, (uint8_t)228, (uint8_t)5, (uint8_t)125, (uint8_t)53, (uint8_t)164, (uint8_t)11, (uint8_t)165, (uint8_t)239, (uint8_t)38, (uint8_t)202, (uint8_t)48, (uint8_t)55, (uint8_t)114} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2670636110L);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)16340);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)3532);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -27678);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)17871);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -11962);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)23578);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)14972);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -31972);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)7573);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)22472);
    assert(p27_time_usec_GET(pack) == (uint64_t)5336946355204920090L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)27653);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -28460);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)12291);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -4189);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)4814);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)3839);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -11358);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)17432);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)6116483033102451736L);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -11624);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -11700);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -4931);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)29670);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float)1.1376476E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)10156);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)4169372646L);
    assert(p29_press_diff_GET(pack) == (float)9.754039E35F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitch_GET(pack) == (float) -2.699166E38F);
    assert(p30_yaw_GET(pack) == (float) -2.2428131E37F);
    assert(p30_rollspeed_GET(pack) == (float)1.6218839E38F);
    assert(p30_pitchspeed_GET(pack) == (float)1.366672E38F);
    assert(p30_roll_GET(pack) == (float)1.3467523E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)335549377L);
    assert(p30_yawspeed_GET(pack) == (float)3.4802348E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_yawspeed_GET(pack) == (float) -1.5157329E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1252572554L);
    assert(p31_rollspeed_GET(pack) == (float)2.8015342E38F);
    assert(p31_q2_GET(pack) == (float)1.977852E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -1.2037808E38F);
    assert(p31_q1_GET(pack) == (float)1.8940521E38F);
    assert(p31_q3_GET(pack) == (float)1.8936846E38F);
    assert(p31_q4_GET(pack) == (float) -2.902244E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float)3.2366209E38F);
    assert(p32_vy_GET(pack) == (float) -4.103674E36F);
    assert(p32_x_GET(pack) == (float)2.0022598E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2119784049L);
    assert(p32_vz_GET(pack) == (float) -2.0809572E38F);
    assert(p32_y_GET(pack) == (float)3.3296914E38F);
    assert(p32_vx_GET(pack) == (float) -7.539541E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)23942);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -24038);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)3104226334L);
    assert(p33_lat_GET(pack) == (int32_t) -490945291);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)37133);
    assert(p33_relative_alt_GET(pack) == (int32_t)2125111211);
    assert(p33_alt_GET(pack) == (int32_t) -500480532);
    assert(p33_lon_GET(pack) == (int32_t) -1433095348);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)9541);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2781093093L);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -20634);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)13254);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -22260);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -13506);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -15591);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -26058);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -11687);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)12547);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)30329);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)2792);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)58728);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)7920);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3565101656L);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)65376);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)47550);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)65410);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)62225);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)82);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)55580);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)5260);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)55555);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)4780);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)59001);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)7688);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)60202);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)37572);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)32500);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)6058);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)52962);
    assert(p36_time_usec_GET(pack) == (uint32_t)3007888727L);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)50194);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)15114);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)36590);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)57075);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -19629);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)29644);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)540);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)10850);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)150);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_START_RX_PAIR);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)19781);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p39_x_GET(pack) == (float)2.9298906E38F);
    assert(p39_y_GET(pack) == (float)3.3412876E38F);
    assert(p39_param1_GET(pack) == (float)2.5913364E37F);
    assert(p39_param2_GET(pack) == (float)4.771834E37F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p39_z_GET(pack) == (float)1.4202114E38F);
    assert(p39_param3_GET(pack) == (float) -2.7440234E38F);
    assert(p39_param4_GET(pack) == (float) -1.7461544E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)17087);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)50857);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)193);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)45527);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)44085);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)106);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)51891);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -915142343);
    assert(p48_longitude_GET(pack) == (int32_t)1128570699);
    assert(p48_altitude_GET(pack) == (int32_t) -470351972);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p48_time_usec_TRY(ph) == (uint64_t)2829818730738694054L);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)305731949494471015L);
    assert(p49_altitude_GET(pack) == (int32_t) -756052815);
    assert(p49_longitude_GET(pack) == (int32_t)878439586);
    assert(p49_latitude_GET(pack) == (int32_t)2119238268);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p50_param_value0_GET(pack) == (float)3.1534168E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p50_param_value_max_GET(pack) == (float) -2.3263042E38F);
    assert(p50_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"ebmfwJixgzvQi";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_min_GET(pack) == (float)3.1828877E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -19495);
    assert(p50_scale_GET(pack) == (float)2.497076E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)37188);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1x_GET(pack) == (float)5.979763E37F);
    assert(p54_p1y_GET(pack) == (float)1.762506E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p54_p2x_GET(pack) == (float)1.4746301E38F);
    assert(p54_p2y_GET(pack) == (float) -3.2924186E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p54_p1z_GET(pack) == (float)1.618148E38F);
    assert(p54_p2z_GET(pack) == (float) -2.6795347E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)73);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2x_GET(pack) == (float) -2.7728822E38F);
    assert(p55_p1z_GET(pack) == (float) -1.5931289E38F);
    assert(p55_p2z_GET(pack) == (float)2.5445569E38F);
    assert(p55_p1x_GET(pack) == (float) -2.451315E38F);
    assert(p55_p2y_GET(pack) == (float) -7.585071E37F);
    assert(p55_p1y_GET(pack) == (float)2.7264589E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_pitchspeed_GET(pack) == (float)1.3826753E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)3434135610052113093L);
    {
        float exemplary[] =  {-2.3120016E38F, -1.3924764E38F, 7.203817E37F, -1.7756672E38F, 1.0527848E38F, 1.805208E38F, 2.7960529E38F, -1.5685595E38F, -2.3839447E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.8531728E36F, 1.6117041E38F, -2.6416076E38F, -6.8527904E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float) -3.3957637E37F);
    assert(p61_rollspeed_GET(pack) == (float) -2.3735532E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -9732);
    assert(p62_nav_roll_GET(pack) == (float)2.7056244E38F);
    assert(p62_aspd_error_GET(pack) == (float)1.9368425E38F);
    assert(p62_nav_pitch_GET(pack) == (float) -3.1653018E38F);
    assert(p62_xtrack_error_GET(pack) == (float)2.7191612E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)22947);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)2074);
    assert(p62_alt_error_GET(pack) == (float)3.2057809E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t) -868806138);
    assert(p63_lat_GET(pack) == (int32_t) -403303500);
    assert(p63_vx_GET(pack) == (float) -1.6030246E38F);
    assert(p63_relative_alt_GET(pack) == (int32_t) -973687288);
    {
        float exemplary[] =  {-3.239034E38F, -1.2033711E38F, 2.5007235E38F, -2.5752476E38F, -2.3235232E38F, -1.6642371E38F, 1.4975653E38F, -3.342737E38F, -2.056897E38F, 6.7402965E37F, 2.5237786E38F, 5.44866E37F, 3.0737951E38F, -2.3864056E38F, -1.2236092E38F, -1.6901441E38F, 3.2037553E38F, 1.4200998E37F, -4.9988214E37F, 1.2144409E38F, 2.2459767E38F, -1.0323024E38F, 2.7316197E38F, -4.066143E37F, -3.177473E38F, 1.0734106E38F, -7.1379443E37F, 2.2348323E38F, -1.2298958E38F, -1.4754168E38F, 1.8674923E38F, -1.8441298E38F, -4.5068604E37F, -1.1949669E38F, 1.993312E38F, -1.5676032E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vz_GET(pack) == (float) -1.652108E38F);
    assert(p63_vy_GET(pack) == (float) -2.8229015E38F);
    assert(p63_alt_GET(pack) == (int32_t)248815890);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p63_time_usec_GET(pack) == (uint64_t)4826881826863284526L);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vz_GET(pack) == (float)1.3942385E38F);
    assert(p64_vy_GET(pack) == (float)4.500147E37F);
    assert(p64_z_GET(pack) == (float)2.6468491E37F);
    assert(p64_ay_GET(pack) == (float)2.7114265E37F);
    assert(p64_y_GET(pack) == (float) -2.5287611E38F);
    assert(p64_az_GET(pack) == (float)1.2321267E38F);
    assert(p64_x_GET(pack) == (float)2.727631E38F);
    assert(p64_vx_GET(pack) == (float) -1.1269235E38F);
    assert(p64_ax_GET(pack) == (float)5.4818137E36F);
    assert(p64_time_usec_GET(pack) == (uint64_t)6652723940646630943L);
    {
        float exemplary[] =  {-2.4572496E38F, -1.3927415E37F, -2.0380216E37F, -3.4929633E37F, 3.196413E37F, -2.5114801E38F, 2.6954083E38F, 6.1510367E37F, -2.7663475E37F, -6.205452E37F, -1.7751019E38F, 2.2543432E38F, 2.784362E38F, -5.8358045E37F, 3.3321543E38F, -2.2846433E38F, -2.0583723E38F, 1.62246E38F, 5.149354E37F, 2.8352184E38F, -5.2137775E37F, 5.280439E37F, 1.6853268E38F, 3.2967728E38F, 8.469143E37F, -1.046969E38F, 8.644893E37F, 2.6563065E38F, -2.6433764E38F, 2.352082E38F, -2.0665003E38F, -3.346764E38F, -1.2072818E38F, -1.877937E38F, -4.596367E37F, -1.8761286E38F, 3.190952E38F, 7.2946107E37F, 1.3592209E37F, 2.5025461E38F, -6.6362594E37F, -2.3309074E38F, 1.301947E38F, -1.288473E38F, -1.7244368E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)65416);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2188519992L);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)36120);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)6976);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)31094);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)64427);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)28249);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)36356);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)47323);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)25898);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)33434);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)525);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)23145);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)14791);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)49633);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)13080);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)59948);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)19564);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)61066);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)184);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)55704);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)151);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)33592);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)173);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_r_GET(pack) == (int16_t)(int16_t)23266);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)28536);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)3651);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)21184);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)8489);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)24823);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)18453);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)48000);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)8710);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)18519);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)16043);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)31568);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)58749);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p73_y_GET(pack) == (int32_t)140690130);
    assert(p73_param3_GET(pack) == (float)1.260575E38F);
    assert(p73_param4_GET(pack) == (float) -2.4459255E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_param2_GET(pack) == (float)1.0296012E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)18223);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION);
    assert(p73_param1_GET(pack) == (float) -1.6222878E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p73_x_GET(pack) == (int32_t) -709876902);
    assert(p73_z_GET(pack) == (float) -3.23091E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_alt_GET(pack) == (float)1.473918E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -22389);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)44875);
    assert(p74_groundspeed_GET(pack) == (float) -3.1275636E38F);
    assert(p74_airspeed_GET(pack) == (float)9.358248E36F);
    assert(p74_climb_GET(pack) == (float) -2.72466E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p75_param2_GET(pack) == (float)1.859429E38F);
    assert(p75_param1_GET(pack) == (float) -1.516049E38F);
    assert(p75_param4_GET(pack) == (float)1.3499299E38F);
    assert(p75_x_GET(pack) == (int32_t)487020625);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p75_z_GET(pack) == (float) -1.2734928E37F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_LOGGING_STOP);
    assert(p75_param3_GET(pack) == (float)4.1519188E37F);
    assert(p75_y_GET(pack) == (int32_t) -1252689007);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p76_param2_GET(pack) == (float)2.3891985E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION);
    assert(p76_param5_GET(pack) == (float) -2.101496E38F);
    assert(p76_param7_GET(pack) == (float)5.629998E37F);
    assert(p76_param6_GET(pack) == (float) -2.3954525E37F);
    assert(p76_param3_GET(pack) == (float)2.2488908E37F);
    assert(p76_param4_GET(pack) == (float)3.0607724E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p76_param1_GET(pack) == (float)2.6181997E37F);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)160);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)138);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1071525431);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)0);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float)7.806374E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2375137648L);
    assert(p81_roll_GET(pack) == (float)1.0686201E37F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p81_pitch_GET(pack) == (float)2.3611834E38F);
    assert(p81_yaw_GET(pack) == (float) -1.0723589E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.110495E38F, 1.8283688E38F, -1.5858954E38F, -2.8520218E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float)1.5138876E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1865019388L);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p82_thrust_GET(pack) == (float) -1.6834304E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float)9.355765E37F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p82_body_roll_rate_GET(pack) == (float)2.2191887E38F);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)1388107930L);
    assert(p83_body_pitch_rate_GET(pack) == (float)7.0352747E37F);
    {
        float exemplary[] =  {-1.6234445E38F, 2.5620551E38F, 9.460931E37F, -2.8050946E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_yaw_rate_GET(pack) == (float)6.399244E37F);
    assert(p83_body_roll_rate_GET(pack) == (float) -7.1474704E37F);
    assert(p83_thrust_GET(pack) == (float)2.52084E37F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_vz_GET(pack) == (float)2.3886864E38F);
    assert(p84_x_GET(pack) == (float) -1.02488E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p84_yaw_GET(pack) == (float)1.7210981E38F);
    assert(p84_vy_GET(pack) == (float)1.8584514E38F);
    assert(p84_y_GET(pack) == (float) -4.7472935E36F);
    assert(p84_afy_GET(pack) == (float)3.2945758E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)37263);
    assert(p84_yaw_rate_GET(pack) == (float) -2.7698837E37F);
    assert(p84_afz_GET(pack) == (float)2.7866063E38F);
    assert(p84_afx_GET(pack) == (float)1.9510337E38F);
    assert(p84_z_GET(pack) == (float)1.1099538E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)4033295809L);
    assert(p84_vx_GET(pack) == (float)3.179521E38F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lat_int_GET(pack) == (int32_t) -1567362979);
    assert(p86_lon_int_GET(pack) == (int32_t)1559058010);
    assert(p86_vy_GET(pack) == (float) -2.2250706E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p86_afy_GET(pack) == (float)2.7821444E38F);
    assert(p86_alt_GET(pack) == (float) -2.738635E38F);
    assert(p86_afx_GET(pack) == (float)1.718975E38F);
    assert(p86_yaw_GET(pack) == (float)1.5041396E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2162802560L);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)60130);
    assert(p86_afz_GET(pack) == (float) -6.416846E37F);
    assert(p86_vx_GET(pack) == (float)7.973448E37F);
    assert(p86_yaw_rate_GET(pack) == (float)2.389407E38F);
    assert(p86_vz_GET(pack) == (float)1.399556E38F);
};


void c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_rate_GET(pack) == (float) -1.0333026E36F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p87_alt_GET(pack) == (float) -1.6659315E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)42231);
    assert(p87_lat_int_GET(pack) == (int32_t) -1824438050);
    assert(p87_vz_GET(pack) == (float)1.4028186E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)1569448406);
    assert(p87_vx_GET(pack) == (float) -1.8053442E38F);
    assert(p87_yaw_GET(pack) == (float) -2.150952E38F);
    assert(p87_afy_GET(pack) == (float)3.2144699E38F);
    assert(p87_afx_GET(pack) == (float)7.8328714E37F);
    assert(p87_afz_GET(pack) == (float)9.170917E37F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)4235700299L);
    assert(p87_vy_GET(pack) == (float)3.3450225E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float) -2.0320524E38F);
    assert(p89_z_GET(pack) == (float)3.2487988E38F);
    assert(p89_y_GET(pack) == (float)1.5250185E38F);
    assert(p89_x_GET(pack) == (float)6.6979377E37F);
    assert(p89_roll_GET(pack) == (float) -8.545645E37F);
    assert(p89_pitch_GET(pack) == (float)2.8056473E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)234747090L);
};


void c_TEST_Channel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_time_usec_GET(pack) == (uint64_t)1284337630584615164L);
    assert(p90_rollspeed_GET(pack) == (float)3.3793549E38F);
    assert(p90_pitchspeed_GET(pack) == (float) -1.4483352E38F);
    assert(p90_lon_GET(pack) == (int32_t)289033223);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)247);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)3548);
    assert(p90_yawspeed_GET(pack) == (float)6.167556E37F);
    assert(p90_lat_GET(pack) == (int32_t)324814523);
    assert(p90_yaw_GET(pack) == (float)1.5827087E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -29280);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)29939);
    assert(p90_alt_GET(pack) == (int32_t)2102524764);
    assert(p90_roll_GET(pack) == (float) -1.978632E38F);
    assert(p90_pitch_GET(pack) == (float)1.7210468E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)30648);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -6655);
};


void c_TEST_Channel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_time_usec_GET(pack) == (uint64_t)5757903766957816839L);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p91_aux1_GET(pack) == (float)3.2588591E38F);
    assert(p91_aux3_GET(pack) == (float)2.6312098E38F);
    assert(p91_aux4_GET(pack) == (float) -2.7828103E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.6928678E37F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.5672295E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)1.2904425E38F);
    assert(p91_throttle_GET(pack) == (float) -6.7706385E37F);
    assert(p91_aux2_GET(pack) == (float) -1.4915871E38F);
};


void c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)48363);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)1317);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)35116);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)48977);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)34362);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)46431);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)51574);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)2185);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)7172);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)51820);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)25791);
    assert(p92_time_usec_GET(pack) == (uint64_t)6059012416357554919L);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)47400);
};


void c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)4580394412636514916L);
    assert(p93_time_usec_GET(pack) == (uint64_t)4861710728761731785L);
    {
        float exemplary[] =  {3.2585622E38F, -3.363136E38F, 1.8356508E38F, -2.123291E38F, 2.8708684E38F, -9.103146E37F, -1.7228568E38F, -6.4989597E37F, -1.6172763E38F, 8.618825E37F, -5.750507E37F, 2.8468954E38F, 2.7720515E38F, -2.379066E38F, 4.2452158E37F, 3.0816034E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
};


void c_TEST_Channel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -9888);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.9185402E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)21900);
    assert(p100_flow_rate_y_TRY(ph) == (float)1.5015584E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p100_time_usec_GET(pack) == (uint64_t)4674736177544946472L);
    assert(p100_flow_comp_m_y_GET(pack) == (float)6.2161443E37F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)2.1452295E38F);
    assert(p100_ground_distance_GET(pack) == (float)2.9896874E38F);
};


void c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_usec_GET(pack) == (uint64_t)6086547414861448083L);
    assert(p101_y_GET(pack) == (float) -8.618382E37F);
    assert(p101_yaw_GET(pack) == (float)2.7658473E38F);
    assert(p101_pitch_GET(pack) == (float) -3.05495E38F);
    assert(p101_roll_GET(pack) == (float)2.5556797E37F);
    assert(p101_x_GET(pack) == (float) -2.0929502E38F);
    assert(p101_z_GET(pack) == (float) -1.297407E38F);
};


void c_TEST_Channel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_x_GET(pack) == (float)9.608518E37F);
    assert(p102_y_GET(pack) == (float) -6.5425115E35F);
    assert(p102_z_GET(pack) == (float)1.0085688E38F);
    assert(p102_yaw_GET(pack) == (float) -1.5608023E38F);
    assert(p102_roll_GET(pack) == (float)2.664251E37F);
    assert(p102_pitch_GET(pack) == (float) -1.5696092E38F);
    assert(p102_usec_GET(pack) == (uint64_t)378144909300573238L);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float) -3.80839E37F);
    assert(p103_usec_GET(pack) == (uint64_t)3483613725074284266L);
    assert(p103_z_GET(pack) == (float)2.1608978E37F);
    assert(p103_y_GET(pack) == (float)2.2293767E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float)2.7888045E38F);
    assert(p104_y_GET(pack) == (float)1.089077E38F);
    assert(p104_yaw_GET(pack) == (float)2.6499134E38F);
    assert(p104_z_GET(pack) == (float)1.1727845E38F);
    assert(p104_usec_GET(pack) == (uint64_t)5921116010459568060L);
    assert(p104_roll_GET(pack) == (float)1.0015415E38F);
    assert(p104_pitch_GET(pack) == (float)9.739904E37F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_xgyro_GET(pack) == (float)3.0429239E38F);
    assert(p105_temperature_GET(pack) == (float)1.3940721E38F);
    assert(p105_xmag_GET(pack) == (float)5.2948493E37F);
    assert(p105_zgyro_GET(pack) == (float)1.9640729E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)55504);
    assert(p105_zacc_GET(pack) == (float)3.3638346E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)817051026694375629L);
    assert(p105_zmag_GET(pack) == (float)1.3177413E38F);
    assert(p105_xacc_GET(pack) == (float) -1.7256002E38F);
    assert(p105_ymag_GET(pack) == (float)1.6076913E38F);
    assert(p105_ygyro_GET(pack) == (float) -1.5347194E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.1878992E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -4.0822598E37F);
    assert(p105_yacc_GET(pack) == (float)2.971699E38F);
    assert(p105_pressure_alt_GET(pack) == (float)3.327981E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_zgyro_GET(pack) == (float) -4.6513843E37F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.2569668E38F);
    assert(p106_distance_GET(pack) == (float) -9.799405E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -2440);
    assert(p106_time_usec_GET(pack) == (uint64_t)6622896616186263140L);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3484616038L);
    assert(p106_integrated_x_GET(pack) == (float)3.0941276E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3730843564L);
    assert(p106_integrated_y_GET(pack) == (float) -8.687961E37F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -4.775582E37F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_xgyro_GET(pack) == (float) -2.2360308E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.878419E38F);
    assert(p107_yacc_GET(pack) == (float) -3.0329307E38F);
    assert(p107_zmag_GET(pack) == (float)3.2615453E38F);
    assert(p107_zacc_GET(pack) == (float)3.2412842E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.4816222E38F);
    assert(p107_xacc_GET(pack) == (float) -6.774448E37F);
    assert(p107_xmag_GET(pack) == (float)1.1837633E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)241717256L);
    assert(p107_temperature_GET(pack) == (float)3.3519027E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)8856169662564523389L);
    assert(p107_ygyro_GET(pack) == (float)9.664411E37F);
    assert(p107_abs_pressure_GET(pack) == (float)1.6570601E38F);
    assert(p107_ymag_GET(pack) == (float)2.5759528E38F);
    assert(p107_zgyro_GET(pack) == (float) -2.3116116E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_xacc_GET(pack) == (float)2.0649278E38F);
    assert(p108_zacc_GET(pack) == (float)1.2866834E38F);
    assert(p108_lon_GET(pack) == (float)1.5591941E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -9.390899E37F);
    assert(p108_q2_GET(pack) == (float) -2.2133751E38F);
    assert(p108_roll_GET(pack) == (float)3.1632888E38F);
    assert(p108_xgyro_GET(pack) == (float) -1.0651798E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -2.0625973E38F);
    assert(p108_alt_GET(pack) == (float) -1.6529827E38F);
    assert(p108_q1_GET(pack) == (float)2.5988707E38F);
    assert(p108_lat_GET(pack) == (float)8.547024E37F);
    assert(p108_yacc_GET(pack) == (float)1.3250589E38F);
    assert(p108_q3_GET(pack) == (float) -3.9577668E36F);
    assert(p108_ygyro_GET(pack) == (float) -2.453613E38F);
    assert(p108_zgyro_GET(pack) == (float)2.4962868E38F);
    assert(p108_pitch_GET(pack) == (float) -1.7488926E37F);
    assert(p108_vn_GET(pack) == (float)2.974551E38F);
    assert(p108_yaw_GET(pack) == (float)2.7203127E38F);
    assert(p108_vd_GET(pack) == (float) -1.4110689E38F);
    assert(p108_ve_GET(pack) == (float)8.9798144E36F);
    assert(p108_q4_GET(pack) == (float) -2.0973673E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)19146);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)63072);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)99);
    {
        uint8_t exemplary[] =  {(uint8_t)141, (uint8_t)243, (uint8_t)166, (uint8_t)144, (uint8_t)138, (uint8_t)131, (uint8_t)206, (uint8_t)38, (uint8_t)39, (uint8_t)59, (uint8_t)156, (uint8_t)98, (uint8_t)41, (uint8_t)212, (uint8_t)21, (uint8_t)182, (uint8_t)134, (uint8_t)63, (uint8_t)222, (uint8_t)104, (uint8_t)66, (uint8_t)140, (uint8_t)88, (uint8_t)248, (uint8_t)150, (uint8_t)114, (uint8_t)115, (uint8_t)78, (uint8_t)24, (uint8_t)210, (uint8_t)208, (uint8_t)0, (uint8_t)104, (uint8_t)67, (uint8_t)103, (uint8_t)134, (uint8_t)6, (uint8_t)95, (uint8_t)9, (uint8_t)107, (uint8_t)249, (uint8_t)189, (uint8_t)56, (uint8_t)102, (uint8_t)222, (uint8_t)218, (uint8_t)220, (uint8_t)246, (uint8_t)103, (uint8_t)20, (uint8_t)160, (uint8_t)26, (uint8_t)136, (uint8_t)187, (uint8_t)132, (uint8_t)196, (uint8_t)224, (uint8_t)26, (uint8_t)58, (uint8_t)243, (uint8_t)12, (uint8_t)184, (uint8_t)183, (uint8_t)48, (uint8_t)151, (uint8_t)19, (uint8_t)49, (uint8_t)33, (uint8_t)25, (uint8_t)189, (uint8_t)117, (uint8_t)181, (uint8_t)131, (uint8_t)69, (uint8_t)225, (uint8_t)63, (uint8_t)63, (uint8_t)254, (uint8_t)4, (uint8_t)121, (uint8_t)224, (uint8_t)227, (uint8_t)233, (uint8_t)74, (uint8_t)115, (uint8_t)99, (uint8_t)108, (uint8_t)30, (uint8_t)22, (uint8_t)196, (uint8_t)201, (uint8_t)21, (uint8_t)38, (uint8_t)82, (uint8_t)206, (uint8_t)227, (uint8_t)110, (uint8_t)110, (uint8_t)166, (uint8_t)63, (uint8_t)94, (uint8_t)229, (uint8_t)127, (uint8_t)80, (uint8_t)106, (uint8_t)27, (uint8_t)7, (uint8_t)59, (uint8_t)224, (uint8_t)56, (uint8_t)235, (uint8_t)137, (uint8_t)143, (uint8_t)82, (uint8_t)132, (uint8_t)226, (uint8_t)192, (uint8_t)23, (uint8_t)145, (uint8_t)27, (uint8_t)13, (uint8_t)125, (uint8_t)49, (uint8_t)78, (uint8_t)128, (uint8_t)30, (uint8_t)240, (uint8_t)100, (uint8_t)34, (uint8_t)82, (uint8_t)71, (uint8_t)3, (uint8_t)49, (uint8_t)252, (uint8_t)14, (uint8_t)60, (uint8_t)111, (uint8_t)29, (uint8_t)86, (uint8_t)66, (uint8_t)111, (uint8_t)98, (uint8_t)187, (uint8_t)251, (uint8_t)140, (uint8_t)55, (uint8_t)153, (uint8_t)33, (uint8_t)144, (uint8_t)113, (uint8_t)186, (uint8_t)212, (uint8_t)228, (uint8_t)176, (uint8_t)180, (uint8_t)132, (uint8_t)172, (uint8_t)182, (uint8_t)234, (uint8_t)140, (uint8_t)25, (uint8_t)173, (uint8_t)190, (uint8_t)147, (uint8_t)156, (uint8_t)31, (uint8_t)103, (uint8_t)92, (uint8_t)87, (uint8_t)150, (uint8_t)174, (uint8_t)241, (uint8_t)53, (uint8_t)196, (uint8_t)173, (uint8_t)119, (uint8_t)42, (uint8_t)216, (uint8_t)47, (uint8_t)111, (uint8_t)101, (uint8_t)64, (uint8_t)147, (uint8_t)235, (uint8_t)23, (uint8_t)83, (uint8_t)113, (uint8_t)174, (uint8_t)204, (uint8_t)100, (uint8_t)113, (uint8_t)105, (uint8_t)53, (uint8_t)172, (uint8_t)112, (uint8_t)203, (uint8_t)235, (uint8_t)14, (uint8_t)131, (uint8_t)186, (uint8_t)236, (uint8_t)42, (uint8_t)10, (uint8_t)128, (uint8_t)37, (uint8_t)171, (uint8_t)113, (uint8_t)213, (uint8_t)135, (uint8_t)25, (uint8_t)107, (uint8_t)129, (uint8_t)154, (uint8_t)223, (uint8_t)162, (uint8_t)30, (uint8_t)208, (uint8_t)181, (uint8_t)63, (uint8_t)0, (uint8_t)32, (uint8_t)12, (uint8_t)233, (uint8_t)218, (uint8_t)251, (uint8_t)236, (uint8_t)176, (uint8_t)6, (uint8_t)216, (uint8_t)1, (uint8_t)15, (uint8_t)170, (uint8_t)103, (uint8_t)147, (uint8_t)174, (uint8_t)119, (uint8_t)26, (uint8_t)54, (uint8_t)217, (uint8_t)143, (uint8_t)213, (uint8_t)99, (uint8_t)217, (uint8_t)77, (uint8_t)210, (uint8_t)24, (uint8_t)229, (uint8_t)161, (uint8_t)137, (uint8_t)16, (uint8_t)56} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)92);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -9018038205928207881L);
    assert(p111_ts1_GET(pack) == (int64_t) -4531486100787339264L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)3710759477213164168L);
    assert(p112_seq_GET(pack) == (uint32_t)704945899L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p113_lon_GET(pack) == (int32_t) -1658080600);
    assert(p113_alt_GET(pack) == (int32_t)2042698332);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)42145);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)2246);
    assert(p113_time_usec_GET(pack) == (uint64_t)114870830184988021L);
    assert(p113_lat_GET(pack) == (int32_t)956735742);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)16881);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -13671);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)27102);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -25816);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)53593);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_usec_GET(pack) == (uint64_t)4269665010210588135L);
    assert(p114_integrated_ygyro_GET(pack) == (float) -3.0068013E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)861976574L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -3.0582078E37F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -29949);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)370485846L);
    assert(p114_integrated_y_GET(pack) == (float)2.6493324E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.800972E37F);
    assert(p114_distance_GET(pack) == (float)1.0443086E37F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p114_integrated_x_GET(pack) == (float)3.1109383E37F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_lon_GET(pack) == (int32_t)1412966736);
    assert(p115_rollspeed_GET(pack) == (float)2.7413259E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)44256);
    {
        float exemplary[] =  {3.3041836E38F, 3.368426E38F, -1.7960792E38F, 3.0419215E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_lat_GET(pack) == (int32_t) -1908724081);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -8988);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -23725);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -20426);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -11477);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -25426);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)30447);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)62680);
    assert(p115_time_usec_GET(pack) == (uint64_t)4156186339292580402L);
    assert(p115_alt_GET(pack) == (int32_t)832105460);
    assert(p115_pitchspeed_GET(pack) == (float) -2.7374696E38F);
    assert(p115_yawspeed_GET(pack) == (float)2.2026764E38F);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)8886);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -27049);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)14337);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -22630);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -21932);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)4070955076L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -18253);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)18208);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)19524);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)17260);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)37076);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)29198);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)22081);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)13177);
    assert(p118_size_GET(pack) == (uint32_t)3369408209L);
    assert(p118_time_utc_GET(pack) == (uint32_t)4205203566L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)20915);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_count_GET(pack) == (uint32_t)553840313L);
    assert(p119_ofs_GET(pack) == (uint32_t)607567072L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)61585);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)3, (uint8_t)32, (uint8_t)176, (uint8_t)70, (uint8_t)136, (uint8_t)137, (uint8_t)55, (uint8_t)155, (uint8_t)96, (uint8_t)79, (uint8_t)219, (uint8_t)200, (uint8_t)78, (uint8_t)253, (uint8_t)37, (uint8_t)161, (uint8_t)133, (uint8_t)221, (uint8_t)32, (uint8_t)100, (uint8_t)196, (uint8_t)126, (uint8_t)51, (uint8_t)81, (uint8_t)76, (uint8_t)230, (uint8_t)160, (uint8_t)29, (uint8_t)253, (uint8_t)92, (uint8_t)19, (uint8_t)248, (uint8_t)144, (uint8_t)113, (uint8_t)160, (uint8_t)216, (uint8_t)202, (uint8_t)195, (uint8_t)74, (uint8_t)240, (uint8_t)168, (uint8_t)17, (uint8_t)72, (uint8_t)156, (uint8_t)166, (uint8_t)106, (uint8_t)5, (uint8_t)48, (uint8_t)180, (uint8_t)145, (uint8_t)7, (uint8_t)127, (uint8_t)176, (uint8_t)140, (uint8_t)243, (uint8_t)179, (uint8_t)184, (uint8_t)107, (uint8_t)16, (uint8_t)149, (uint8_t)166, (uint8_t)86, (uint8_t)233, (uint8_t)5, (uint8_t)87, (uint8_t)177, (uint8_t)155, (uint8_t)219, (uint8_t)46, (uint8_t)108, (uint8_t)35, (uint8_t)40, (uint8_t)250, (uint8_t)42, (uint8_t)68, (uint8_t)27, (uint8_t)198, (uint8_t)60, (uint8_t)187, (uint8_t)229, (uint8_t)71, (uint8_t)164, (uint8_t)165, (uint8_t)235, (uint8_t)88, (uint8_t)198, (uint8_t)15, (uint8_t)87, (uint8_t)76} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p120_ofs_GET(pack) == (uint32_t)148672024L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)4928);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)228);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)111);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)132, (uint8_t)49, (uint8_t)194, (uint8_t)187, (uint8_t)216, (uint8_t)43, (uint8_t)66, (uint8_t)229, (uint8_t)98, (uint8_t)93, (uint8_t)9, (uint8_t)105, (uint8_t)137, (uint8_t)55, (uint8_t)4, (uint8_t)184, (uint8_t)147, (uint8_t)211, (uint8_t)22, (uint8_t)42, (uint8_t)185, (uint8_t)153, (uint8_t)208, (uint8_t)87, (uint8_t)241, (uint8_t)63, (uint8_t)123, (uint8_t)213, (uint8_t)186, (uint8_t)238, (uint8_t)85, (uint8_t)216, (uint8_t)167, (uint8_t)207, (uint8_t)145, (uint8_t)104, (uint8_t)201, (uint8_t)23, (uint8_t)5, (uint8_t)111, (uint8_t)175, (uint8_t)209, (uint8_t)160, (uint8_t)195, (uint8_t)136, (uint8_t)192, (uint8_t)245, (uint8_t)72, (uint8_t)13, (uint8_t)15, (uint8_t)134, (uint8_t)127, (uint8_t)116, (uint8_t)103, (uint8_t)59, (uint8_t)167, (uint8_t)193, (uint8_t)41, (uint8_t)116, (uint8_t)3, (uint8_t)93, (uint8_t)244, (uint8_t)200, (uint8_t)28, (uint8_t)232, (uint8_t)82, (uint8_t)98, (uint8_t)106, (uint8_t)100, (uint8_t)237, (uint8_t)194, (uint8_t)26, (uint8_t)3, (uint8_t)82, (uint8_t)209, (uint8_t)48, (uint8_t)158, (uint8_t)96, (uint8_t)177, (uint8_t)108, (uint8_t)148, (uint8_t)60, (uint8_t)146, (uint8_t)250, (uint8_t)25, (uint8_t)92, (uint8_t)124, (uint8_t)164, (uint8_t)28, (uint8_t)251, (uint8_t)22, (uint8_t)14, (uint8_t)28, (uint8_t)147, (uint8_t)116, (uint8_t)102, (uint8_t)122, (uint8_t)243, (uint8_t)122, (uint8_t)34, (uint8_t)164, (uint8_t)142, (uint8_t)13, (uint8_t)217, (uint8_t)104, (uint8_t)194, (uint8_t)140, (uint8_t)251, (uint8_t)121, (uint8_t)73} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lat_GET(pack) == (int32_t) -1220777752);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)25641);
    assert(p124_time_usec_GET(pack) == (uint64_t)1131950209486071480L);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)61587);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)45678);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)26970);
    assert(p124_alt_GET(pack) == (int32_t)1646134069);
    assert(p124_dgps_age_GET(pack) == (uint32_t)3818283698L);
    assert(p124_lon_GET(pack) == (int32_t) -941543304);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)43);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)52439);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)16079);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)1977294454L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)29413);
    {
        uint8_t exemplary[] =  {(uint8_t)21, (uint8_t)230, (uint8_t)191, (uint8_t)199, (uint8_t)245, (uint8_t)191, (uint8_t)35, (uint8_t)155, (uint8_t)147, (uint8_t)198, (uint8_t)124, (uint8_t)120, (uint8_t)20, (uint8_t)109, (uint8_t)235, (uint8_t)228, (uint8_t)71, (uint8_t)98, (uint8_t)80, (uint8_t)250, (uint8_t)139, (uint8_t)168, (uint8_t)173, (uint8_t)213, (uint8_t)119, (uint8_t)215, (uint8_t)54, (uint8_t)196, (uint8_t)250, (uint8_t)74, (uint8_t)53, (uint8_t)23, (uint8_t)138, (uint8_t)220, (uint8_t)81, (uint8_t)109, (uint8_t)15, (uint8_t)146, (uint8_t)59, (uint8_t)223, (uint8_t)150, (uint8_t)93, (uint8_t)95, (uint8_t)200, (uint8_t)129, (uint8_t)49, (uint8_t)189, (uint8_t)223, (uint8_t)216, (uint8_t)70, (uint8_t)107, (uint8_t)51, (uint8_t)89, (uint8_t)98, (uint8_t)24, (uint8_t)233, (uint8_t)249, (uint8_t)149, (uint8_t)188, (uint8_t)49, (uint8_t)149, (uint8_t)255, (uint8_t)83, (uint8_t)125, (uint8_t)235, (uint8_t)129, (uint8_t)207, (uint8_t)215, (uint8_t)134, (uint8_t)77} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING));
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1936516311);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)31214);
    assert(p127_tow_GET(pack) == (uint32_t)775965332L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p127_accuracy_GET(pack) == (uint32_t)1345958897L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)2016806904);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -1786774486);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)836759433);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)3906102190L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)73);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p128_accuracy_GET(pack) == (uint32_t)1916497142L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p128_tow_GET(pack) == (uint32_t)535457869L);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)1290874176L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -805299359);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)60403);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -163644426);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1007744307);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1685412661);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)8928);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)8762);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -24656);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1958407190L);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -22720);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -8351);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -27100);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -95);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -24924);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -7600);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)27403);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p130_size_GET(pack) == (uint32_t)2357108665L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)16517);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)40212);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)180, (uint8_t)153, (uint8_t)59, (uint8_t)162, (uint8_t)193, (uint8_t)213, (uint8_t)15, (uint8_t)69, (uint8_t)30, (uint8_t)217, (uint8_t)77, (uint8_t)85, (uint8_t)26, (uint8_t)7, (uint8_t)221, (uint8_t)195, (uint8_t)65, (uint8_t)170, (uint8_t)216, (uint8_t)208, (uint8_t)21, (uint8_t)199, (uint8_t)134, (uint8_t)182, (uint8_t)43, (uint8_t)206, (uint8_t)132, (uint8_t)199, (uint8_t)92, (uint8_t)36, (uint8_t)139, (uint8_t)202, (uint8_t)29, (uint8_t)182, (uint8_t)103, (uint8_t)164, (uint8_t)79, (uint8_t)170, (uint8_t)32, (uint8_t)97, (uint8_t)250, (uint8_t)88, (uint8_t)219, (uint8_t)31, (uint8_t)44, (uint8_t)164, (uint8_t)142, (uint8_t)104, (uint8_t)220, (uint8_t)209, (uint8_t)82, (uint8_t)214, (uint8_t)198, (uint8_t)107, (uint8_t)207, (uint8_t)89, (uint8_t)138, (uint8_t)82, (uint8_t)14, (uint8_t)212, (uint8_t)80, (uint8_t)239, (uint8_t)0, (uint8_t)79, (uint8_t)106, (uint8_t)218, (uint8_t)96, (uint8_t)79, (uint8_t)255, (uint8_t)123, (uint8_t)234, (uint8_t)218, (uint8_t)193, (uint8_t)91, (uint8_t)111, (uint8_t)20, (uint8_t)9, (uint8_t)249, (uint8_t)177, (uint8_t)133, (uint8_t)2, (uint8_t)70, (uint8_t)213, (uint8_t)183, (uint8_t)22, (uint8_t)58, (uint8_t)213, (uint8_t)41, (uint8_t)96, (uint8_t)42, (uint8_t)213, (uint8_t)250, (uint8_t)139, (uint8_t)90, (uint8_t)219, (uint8_t)206, (uint8_t)148, (uint8_t)177, (uint8_t)205, (uint8_t)96, (uint8_t)171, (uint8_t)203, (uint8_t)157, (uint8_t)137, (uint8_t)215, (uint8_t)179, (uint8_t)46, (uint8_t)78, (uint8_t)125, (uint8_t)89, (uint8_t)184, (uint8_t)95, (uint8_t)4, (uint8_t)130, (uint8_t)175, (uint8_t)156, (uint8_t)249, (uint8_t)14, (uint8_t)100, (uint8_t)157, (uint8_t)152, (uint8_t)122, (uint8_t)31, (uint8_t)76, (uint8_t)23, (uint8_t)111, (uint8_t)68, (uint8_t)196, (uint8_t)19, (uint8_t)72, (uint8_t)156, (uint8_t)219, (uint8_t)248, (uint8_t)74, (uint8_t)218, (uint8_t)93, (uint8_t)0, (uint8_t)194, (uint8_t)167, (uint8_t)5, (uint8_t)138, (uint8_t)249, (uint8_t)189, (uint8_t)144, (uint8_t)115, (uint8_t)241, (uint8_t)231, (uint8_t)87, (uint8_t)224, (uint8_t)125, (uint8_t)179, (uint8_t)195, (uint8_t)49, (uint8_t)180, (uint8_t)160, (uint8_t)239, (uint8_t)90, (uint8_t)222, (uint8_t)36, (uint8_t)59, (uint8_t)184, (uint8_t)171, (uint8_t)33, (uint8_t)93, (uint8_t)155, (uint8_t)231, (uint8_t)224, (uint8_t)2, (uint8_t)73, (uint8_t)35, (uint8_t)105, (uint8_t)129, (uint8_t)77, (uint8_t)245, (uint8_t)101, (uint8_t)158, (uint8_t)67, (uint8_t)166, (uint8_t)172, (uint8_t)239, (uint8_t)7, (uint8_t)167, (uint8_t)153, (uint8_t)182, (uint8_t)112, (uint8_t)106, (uint8_t)108, (uint8_t)208, (uint8_t)33, (uint8_t)146, (uint8_t)53, (uint8_t)46, (uint8_t)255, (uint8_t)207, (uint8_t)31, (uint8_t)181, (uint8_t)225, (uint8_t)116, (uint8_t)216, (uint8_t)43, (uint8_t)67, (uint8_t)141, (uint8_t)130, (uint8_t)69, (uint8_t)116, (uint8_t)240, (uint8_t)210, (uint8_t)236, (uint8_t)172, (uint8_t)31, (uint8_t)104, (uint8_t)11, (uint8_t)180, (uint8_t)254, (uint8_t)88, (uint8_t)205, (uint8_t)220, (uint8_t)169, (uint8_t)46, (uint8_t)247, (uint8_t)90, (uint8_t)94, (uint8_t)96, (uint8_t)255, (uint8_t)62, (uint8_t)84, (uint8_t)84, (uint8_t)224, (uint8_t)81, (uint8_t)99, (uint8_t)80, (uint8_t)255, (uint8_t)253, (uint8_t)175, (uint8_t)174, (uint8_t)204, (uint8_t)73, (uint8_t)213, (uint8_t)3, (uint8_t)45, (uint8_t)140, (uint8_t)213, (uint8_t)166, (uint8_t)181, (uint8_t)130, (uint8_t)99, (uint8_t)229, (uint8_t)202, (uint8_t)84, (uint8_t)159, (uint8_t)188, (uint8_t)12, (uint8_t)44} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)29687);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)49139);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1806799806L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)52494);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)36267);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lon_GET(pack) == (int32_t) -265485409);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)60286);
    assert(p133_lat_GET(pack) == (int32_t)700250427);
    assert(p133_mask_GET(pack) == (uint64_t)226161708013557847L);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)48639);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)112);
    {
        int16_t exemplary[] =  {(int16_t)16424, (int16_t) -11260, (int16_t)20187, (int16_t)4880, (int16_t) -28203, (int16_t) -24626, (int16_t)24512, (int16_t) -18363, (int16_t) -20329, (int16_t)18125, (int16_t)27339, (int16_t)372, (int16_t)10559, (int16_t) -4121, (int16_t)11913, (int16_t) -11405} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t) -858460987);
    assert(p134_lat_GET(pack) == (int32_t) -2063664651);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)216688408);
    assert(p135_lat_GET(pack) == (int32_t) -155611062);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lat_GET(pack) == (int32_t)1272809585);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)10718);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)6766);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)34357);
    assert(p136_terrain_height_GET(pack) == (float) -6.980262E37F);
    assert(p136_lon_GET(pack) == (int32_t)1327804119);
    assert(p136_current_height_GET(pack) == (float)3.333893E37F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float) -5.0367394E37F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -27537);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)2767625860L);
    assert(p137_press_diff_GET(pack) == (float) -9.4548346E35F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_z_GET(pack) == (float) -9.81521E37F);
    {
        float exemplary[] =  {-1.6825519E38F, -5.7302203E37F, -1.2200169E38F, 1.0237102E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float)2.0221032E37F);
    assert(p138_time_usec_GET(pack) == (uint64_t)2398244052133605070L);
    assert(p138_y_GET(pack) == (float)1.200075E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)4812012534207840281L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)58);
    {
        float exemplary[] =  {-1.0809938E38F, 2.953559E38F, -2.2081037E38F, 1.7026683E38F, 2.4622782E38F, 3.0976422E38F, 1.0592633E38F, -2.9436056E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)202);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-7.965937E36F, -3.2916335E38F, -1.3553233E38F, -3.2250295E38F, -2.6435632E38F, 2.3159084E38F, -2.3341163E38F, -2.0981883E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)1951597053655921652L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float) -1.4512852E36F);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.5503402E38F);
    assert(p141_altitude_local_GET(pack) == (float)2.8037407E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)1.5789403E37F);
    assert(p141_altitude_relative_GET(pack) == (float) -3.366431E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -1.5307039E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)3310766920882334836L);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)134);
    {
        uint8_t exemplary[] =  {(uint8_t)118, (uint8_t)136, (uint8_t)225, (uint8_t)234, (uint8_t)65, (uint8_t)223, (uint8_t)0, (uint8_t)137, (uint8_t)7, (uint8_t)89, (uint8_t)211, (uint8_t)135, (uint8_t)193, (uint8_t)25, (uint8_t)151, (uint8_t)45, (uint8_t)190, (uint8_t)21, (uint8_t)100, (uint8_t)54, (uint8_t)1, (uint8_t)26, (uint8_t)19, (uint8_t)116, (uint8_t)22, (uint8_t)28, (uint8_t)110, (uint8_t)98, (uint8_t)199, (uint8_t)235, (uint8_t)203, (uint8_t)97, (uint8_t)148, (uint8_t)220, (uint8_t)58, (uint8_t)164, (uint8_t)200, (uint8_t)144, (uint8_t)177, (uint8_t)216, (uint8_t)110, (uint8_t)136, (uint8_t)249, (uint8_t)151, (uint8_t)144, (uint8_t)126, (uint8_t)213, (uint8_t)145, (uint8_t)123, (uint8_t)176, (uint8_t)130, (uint8_t)126, (uint8_t)213, (uint8_t)112, (uint8_t)194, (uint8_t)244, (uint8_t)233, (uint8_t)129, (uint8_t)47, (uint8_t)29, (uint8_t)3, (uint8_t)97, (uint8_t)9, (uint8_t)130, (uint8_t)73, (uint8_t)27, (uint8_t)192, (uint8_t)6, (uint8_t)155, (uint8_t)83, (uint8_t)58, (uint8_t)142, (uint8_t)163, (uint8_t)62, (uint8_t)234, (uint8_t)133, (uint8_t)15, (uint8_t)238, (uint8_t)107, (uint8_t)78, (uint8_t)57, (uint8_t)104, (uint8_t)54, (uint8_t)124, (uint8_t)124, (uint8_t)203, (uint8_t)250, (uint8_t)57, (uint8_t)201, (uint8_t)90, (uint8_t)18, (uint8_t)103, (uint8_t)164, (uint8_t)220, (uint8_t)250, (uint8_t)185, (uint8_t)30, (uint8_t)91, (uint8_t)73, (uint8_t)35, (uint8_t)106, (uint8_t)12, (uint8_t)206, (uint8_t)150, (uint8_t)175, (uint8_t)166, (uint8_t)212, (uint8_t)173, (uint8_t)215, (uint8_t)64, (uint8_t)229, (uint8_t)49, (uint8_t)76, (uint8_t)13, (uint8_t)148, (uint8_t)150, (uint8_t)196, (uint8_t)155, (uint8_t)80, (uint8_t)10} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)150, (uint8_t)128, (uint8_t)129, (uint8_t)107, (uint8_t)49, (uint8_t)126, (uint8_t)11, (uint8_t)107, (uint8_t)107, (uint8_t)130, (uint8_t)61, (uint8_t)184, (uint8_t)82, (uint8_t)125, (uint8_t)239, (uint8_t)142, (uint8_t)53, (uint8_t)249, (uint8_t)193, (uint8_t)208, (uint8_t)138, (uint8_t)163, (uint8_t)136, (uint8_t)195, (uint8_t)112, (uint8_t)138, (uint8_t)146, (uint8_t)172, (uint8_t)205, (uint8_t)239, (uint8_t)26, (uint8_t)225, (uint8_t)31, (uint8_t)80, (uint8_t)124, (uint8_t)196, (uint8_t)154, (uint8_t)112, (uint8_t)185, (uint8_t)154, (uint8_t)97, (uint8_t)155, (uint8_t)31, (uint8_t)144, (uint8_t)61, (uint8_t)109, (uint8_t)219, (uint8_t)178, (uint8_t)119, (uint8_t)221, (uint8_t)169, (uint8_t)43, (uint8_t)38, (uint8_t)15, (uint8_t)164, (uint8_t)243, (uint8_t)177, (uint8_t)140, (uint8_t)147, (uint8_t)240, (uint8_t)124, (uint8_t)99, (uint8_t)220, (uint8_t)175, (uint8_t)68, (uint8_t)151, (uint8_t)24, (uint8_t)125, (uint8_t)239, (uint8_t)246, (uint8_t)59, (uint8_t)2, (uint8_t)127, (uint8_t)71, (uint8_t)228, (uint8_t)6, (uint8_t)179, (uint8_t)1, (uint8_t)242, (uint8_t)8, (uint8_t)9, (uint8_t)164, (uint8_t)128, (uint8_t)101, (uint8_t)51, (uint8_t)79, (uint8_t)16, (uint8_t)99, (uint8_t)129, (uint8_t)211, (uint8_t)79, (uint8_t)37, (uint8_t)90, (uint8_t)3, (uint8_t)18, (uint8_t)195, (uint8_t)91, (uint8_t)153, (uint8_t)157, (uint8_t)135, (uint8_t)53, (uint8_t)199, (uint8_t)18, (uint8_t)54, (uint8_t)178, (uint8_t)56, (uint8_t)184, (uint8_t)19, (uint8_t)226, (uint8_t)49, (uint8_t)186, (uint8_t)93, (uint8_t)232, (uint8_t)127, (uint8_t)241, (uint8_t)227, (uint8_t)160, (uint8_t)166, (uint8_t)104, (uint8_t)239} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -18076);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3240972552L);
    assert(p143_press_abs_GET(pack) == (float)2.0003854E37F);
    assert(p143_press_diff_GET(pack) == (float)1.9896304E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.2384128E38F, 1.6639044E38F, -1.2413826E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)9);
    {
        float exemplary[] =  {2.410588E38F, 1.9170337E38F, -1.5112452E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)340614057);
    {
        float exemplary[] =  {9.404818E37F, -1.7179199E38F, 2.6446526E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)3817421245373467765L);
    {
        float exemplary[] =  {2.2767098E38F, -1.0610788E38F, -4.7190484E37F, 2.9480178E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)2933913172635936189L);
    {
        float exemplary[] =  {-3.5204995E37F, 2.6464125E38F, -1.5242135E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)1.4237775E38F);
    assert(p144_lat_GET(pack) == (int32_t) -818899896);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_z_pos_GET(pack) == (float) -2.6546812E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.507402E38F);
    assert(p146_z_vel_GET(pack) == (float)3.2140311E38F);
    assert(p146_roll_rate_GET(pack) == (float) -2.4099232E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)2018212447498306088L);
    assert(p146_pitch_rate_GET(pack) == (float)1.3322482E38F);
    assert(p146_y_pos_GET(pack) == (float)2.2540122E38F);
    assert(p146_z_acc_GET(pack) == (float) -8.0365463E37F);
    assert(p146_x_pos_GET(pack) == (float) -3.2032087E38F);
    assert(p146_airspeed_GET(pack) == (float) -1.739689E38F);
    assert(p146_x_acc_GET(pack) == (float)2.155039E38F);
    {
        float exemplary[] =  {3.8466939E37F, 1.9940373E37F, -1.6056338E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float) -2.9230636E38F);
    {
        float exemplary[] =  {-2.0764512E38F, 8.510974E37F, 2.473523E38F, -3.3946685E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.6276689E38F, 1.3297273E38F, -2.428628E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float) -3.7477015E37F);
    assert(p146_y_acc_GET(pack) == (float)7.3436045E36F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)12530);
    {
        uint16_t exemplary[] =  {(uint16_t)13654, (uint16_t)20284, (uint16_t)17962, (uint16_t)13063, (uint16_t)27123, (uint16_t)41567, (uint16_t)57755, (uint16_t)10987, (uint16_t)21801, (uint16_t)12750} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -1192368493);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)90);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)9718);
    assert(p147_current_consumed_GET(pack) == (int32_t)102835190);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)386754083L);
    {
        uint8_t exemplary[] =  {(uint8_t)154, (uint8_t)193, (uint8_t)25, (uint8_t)162, (uint8_t)217, (uint8_t)25, (uint8_t)35, (uint8_t)145, (uint8_t)224, (uint8_t)20, (uint8_t)197, (uint8_t)45, (uint8_t)105, (uint8_t)62, (uint8_t)140, (uint8_t)146, (uint8_t)86, (uint8_t)87} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)8134379424959737666L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)990375664L);
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)121, (uint8_t)167, (uint8_t)148, (uint8_t)236, (uint8_t)80, (uint8_t)242, (uint8_t)34} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)37477);
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT));
    {
        uint8_t exemplary[] =  {(uint8_t)186, (uint8_t)23, (uint8_t)177, (uint8_t)227, (uint8_t)125, (uint8_t)224, (uint8_t)171, (uint8_t)109} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)26686);
    {
        uint8_t exemplary[] =  {(uint8_t)25, (uint8_t)169, (uint8_t)100, (uint8_t)66, (uint8_t)247, (uint8_t)100, (uint8_t)237, (uint8_t)5} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)2159590131L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)829175315L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_size_y_GET(pack) == (float) -2.3204162E38F);
    assert(p149_angle_y_GET(pack) == (float) -1.9481183E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)2318874480956988797L);
    assert(p149_size_x_GET(pack) == (float)2.1897512E38F);
    assert(p149_z_TRY(ph) == (float)1.5471252E38F);
    {
        float exemplary[] =  {3.7198185E37F, 1.0602378E38F, -2.2280415E37F, -1.1215918E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p149_distance_GET(pack) == (float) -2.8338082E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p149_angle_x_GET(pack) == (float) -7.2453087E37F);
    assert(p149_y_TRY(ph) == (float)8.793153E37F);
    assert(p149_x_TRY(ph) == (float)1.1190085E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)228);
};


void c_CommunicationChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_gyro_cal_z_GET(pack) == (float) -2.295566E38F);
    assert(p150_raw_temp_GET(pack) == (int32_t)14578107);
    assert(p150_accel_cal_z_GET(pack) == (float) -2.72759E38F);
    assert(p150_gyro_cal_y_GET(pack) == (float)1.3704507E37F);
    assert(p150_mag_ofs_x_GET(pack) == (int16_t)(int16_t) -9261);
    assert(p150_accel_cal_x_GET(pack) == (float) -2.6226803E38F);
    assert(p150_gyro_cal_x_GET(pack) == (float) -1.5491786E38F);
    assert(p150_mag_declination_GET(pack) == (float)2.414507E38F);
    assert(p150_mag_ofs_y_GET(pack) == (int16_t)(int16_t)22394);
    assert(p150_raw_press_GET(pack) == (int32_t)503066702);
    assert(p150_accel_cal_y_GET(pack) == (float) -3.3353135E38F);
    assert(p150_mag_ofs_z_GET(pack) == (int16_t)(int16_t)9311);
};


void c_CommunicationChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_mag_ofs_z_GET(pack) == (int16_t)(int16_t)24110);
    assert(p151_mag_ofs_x_GET(pack) == (int16_t)(int16_t) -27626);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p151_mag_ofs_y_GET(pack) == (int16_t)(int16_t)6611);
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_CommunicationChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_freemem_GET(pack) == (uint16_t)(uint16_t)23606);
    assert(p152_freemem32_TRY(ph) == (uint32_t)2206115995L);
    assert(p152_brkval_GET(pack) == (uint16_t)(uint16_t)46330);
};


void c_CommunicationChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_adc3_GET(pack) == (uint16_t)(uint16_t)57669);
    assert(p153_adc2_GET(pack) == (uint16_t)(uint16_t)39303);
    assert(p153_adc5_GET(pack) == (uint16_t)(uint16_t)2807);
    assert(p153_adc4_GET(pack) == (uint16_t)(uint16_t)10236);
    assert(p153_adc6_GET(pack) == (uint16_t)(uint16_t)51293);
    assert(p153_adc1_GET(pack) == (uint16_t)(uint16_t)47928);
};


void c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    assert(p154_iso_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p154_shutter_speed_GET(pack) == (uint16_t)(uint16_t)19377);
    assert(p154_engine_cut_off_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p154_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p154_exposure_type_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p154_target_component_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p154_extra_value_GET(pack) == (float)1.1613895E37F);
    assert(p154_command_id_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p154_extra_param_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p154_aperture_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p154_mode_GET(pack) == (uint8_t)(uint8_t)210);
};


void c_CommunicationChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_focus_lock_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p155_zoom_step_GET(pack) == (int8_t)(int8_t) -66);
    assert(p155_extra_value_GET(pack) == (float) -3.294074E38F);
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p155_extra_param_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p155_shot_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p155_command_id_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p155_zoom_pos_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p155_session_GET(pack) == (uint8_t)(uint8_t)162);
};


void c_CommunicationChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_stab_roll_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p156_stab_yaw_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p156_mount_mode_GET(pack) == e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p156_stab_pitch_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_CommunicationChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_input_a_GET(pack) == (int32_t) -1410429504);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p157_input_b_GET(pack) == (int32_t) -1418641959);
    assert(p157_input_c_GET(pack) == (int32_t)1736843062);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p157_save_position_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_CommunicationChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_target_component_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p158_target_system_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p158_pointing_c_GET(pack) == (int32_t) -640303132);
    assert(p158_pointing_a_GET(pack) == (int32_t)1521328363);
    assert(p158_pointing_b_GET(pack) == (int32_t)5041733);
};


void c_CommunicationChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    assert(p160_count_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p160_lat_GET(pack) == (float) -1.2652158E38F);
    assert(p160_idx_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p160_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p160_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p160_lng_GET(pack) == (float)1.85313E38F);
};


void c_CommunicationChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    assert(p161_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p161_target_system_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p161_idx_GET(pack) == (uint8_t)(uint8_t)127);
};


void c_CommunicationChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    assert(p162_breach_status_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p162_breach_type_GET(pack) == e_FENCE_BREACH_FENCE_BREACH_MAXALT);
    assert(p162_breach_count_GET(pack) == (uint16_t)(uint16_t)12368);
    assert(p162_breach_time_GET(pack) == (uint32_t)1056811203L);
};


void c_CommunicationChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    assert(p163_omegaIx_GET(pack) == (float) -5.477533E37F);
    assert(p163_omegaIy_GET(pack) == (float)3.845038E37F);
    assert(p163_error_rp_GET(pack) == (float) -2.3232701E38F);
    assert(p163_error_yaw_GET(pack) == (float)3.341284E38F);
    assert(p163_omegaIz_GET(pack) == (float) -1.7242366E38F);
    assert(p163_accel_weight_GET(pack) == (float) -1.3689269E38F);
    assert(p163_renorm_val_GET(pack) == (float)2.5822647E38F);
};


void c_CommunicationChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    assert(p164_xgyro_GET(pack) == (float)1.5215825E38F);
    assert(p164_ygyro_GET(pack) == (float) -2.3258208E38F);
    assert(p164_lng_GET(pack) == (int32_t) -1875901161);
    assert(p164_zgyro_GET(pack) == (float)2.423396E38F);
    assert(p164_yacc_GET(pack) == (float) -1.5755294E38F);
    assert(p164_lat_GET(pack) == (int32_t)1205239707);
    assert(p164_roll_GET(pack) == (float) -1.4262921E38F);
    assert(p164_pitch_GET(pack) == (float)2.7210398E38F);
    assert(p164_yaw_GET(pack) == (float)3.275304E38F);
    assert(p164_xacc_GET(pack) == (float)1.1208787E37F);
    assert(p164_zacc_GET(pack) == (float)1.7711585E37F);
};


void c_CommunicationChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    assert(p165_I2Cerr_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p165_Vcc_GET(pack) == (uint16_t)(uint16_t)47956);
};


void c_CommunicationChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    assert(p166_rssi_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p166_rxerrors_GET(pack) == (uint16_t)(uint16_t)6362);
    assert(p166_remrssi_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p166_noise_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p166_remnoise_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p166_fixed__GET(pack) == (uint16_t)(uint16_t)18137);
    assert(p166_txbuf_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_CommunicationChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    assert(p167_mods_triggered_GET(pack) == (e_LIMIT_MODULE_LIMIT_GPSLOCK));
    assert(p167_breach_count_GET(pack) == (uint16_t)(uint16_t)39664);
    assert(p167_last_clear_GET(pack) == (uint32_t)775621687L);
    assert(p167_mods_required_GET(pack) == (e_LIMIT_MODULE_LIMIT_GEOFENCE |
                                            e_LIMIT_MODULE_LIMIT_GPSLOCK));
    assert(p167_mods_enabled_GET(pack) == (e_LIMIT_MODULE_LIMIT_GEOFENCE));
    assert(p167_limits_state_GET(pack) == e_LIMITS_STATE_LIMITS_INIT);
    assert(p167_last_action_GET(pack) == (uint32_t)1777506543L);
    assert(p167_last_recovery_GET(pack) == (uint32_t)2414151283L);
    assert(p167_last_trigger_GET(pack) == (uint32_t)781938888L);
};


void c_CommunicationChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    assert(p168_speed_GET(pack) == (float) -1.1065599E38F);
    assert(p168_speed_z_GET(pack) == (float)1.9983788E37F);
    assert(p168_direction_GET(pack) == (float)2.4293781E38F);
};


void c_CommunicationChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)123, (uint8_t)64, (uint8_t)6, (uint8_t)70, (uint8_t)107, (uint8_t)134, (uint8_t)89, (uint8_t)100, (uint8_t)177, (uint8_t)122, (uint8_t)84, (uint8_t)44, (uint8_t)242, (uint8_t)82, (uint8_t)135, (uint8_t)147} ;
        uint8_t*  sample = p169_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p169_len_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p169_type_GET(pack) == (uint8_t)(uint8_t)111);
};


void c_CommunicationChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_len_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p170_type_GET(pack) == (uint8_t)(uint8_t)201);
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)156, (uint8_t)35, (uint8_t)219, (uint8_t)14, (uint8_t)82, (uint8_t)36, (uint8_t)118, (uint8_t)146, (uint8_t)61, (uint8_t)177, (uint8_t)141, (uint8_t)74, (uint8_t)111, (uint8_t)10, (uint8_t)72, (uint8_t)207, (uint8_t)75, (uint8_t)43, (uint8_t)62, (uint8_t)3, (uint8_t)161, (uint8_t)213, (uint8_t)193, (uint8_t)143, (uint8_t)43, (uint8_t)211, (uint8_t)183, (uint8_t)2, (uint8_t)3, (uint8_t)62, (uint8_t)46} ;
        uint8_t*  sample = p170_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_type_GET(pack) == (uint8_t)(uint8_t)194);
    {
        uint8_t exemplary[] =  {(uint8_t)22, (uint8_t)23, (uint8_t)3, (uint8_t)239, (uint8_t)50, (uint8_t)200, (uint8_t)26, (uint8_t)210, (uint8_t)86, (uint8_t)91, (uint8_t)138, (uint8_t)125, (uint8_t)41, (uint8_t)4, (uint8_t)87, (uint8_t)13, (uint8_t)40, (uint8_t)105, (uint8_t)237, (uint8_t)95, (uint8_t)62, (uint8_t)101, (uint8_t)31, (uint8_t)192, (uint8_t)94, (uint8_t)243, (uint8_t)103, (uint8_t)55, (uint8_t)209, (uint8_t)65, (uint8_t)96, (uint8_t)237, (uint8_t)47, (uint8_t)189, (uint8_t)25, (uint8_t)167, (uint8_t)137, (uint8_t)13, (uint8_t)169, (uint8_t)136, (uint8_t)159, (uint8_t)73, (uint8_t)236, (uint8_t)174, (uint8_t)102, (uint8_t)212, (uint8_t)31, (uint8_t)3, (uint8_t)170, (uint8_t)175, (uint8_t)185, (uint8_t)57, (uint8_t)190, (uint8_t)106, (uint8_t)65, (uint8_t)190, (uint8_t)193, (uint8_t)76, (uint8_t)196, (uint8_t)141, (uint8_t)206, (uint8_t)108, (uint8_t)53, (uint8_t)235} ;
        uint8_t*  sample = p171_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p171_len_GET(pack) == (uint8_t)(uint8_t)68);
};


void c_CommunicationChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)214, (uint8_t)173, (uint8_t)36, (uint8_t)237, (uint8_t)70, (uint8_t)63, (uint8_t)228, (uint8_t)89, (uint8_t)220, (uint8_t)34, (uint8_t)165, (uint8_t)13, (uint8_t)253, (uint8_t)136, (uint8_t)79, (uint8_t)198, (uint8_t)79, (uint8_t)1, (uint8_t)160, (uint8_t)83, (uint8_t)200, (uint8_t)224, (uint8_t)26, (uint8_t)169, (uint8_t)75, (uint8_t)249, (uint8_t)86, (uint8_t)67, (uint8_t)0, (uint8_t)203, (uint8_t)175, (uint8_t)206, (uint8_t)8, (uint8_t)81, (uint8_t)104, (uint8_t)165, (uint8_t)140, (uint8_t)77, (uint8_t)245, (uint8_t)100, (uint8_t)238, (uint8_t)223, (uint8_t)24, (uint8_t)96, (uint8_t)19, (uint8_t)57, (uint8_t)108, (uint8_t)16, (uint8_t)69, (uint8_t)229, (uint8_t)245, (uint8_t)0, (uint8_t)26, (uint8_t)71, (uint8_t)95, (uint8_t)243, (uint8_t)55, (uint8_t)193, (uint8_t)110, (uint8_t)196, (uint8_t)14, (uint8_t)104, (uint8_t)112, (uint8_t)94, (uint8_t)57, (uint8_t)241, (uint8_t)164, (uint8_t)91, (uint8_t)36, (uint8_t)169, (uint8_t)49, (uint8_t)248, (uint8_t)107, (uint8_t)189, (uint8_t)95, (uint8_t)12, (uint8_t)85, (uint8_t)222, (uint8_t)222, (uint8_t)228, (uint8_t)193, (uint8_t)139, (uint8_t)71, (uint8_t)59, (uint8_t)37, (uint8_t)16, (uint8_t)144, (uint8_t)237, (uint8_t)70, (uint8_t)189, (uint8_t)214, (uint8_t)25, (uint8_t)188, (uint8_t)76, (uint8_t)251} ;
        uint8_t*  sample = p172_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p172_len_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p172_type_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_CommunicationChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_voltage_GET(pack) == (float) -1.8790527E38F);
    assert(p173_distance_GET(pack) == (float)2.561586E38F);
};


void c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_Pby_GET(pack) == (float) -1.5351117E38F);
    assert(p174_Pcz_GET(pack) == (float) -2.3267228E38F);
    assert(p174_vx_GET(pack) == (float)1.1364388E38F);
    assert(p174_diff_pressure_GET(pack) == (float) -7.262932E37F);
    assert(p174_vy_GET(pack) == (float)2.430164E38F);
    assert(p174_EAS2TAS_GET(pack) == (float) -1.9346427E38F);
    assert(p174_vz_GET(pack) == (float)3.0419742E38F);
    assert(p174_state_x_GET(pack) == (float)3.111261E38F);
    assert(p174_state_z_GET(pack) == (float)1.981293E38F);
    assert(p174_state_y_GET(pack) == (float) -1.4023957E38F);
    assert(p174_ratio_GET(pack) == (float) -2.1917476E38F);
    assert(p174_Pax_GET(pack) == (float) -2.5706285E38F);
};


void c_CommunicationChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_target_system_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p175_count_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p175_break_alt_GET(pack) == (int16_t)(int16_t) -13976);
    assert(p175_alt_GET(pack) == (int16_t)(int16_t) -15879);
    assert(p175_idx_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p175_lat_GET(pack) == (int32_t)103367418);
    assert(p175_flags_GET(pack) == e_RALLY_FLAGS_FAVORABLE_WIND);
    assert(p175_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p175_land_dir_GET(pack) == (uint16_t)(uint16_t)48746);
    assert(p175_lng_GET(pack) == (int32_t) -844794889);
};


void c_CommunicationChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_target_system_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p176_target_component_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p176_idx_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_CommunicationChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_CompensationX_GET(pack) == (float) -1.1624441E38F);
    assert(p177_CompensationZ_GET(pack) == (float) -3.1142157E38F);
    assert(p177_interference_GET(pack) == (uint16_t)(uint16_t)55005);
    assert(p177_current_GET(pack) == (float) -3.0409922E38F);
    assert(p177_throttle_GET(pack) == (uint16_t)(uint16_t)31857);
    assert(p177_CompensationY_GET(pack) == (float) -9.063226E37F);
};


void c_CommunicationChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_altitude_GET(pack) == (float)3.3275465E38F);
    assert(p178_pitch_GET(pack) == (float) -2.3437722E38F);
    assert(p178_lat_GET(pack) == (int32_t)2049905234);
    assert(p178_yaw_GET(pack) == (float) -2.6190503E37F);
    assert(p178_lng_GET(pack) == (int32_t)1436887039);
    assert(p178_roll_GET(pack) == (float)1.0544186E38F);
};


void c_CommunicationChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_p4_GET(pack) == (float)1.8058774E38F);
    assert(p179_time_usec_GET(pack) == (uint64_t)2993184925173995351L);
    assert(p179_img_idx_GET(pack) == (uint16_t)(uint16_t)20356);
    assert(p179_p1_GET(pack) == (float)5.6325494E37F);
    assert(p179_target_system_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p179_cam_idx_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p179_p2_GET(pack) == (float) -2.9105925E38F);
    assert(p179_event_id_GET(pack) == e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER);
    assert(p179_p3_GET(pack) == (float)1.3779528E38F);
};


void c_CommunicationChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_roll_GET(pack) == (float)2.5728873E38F);
    assert(p180_alt_rel_GET(pack) == (float)2.922496E38F);
    assert(p180_lng_GET(pack) == (int32_t) -1930845716);
    assert(p180_cam_idx_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p180_pitch_GET(pack) == (float) -1.47088E38F);
    assert(p180_flags_GET(pack) == e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_BADEXPOSURE);
    assert(p180_img_idx_GET(pack) == (uint16_t)(uint16_t)61261);
    assert(p180_lat_GET(pack) == (int32_t) -938431042);
    assert(p180_foc_len_GET(pack) == (float) -1.5871107E38F);
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p180_yaw_GET(pack) == (float) -2.1199742E38F);
    assert(p180_alt_msl_GET(pack) == (float) -2.7157475E38F);
    assert(p180_time_usec_GET(pack) == (uint64_t)8155565993618783924L);
};


void c_CommunicationChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_current_battery_GET(pack) == (int16_t)(int16_t) -24475);
    assert(p181_voltage_GET(pack) == (uint16_t)(uint16_t)64954);
};


void c_CommunicationChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_yaw_GET(pack) == (float) -2.7070143E38F);
    assert(p182_v1_GET(pack) == (float)2.549625E38F);
    assert(p182_roll_GET(pack) == (float)2.0357053E38F);
    assert(p182_v3_GET(pack) == (float) -1.0307818E38F);
    assert(p182_lng_GET(pack) == (int32_t)274290649);
    assert(p182_altitude_GET(pack) == (float) -4.5911354E37F);
    assert(p182_pitch_GET(pack) == (float) -2.8939772E38F);
    assert(p182_v4_GET(pack) == (float) -2.1188601E38F);
    assert(p182_v2_GET(pack) == (float)1.0528232E38F);
    assert(p182_lat_GET(pack) == (int32_t) -2031478790);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)152);
};


void c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_seqno_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP);
    assert(p184_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    {
        uint8_t exemplary[] =  {(uint8_t)241, (uint8_t)227, (uint8_t)127, (uint8_t)40, (uint8_t)197, (uint8_t)116, (uint8_t)160, (uint8_t)254, (uint8_t)59, (uint8_t)42, (uint8_t)251, (uint8_t)6, (uint8_t)124, (uint8_t)140, (uint8_t)28, (uint8_t)93, (uint8_t)77, (uint8_t)41, (uint8_t)245, (uint8_t)92, (uint8_t)45, (uint8_t)106, (uint8_t)12, (uint8_t)155, (uint8_t)171, (uint8_t)120, (uint8_t)91, (uint8_t)178, (uint8_t)239, (uint8_t)229, (uint8_t)254, (uint8_t)190, (uint8_t)32, (uint8_t)53, (uint8_t)178, (uint8_t)174, (uint8_t)139, (uint8_t)236, (uint8_t)252, (uint8_t)232, (uint8_t)180, (uint8_t)111, (uint8_t)13, (uint8_t)73, (uint8_t)183, (uint8_t)195, (uint8_t)57, (uint8_t)43, (uint8_t)54, (uint8_t)202, (uint8_t)245, (uint8_t)169, (uint8_t)32, (uint8_t)86, (uint8_t)33, (uint8_t)164, (uint8_t)92, (uint8_t)228, (uint8_t)77, (uint8_t)145, (uint8_t)224, (uint8_t)39, (uint8_t)187, (uint8_t)86, (uint8_t)219, (uint8_t)226, (uint8_t)105, (uint8_t)192, (uint8_t)132, (uint8_t)241, (uint8_t)4, (uint8_t)164, (uint8_t)168, (uint8_t)38, (uint8_t)137, (uint8_t)123, (uint8_t)173, (uint8_t)220, (uint8_t)227, (uint8_t)40, (uint8_t)57, (uint8_t)203, (uint8_t)37, (uint8_t)124, (uint8_t)128, (uint8_t)140, (uint8_t)170, (uint8_t)11, (uint8_t)236, (uint8_t)208, (uint8_t)65, (uint8_t)178, (uint8_t)208, (uint8_t)46, (uint8_t)175, (uint8_t)208, (uint8_t)32, (uint8_t)88, (uint8_t)168, (uint8_t)52, (uint8_t)248, (uint8_t)203, (uint8_t)30, (uint8_t)231, (uint8_t)176, (uint8_t)11, (uint8_t)36, (uint8_t)234, (uint8_t)187, (uint8_t)202, (uint8_t)152, (uint8_t)146, (uint8_t)106, (uint8_t)194, (uint8_t)113, (uint8_t)2, (uint8_t)241, (uint8_t)37, (uint8_t)106, (uint8_t)245, (uint8_t)27, (uint8_t)177, (uint8_t)120, (uint8_t)252, (uint8_t)156, (uint8_t)249, (uint8_t)175, (uint8_t)117, (uint8_t)181, (uint8_t)203, (uint8_t)99, (uint8_t)115, (uint8_t)47, (uint8_t)2, (uint8_t)75, (uint8_t)145, (uint8_t)104, (uint8_t)66, (uint8_t)169, (uint8_t)153, (uint8_t)222, (uint8_t)58, (uint8_t)73, (uint8_t)161, (uint8_t)174, (uint8_t)22, (uint8_t)39, (uint8_t)78, (uint8_t)34, (uint8_t)7, (uint8_t)39, (uint8_t)70, (uint8_t)149, (uint8_t)165, (uint8_t)144, (uint8_t)71, (uint8_t)53, (uint8_t)25, (uint8_t)219, (uint8_t)97, (uint8_t)12, (uint8_t)99, (uint8_t)21, (uint8_t)186, (uint8_t)185, (uint8_t)168, (uint8_t)10, (uint8_t)99, (uint8_t)198, (uint8_t)1, (uint8_t)191, (uint8_t)96, (uint8_t)86, (uint8_t)42, (uint8_t)70, (uint8_t)109, (uint8_t)131, (uint8_t)248, (uint8_t)199, (uint8_t)229, (uint8_t)31, (uint8_t)83, (uint8_t)179, (uint8_t)171, (uint8_t)66, (uint8_t)247, (uint8_t)230, (uint8_t)124, (uint8_t)187, (uint8_t)239, (uint8_t)183, (uint8_t)237, (uint8_t)236, (uint8_t)247, (uint8_t)31, (uint8_t)243, (uint8_t)188, (uint8_t)55, (uint8_t)203, (uint8_t)196} ;
        uint8_t*  sample = p184_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 200);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p184_target_component_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_seqno_GET(pack) == (uint32_t)3253929960L);
    assert(p185_target_system_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p185_target_component_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p185_status_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK);
};


void c_CommunicationChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_target_system_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p186_instance_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p186_pattern_GET(pack) == (uint8_t)(uint8_t)173);
    {
        uint8_t exemplary[] =  {(uint8_t)241, (uint8_t)169, (uint8_t)132, (uint8_t)10, (uint8_t)218, (uint8_t)73, (uint8_t)8, (uint8_t)69, (uint8_t)54, (uint8_t)144, (uint8_t)121, (uint8_t)200, (uint8_t)142, (uint8_t)45, (uint8_t)230, (uint8_t)250, (uint8_t)246, (uint8_t)70, (uint8_t)121, (uint8_t)105, (uint8_t)18, (uint8_t)43, (uint8_t)124, (uint8_t)0} ;
        uint8_t*  sample = p186_custom_bytes_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p186_target_component_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p186_custom_len_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_direction_z_GET(pack) == (float)3.041614E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)112, (uint8_t)217, (uint8_t)77, (uint8_t)144, (uint8_t)97, (uint8_t)229, (uint8_t)91, (uint8_t)124, (uint8_t)33, (uint8_t)183} ;
        uint8_t*  sample = p191_completion_mask_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p191_cal_mask_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p191_attempt_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p191_direction_x_GET(pack) == (float) -2.8114162E38F);
    assert(p191_compass_id_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p191_direction_y_GET(pack) == (float) -6.188112E37F);
    assert(p191_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE);
    assert(p191_completion_pct_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_CommunicationChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_ofs_y_GET(pack) == (float)1.3234556E38F);
    assert(p192_offdiag_z_GET(pack) == (float) -2.5587872E38F);
    assert(p192_compass_id_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p192_autosaved_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p192_cal_mask_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p192_ofs_x_GET(pack) == (float)9.731662E37F);
    assert(p192_diag_x_GET(pack) == (float) -9.16103E36F);
    assert(p192_diag_y_GET(pack) == (float) -2.1910896E38F);
    assert(p192_offdiag_x_GET(pack) == (float)1.760406E38F);
    assert(p192_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED);
    assert(p192_fitness_GET(pack) == (float) -9.897789E37F);
    assert(p192_diag_z_GET(pack) == (float) -1.3230701E38F);
    assert(p192_ofs_z_GET(pack) == (float) -1.7167431E38F);
    assert(p192_offdiag_y_GET(pack) == (float) -2.857091E38F);
};


void c_CommunicationChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_pos_vert_variance_GET(pack) == (float) -2.9666584E37F);
    assert(p193_terrain_alt_variance_GET(pack) == (float) -1.540064E38F);
    assert(p193_compass_variance_GET(pack) == (float) -1.6123477E38F);
    assert(p193_velocity_variance_GET(pack) == (float)1.5838299E38F);
    assert(p193_flags_GET(pack) == (e_EKF_STATUS_FLAGS_EKF_ATTITUDE |
                                    e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT |
                                    e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL |
                                    e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE |
                                    e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS));
    assert(p193_pos_horiz_variance_GET(pack) == (float) -3.1226725E38F);
};


void c_CommunicationChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_P_GET(pack) == (float) -1.6272534E38F);
    assert(p194_achieved_GET(pack) == (float)1.118314E38F);
    assert(p194_FF_GET(pack) == (float) -2.8814376E38F);
    assert(p194_desired_GET(pack) == (float) -3.150835E38F);
    assert(p194_D_GET(pack) == (float) -1.7362124E38F);
    assert(p194_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_STEER);
    assert(p194_I_GET(pack) == (float)2.2600286E37F);
};


void c_CommunicationChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    assert(p200_joint_el_GET(pack) == (float)2.3766834E38F);
    assert(p200_delta_angle_z_GET(pack) == (float)2.0236332E38F);
    assert(p200_target_component_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p200_joint_az_GET(pack) == (float) -3.2481993E38F);
    assert(p200_delta_time_GET(pack) == (float) -1.6660262E38F);
    assert(p200_delta_velocity_x_GET(pack) == (float)2.2162705E38F);
    assert(p200_delta_angle_y_GET(pack) == (float)1.0392357E38F);
    assert(p200_delta_velocity_y_GET(pack) == (float)1.739594E38F);
    assert(p200_delta_velocity_z_GET(pack) == (float) -2.5588347E38F);
    assert(p200_joint_roll_GET(pack) == (float) -2.4109295E38F);
    assert(p200_target_system_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p200_delta_angle_x_GET(pack) == (float)1.5241133E38F);
};


void c_CommunicationChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_demanded_rate_y_GET(pack) == (float)2.2219242E38F);
    assert(p201_demanded_rate_x_GET(pack) == (float)2.8767014E38F);
    assert(p201_demanded_rate_z_GET(pack) == (float) -1.2787064E37F);
    assert(p201_target_system_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p201_target_component_GET(pack) == (uint8_t)(uint8_t)162);
};


void c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    assert(p214_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p214_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p214_rl_torque_cmd_GET(pack) == (int16_t)(int16_t)7137);
    assert(p214_az_torque_cmd_GET(pack) == (int16_t)(int16_t) -20845);
    assert(p214_el_torque_cmd_GET(pack) == (int16_t)(int16_t)3552);
};


void c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    assert(p215_flags_GET(pack) == e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING);
    assert(p215_capture_mode_GET(pack) == e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE);
    assert(p215_status_GET(pack) == e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED);
};


void c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    assert(p216_target_component_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p216_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p216_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT);
};


void c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    assert(p217_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE);
    assert(p217_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS);
    {
        uint8_t exemplary[] =  {(uint8_t)201, (uint8_t)219, (uint8_t)171, (uint8_t)159} ;
        uint8_t*  sample = p217_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    assert(p218_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN);
    assert(p218_target_system_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p218_target_component_GET(pack) == (uint8_t)(uint8_t)199);
    {
        uint8_t exemplary[] =  {(uint8_t)230, (uint8_t)240, (uint8_t)220, (uint8_t)97} ;
        uint8_t*  sample = p218_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    assert(p219_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS);
    assert(p219_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS);
};


void c_CommunicationChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    assert(p226_rpm2_GET(pack) == (float)1.0434025E38F);
    assert(p226_rpm1_GET(pack) == (float)7.4162626E37F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_tas_ratio_GET(pack) == (float)4.1985894E37F);
    assert(p230_hagl_ratio_GET(pack) == (float) -6.4971383E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -2.4293223E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.176877E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -8.193581E37F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -3.298805E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL));
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -1.9911106E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)6506622306616413683L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)1.992366E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_z_GET(pack) == (float) -2.2825828E38F);
    assert(p231_wind_y_GET(pack) == (float) -2.657095E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -1.941195E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)2590716585105715623L);
    assert(p231_var_horiz_GET(pack) == (float) -1.9095134E38F);
    assert(p231_wind_x_GET(pack) == (float)2.2652934E38F);
    assert(p231_var_vert_GET(pack) == (float) -2.8532318E38F);
    assert(p231_wind_alt_GET(pack) == (float) -3.1495783E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -2.2426982E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_hdop_GET(pack) == (float)7.80131E37F);
    assert(p232_speed_accuracy_GET(pack) == (float)2.7083424E38F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)39986);
    assert(p232_alt_GET(pack) == (float)2.0383698E38F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
    assert(p232_time_usec_GET(pack) == (uint64_t)1232513044434789164L);
    assert(p232_vert_accuracy_GET(pack) == (float)2.8893491E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)444591517L);
    assert(p232_vd_GET(pack) == (float)1.6731291E38F);
    assert(p232_lon_GET(pack) == (int32_t)816464614);
    assert(p232_ve_GET(pack) == (float) -2.4769426E38F);
    assert(p232_vn_GET(pack) == (float)5.155052E37F);
    assert(p232_horiz_accuracy_GET(pack) == (float)2.9227922E38F);
    assert(p232_vdop_GET(pack) == (float)1.834544E38F);
    assert(p232_lat_GET(pack) == (int32_t)617627158);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)249, (uint8_t)57, (uint8_t)73, (uint8_t)236, (uint8_t)67, (uint8_t)30, (uint8_t)88, (uint8_t)108, (uint8_t)85, (uint8_t)28, (uint8_t)20, (uint8_t)87, (uint8_t)0, (uint8_t)121, (uint8_t)2, (uint8_t)3, (uint8_t)37, (uint8_t)97, (uint8_t)213, (uint8_t)124, (uint8_t)81, (uint8_t)160, (uint8_t)151, (uint8_t)173, (uint8_t)38, (uint8_t)37, (uint8_t)112, (uint8_t)218, (uint8_t)209, (uint8_t)125, (uint8_t)170, (uint8_t)105, (uint8_t)76, (uint8_t)55, (uint8_t)52, (uint8_t)249, (uint8_t)60, (uint8_t)100, (uint8_t)154, (uint8_t)195, (uint8_t)68, (uint8_t)140, (uint8_t)127, (uint8_t)215, (uint8_t)64, (uint8_t)98, (uint8_t)18, (uint8_t)191, (uint8_t)231, (uint8_t)228, (uint8_t)17, (uint8_t)180, (uint8_t)241, (uint8_t)97, (uint8_t)82, (uint8_t)216, (uint8_t)230, (uint8_t)187, (uint8_t)223, (uint8_t)219, (uint8_t)202, (uint8_t)118, (uint8_t)145, (uint8_t)12, (uint8_t)176, (uint8_t)68, (uint8_t)181, (uint8_t)33, (uint8_t)98, (uint8_t)215, (uint8_t)117, (uint8_t)59, (uint8_t)124, (uint8_t)0, (uint8_t)43, (uint8_t)251, (uint8_t)1, (uint8_t)137, (uint8_t)35, (uint8_t)163, (uint8_t)167, (uint8_t)15, (uint8_t)75, (uint8_t)253, (uint8_t)175, (uint8_t)77, (uint8_t)232, (uint8_t)18, (uint8_t)186, (uint8_t)49, (uint8_t)52, (uint8_t)66, (uint8_t)46, (uint8_t)67, (uint8_t)54, (uint8_t)107, (uint8_t)29, (uint8_t)60, (uint8_t)202, (uint8_t)34, (uint8_t)206, (uint8_t)159, (uint8_t)249, (uint8_t)29, (uint8_t)4, (uint8_t)57, (uint8_t)29, (uint8_t)105, (uint8_t)35, (uint8_t)212, (uint8_t)188, (uint8_t)13, (uint8_t)5, (uint8_t)200, (uint8_t)65, (uint8_t)162, (uint8_t)128, (uint8_t)19, (uint8_t)59, (uint8_t)197, (uint8_t)230, (uint8_t)75, (uint8_t)240, (uint8_t)78, (uint8_t)174, (uint8_t)36, (uint8_t)79, (uint8_t)73, (uint8_t)16, (uint8_t)222, (uint8_t)91, (uint8_t)156, (uint8_t)234, (uint8_t)188, (uint8_t)45, (uint8_t)212, (uint8_t)138, (uint8_t)111, (uint8_t)133, (uint8_t)70, (uint8_t)120, (uint8_t)13, (uint8_t)160, (uint8_t)248, (uint8_t)82, (uint8_t)127, (uint8_t)84, (uint8_t)188, (uint8_t)231, (uint8_t)75, (uint8_t)111, (uint8_t)84, (uint8_t)174, (uint8_t)139, (uint8_t)144, (uint8_t)145, (uint8_t)129, (uint8_t)25, (uint8_t)14, (uint8_t)244, (uint8_t)80, (uint8_t)75, (uint8_t)60, (uint8_t)143, (uint8_t)53, (uint8_t)67, (uint8_t)158, (uint8_t)36, (uint8_t)245, (uint8_t)129, (uint8_t)154, (uint8_t)224, (uint8_t)48, (uint8_t)243, (uint8_t)92, (uint8_t)183, (uint8_t)184, (uint8_t)105, (uint8_t)35, (uint8_t)185} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -7);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)30972);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)56236);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)15791);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1808589629L);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)19873);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p234_latitude_GET(pack) == (int32_t)587638534);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)7648);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)42);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -82);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)40);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)12132);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -9627);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p234_longitude_GET(pack) == (int32_t)1148647161);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_y_GET(pack) == (float) -1.6855466E38F);
    assert(p241_vibration_z_GET(pack) == (float)2.8700246E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3889224229L);
    assert(p241_time_usec_GET(pack) == (uint64_t)1005456643005828106L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3466782382L);
    assert(p241_vibration_x_GET(pack) == (float) -2.7269454E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)774564120L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_time_usec_TRY(ph) == (uint64_t)8978235579722084327L);
    assert(p242_altitude_GET(pack) == (int32_t)1974913983);
    {
        float exemplary[] =  {-1.9081152E37F, 6.5504897E37F, 8.051274E37F, -1.3140046E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_y_GET(pack) == (float) -1.9842188E38F);
    assert(p242_z_GET(pack) == (float)3.31791E38F);
    assert(p242_x_GET(pack) == (float) -3.2759329E38F);
    assert(p242_y_GET(pack) == (float) -1.115745E38F);
    assert(p242_latitude_GET(pack) == (int32_t) -1456905929);
    assert(p242_longitude_GET(pack) == (int32_t) -1695465939);
    assert(p242_approach_z_GET(pack) == (float) -1.4935021E38F);
    assert(p242_approach_x_GET(pack) == (float) -8.0141444E37F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.278645E38F, 7.362372E37F, -1.4122965E38F, 2.7135035E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_x_GET(pack) == (float)3.1214488E38F);
    assert(p243_approach_y_GET(pack) == (float) -1.1507413E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)6595750134801838430L);
    assert(p243_z_GET(pack) == (float)1.5431788E38F);
    assert(p243_y_GET(pack) == (float)1.695032E38F);
    assert(p243_longitude_GET(pack) == (int32_t)1854400914);
    assert(p243_altitude_GET(pack) == (int32_t)683515314);
    assert(p243_x_GET(pack) == (float)2.462426E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p243_latitude_GET(pack) == (int32_t)291712217);
    assert(p243_approach_z_GET(pack) == (float)8.710898E37F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -1833239760);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)58349);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p246_altitude_GET(pack) == (int32_t)615650320);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED));
    assert(p246_lon_GET(pack) == (int32_t)1132951292);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)46408);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)5978);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2519749146L);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -32193);
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"iarwn";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)62136);
    assert(p246_lat_GET(pack) == (int32_t) -1612996993);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -9.669867E37F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)1.2449657E37F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT);
    assert(p247_id_GET(pack) == (uint32_t)2913219049L);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -2.9761791E38F);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)22618);
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)16, (uint8_t)91, (uint8_t)39, (uint8_t)3, (uint8_t)98, (uint8_t)222, (uint8_t)153, (uint8_t)63, (uint8_t)195, (uint8_t)155, (uint8_t)102, (uint8_t)31, (uint8_t)125, (uint8_t)87, (uint8_t)38, (uint8_t)166, (uint8_t)22, (uint8_t)228, (uint8_t)136, (uint8_t)243, (uint8_t)254, (uint8_t)138, (uint8_t)46, (uint8_t)97, (uint8_t)44, (uint8_t)161, (uint8_t)116, (uint8_t)164, (uint8_t)182, (uint8_t)153, (uint8_t)100, (uint8_t)251, (uint8_t)255, (uint8_t)116, (uint8_t)96, (uint8_t)127, (uint8_t)134, (uint8_t)176, (uint8_t)222, (uint8_t)85, (uint8_t)183, (uint8_t)231, (uint8_t)199, (uint8_t)143, (uint8_t)77, (uint8_t)161, (uint8_t)122, (uint8_t)50, (uint8_t)213, (uint8_t)252, (uint8_t)177, (uint8_t)38, (uint8_t)11, (uint8_t)118, (uint8_t)222, (uint8_t)248, (uint8_t)102, (uint8_t)143, (uint8_t)56, (uint8_t)244, (uint8_t)9, (uint8_t)193, (uint8_t)175, (uint8_t)173, (uint8_t)239, (uint8_t)77, (uint8_t)78, (uint8_t)103, (uint8_t)125, (uint8_t)17, (uint8_t)203, (uint8_t)22, (uint8_t)68, (uint8_t)248, (uint8_t)241, (uint8_t)237, (uint8_t)86, (uint8_t)203, (uint8_t)147, (uint8_t)138, (uint8_t)33, (uint8_t)211, (uint8_t)231, (uint8_t)57, (uint8_t)144, (uint8_t)206, (uint8_t)211, (uint8_t)149, (uint8_t)163, (uint8_t)111, (uint8_t)180, (uint8_t)196, (uint8_t)41, (uint8_t)183, (uint8_t)88, (uint8_t)138, (uint8_t)38, (uint8_t)111, (uint8_t)141, (uint8_t)212, (uint8_t)86, (uint8_t)175, (uint8_t)187, (uint8_t)189, (uint8_t)138, (uint8_t)243, (uint8_t)174, (uint8_t)53, (uint8_t)187, (uint8_t)146, (uint8_t)136, (uint8_t)106, (uint8_t)188, (uint8_t)227, (uint8_t)178, (uint8_t)36, (uint8_t)93, (uint8_t)72, (uint8_t)194, (uint8_t)49, (uint8_t)193, (uint8_t)31, (uint8_t)176, (uint8_t)203, (uint8_t)92, (uint8_t)206, (uint8_t)169, (uint8_t)218, (uint8_t)50, (uint8_t)67, (uint8_t)8, (uint8_t)22, (uint8_t)122, (uint8_t)6, (uint8_t)156, (uint8_t)103, (uint8_t)200, (uint8_t)133, (uint8_t)208, (uint8_t)194, (uint8_t)95, (uint8_t)73, (uint8_t)211, (uint8_t)158, (uint8_t)236, (uint8_t)243, (uint8_t)201, (uint8_t)193, (uint8_t)136, (uint8_t)168, (uint8_t)49, (uint8_t)234, (uint8_t)221, (uint8_t)121, (uint8_t)50, (uint8_t)186, (uint8_t)48, (uint8_t)13, (uint8_t)225, (uint8_t)76, (uint8_t)88, (uint8_t)166, (uint8_t)136, (uint8_t)180, (uint8_t)176, (uint8_t)225, (uint8_t)207, (uint8_t)220, (uint8_t)148, (uint8_t)213, (uint8_t)22, (uint8_t)6, (uint8_t)252, (uint8_t)172, (uint8_t)39, (uint8_t)196, (uint8_t)242, (uint8_t)99, (uint8_t)40, (uint8_t)149, (uint8_t)147, (uint8_t)6, (uint8_t)59, (uint8_t)154, (uint8_t)177, (uint8_t)255, (uint8_t)233, (uint8_t)199, (uint8_t)10, (uint8_t)112, (uint8_t)238, (uint8_t)5, (uint8_t)97, (uint8_t)91, (uint8_t)228, (uint8_t)96, (uint8_t)12, (uint8_t)95, (uint8_t)68, (uint8_t)123, (uint8_t)116, (uint8_t)95, (uint8_t)151, (uint8_t)161, (uint8_t)129, (uint8_t)168, (uint8_t)214, (uint8_t)239, (uint8_t)88, (uint8_t)182, (uint8_t)255, (uint8_t)165, (uint8_t)175, (uint8_t)102, (uint8_t)47, (uint8_t)204, (uint8_t)99, (uint8_t)56, (uint8_t)162, (uint8_t)246, (uint8_t)255, (uint8_t)193, (uint8_t)11, (uint8_t)214, (uint8_t)17, (uint8_t)3, (uint8_t)118, (uint8_t)82, (uint8_t)179, (uint8_t)67, (uint8_t)20, (uint8_t)55, (uint8_t)29, (uint8_t)103, (uint8_t)26, (uint8_t)15, (uint8_t)202, (uint8_t)27, (uint8_t)11, (uint8_t)120, (uint8_t)143, (uint8_t)42, (uint8_t)142, (uint8_t)147, (uint8_t)221, (uint8_t)87, (uint8_t)10, (uint8_t)191} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)58513);
    {
        int8_t exemplary[] =  {(int8_t) -48, (int8_t) -119, (int8_t) -47, (int8_t) -117, (int8_t)63, (int8_t)54, (int8_t) -14, (int8_t) -39, (int8_t)53, (int8_t) -76, (int8_t)64, (int8_t)3, (int8_t) -39, (int8_t) -77, (int8_t)51, (int8_t) -15, (int8_t) -74, (int8_t)123, (int8_t)23, (int8_t) -8, (int8_t)61, (int8_t) -50, (int8_t) -108, (int8_t) -127, (int8_t) -2, (int8_t) -121, (int8_t)59, (int8_t) -106, (int8_t)88, (int8_t) -104, (int8_t) -84, (int8_t)120} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)126);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float)1.2399032E38F);
    assert(p250_z_GET(pack) == (float) -1.5535985E38F);
    assert(p250_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"pizwgi";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_y_GET(pack) == (float)3.4171162E37F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5334953935723241872L);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"rah";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float) -1.834151E38F);
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3568728030L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"zhRjz";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1479923066L);
    assert(p252_value_GET(pack) == (int32_t)1478118449);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 40);
    {
        char16_t * exemplary = u"ixdnitKbpNZlhxjneyVmxBrjpgsrnDjdkpvcnwuo";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 80);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)630124574L);
    assert(p254_value_GET(pack) == (float)8.0777536E37F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)170);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)111944636187843693L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)100);
    {
        uint8_t exemplary[] =  {(uint8_t)85, (uint8_t)201, (uint8_t)226, (uint8_t)152, (uint8_t)167, (uint8_t)46, (uint8_t)146, (uint8_t)136, (uint8_t)168, (uint8_t)136, (uint8_t)21, (uint8_t)105, (uint8_t)47, (uint8_t)185, (uint8_t)0, (uint8_t)197, (uint8_t)225, (uint8_t)182, (uint8_t)14, (uint8_t)138, (uint8_t)168, (uint8_t)210, (uint8_t)97, (uint8_t)237, (uint8_t)58, (uint8_t)19, (uint8_t)194, (uint8_t)241, (uint8_t)2, (uint8_t)73, (uint8_t)160, (uint8_t)111} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1667509278L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)2098271176L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Dy";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_focal_length_GET(pack) == (float)1.6628022E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)36593);
    assert(p259_sensor_size_v_GET(pack) == (float)2.0366379E38F);
    assert(p259_cam_definition_uri_LEN(ph) == 19);
    {
        char16_t * exemplary = u"mcnfxtYvzuyuUcpjtmq";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)16769);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1064213378L);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1868413401L);
    {
        uint8_t exemplary[] =  {(uint8_t)233, (uint8_t)232, (uint8_t)236, (uint8_t)242, (uint8_t)201, (uint8_t)178, (uint8_t)19, (uint8_t)114, (uint8_t)109, (uint8_t)172, (uint8_t)12, (uint8_t)84, (uint8_t)164, (uint8_t)80, (uint8_t)78, (uint8_t)110, (uint8_t)89, (uint8_t)103, (uint8_t)25, (uint8_t)2, (uint8_t)194, (uint8_t)91, (uint8_t)244, (uint8_t)113, (uint8_t)236, (uint8_t)128, (uint8_t)37, (uint8_t)155, (uint8_t)202, (uint8_t)55, (uint8_t)49, (uint8_t)159} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)223, (uint8_t)82, (uint8_t)186, (uint8_t)196, (uint8_t)75, (uint8_t)10, (uint8_t)91, (uint8_t)2, (uint8_t)107, (uint8_t)160, (uint8_t)92, (uint8_t)96, (uint8_t)164, (uint8_t)174, (uint8_t)101, (uint8_t)84, (uint8_t)55, (uint8_t)254, (uint8_t)63, (uint8_t)64, (uint8_t)66, (uint8_t)216, (uint8_t)80, (uint8_t)211, (uint8_t)76, (uint8_t)101, (uint8_t)29, (uint8_t)5, (uint8_t)149, (uint8_t)159, (uint8_t)87, (uint8_t)202} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_h_GET(pack) == (float)4.708464E37F);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)50341);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1187317329L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1638140553L);
    assert(p261_read_speed_GET(pack) == (float) -2.4270088E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p261_total_capacity_GET(pack) == (float) -1.8929161E38F);
    assert(p261_write_speed_GET(pack) == (float)1.0586661E38F);
    assert(p261_used_capacity_GET(pack) == (float)2.7667637E38F);
    assert(p261_available_capacity_GET(pack) == (float)1.2498824E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_available_capacity_GET(pack) == (float)1.1505174E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)4244118059L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p262_image_interval_GET(pack) == (float) -6.954145E36F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2450410825L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_lon_GET(pack) == (int32_t) -806503323);
    assert(p263_relative_alt_GET(pack) == (int32_t) -1198654167);
    assert(p263_lat_GET(pack) == (int32_t) -1345415774);
    assert(p263_alt_GET(pack) == (int32_t)1254304400);
    assert(p263_file_url_LEN(ph) == 87);
    {
        char16_t * exemplary = u"cfuvcrfvwdflmlusivighGycgaKjaByhhvybsxcfrkoonodwkjnglizfjykrbtmussjemuvcsiMjjsuxjLxqrar";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 174);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)57);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1081433290L);
    {
        float exemplary[] =  {-8.0872457E37F, 4.4570255E37F, 1.0402474E38F, 1.1138858E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p263_time_utc_GET(pack) == (uint64_t)799257849446727705L);
    assert(p263_image_index_GET(pack) == (int32_t) -2020076572);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)4852189231299334848L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2006634689L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7492000588839895190L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)3995619622899308902L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2003472730L);
    assert(p265_yaw_GET(pack) == (float)3.7231905E37F);
    assert(p265_pitch_GET(pack) == (float)1.1071837E38F);
    assert(p265_roll_GET(pack) == (float)2.6047674E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)245);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)93, (uint8_t)143, (uint8_t)107, (uint8_t)188, (uint8_t)207, (uint8_t)137, (uint8_t)18, (uint8_t)181, (uint8_t)2, (uint8_t)71, (uint8_t)169, (uint8_t)249, (uint8_t)187, (uint8_t)246, (uint8_t)203, (uint8_t)72, (uint8_t)61, (uint8_t)9, (uint8_t)174, (uint8_t)221, (uint8_t)142, (uint8_t)206, (uint8_t)104, (uint8_t)181, (uint8_t)162, (uint8_t)149, (uint8_t)145, (uint8_t)55, (uint8_t)215, (uint8_t)60, (uint8_t)204, (uint8_t)217, (uint8_t)254, (uint8_t)252, (uint8_t)133, (uint8_t)254, (uint8_t)166, (uint8_t)120, (uint8_t)205, (uint8_t)41, (uint8_t)47, (uint8_t)227, (uint8_t)121, (uint8_t)7, (uint8_t)89, (uint8_t)7, (uint8_t)202, (uint8_t)226, (uint8_t)218, (uint8_t)37, (uint8_t)128, (uint8_t)138, (uint8_t)210, (uint8_t)138, (uint8_t)78, (uint8_t)235, (uint8_t)183, (uint8_t)199, (uint8_t)103, (uint8_t)168, (uint8_t)26, (uint8_t)134, (uint8_t)200, (uint8_t)123, (uint8_t)31, (uint8_t)192, (uint8_t)207, (uint8_t)177, (uint8_t)80, (uint8_t)176, (uint8_t)73, (uint8_t)181, (uint8_t)111, (uint8_t)61, (uint8_t)10, (uint8_t)196, (uint8_t)104, (uint8_t)38, (uint8_t)255, (uint8_t)147, (uint8_t)79, (uint8_t)222, (uint8_t)80, (uint8_t)0, (uint8_t)197, (uint8_t)251, (uint8_t)190, (uint8_t)132, (uint8_t)173, (uint8_t)234, (uint8_t)155, (uint8_t)84, (uint8_t)91, (uint8_t)72, (uint8_t)228, (uint8_t)184, (uint8_t)155, (uint8_t)125, (uint8_t)245, (uint8_t)150, (uint8_t)218, (uint8_t)124, (uint8_t)236, (uint8_t)111, (uint8_t)9, (uint8_t)236, (uint8_t)148, (uint8_t)194, (uint8_t)201, (uint8_t)96, (uint8_t)21, (uint8_t)87, (uint8_t)103, (uint8_t)199, (uint8_t)107, (uint8_t)51, (uint8_t)255, (uint8_t)81, (uint8_t)136, (uint8_t)130, (uint8_t)27, (uint8_t)145, (uint8_t)108, (uint8_t)96, (uint8_t)174, (uint8_t)203, (uint8_t)155, (uint8_t)188, (uint8_t)60, (uint8_t)16, (uint8_t)62, (uint8_t)145, (uint8_t)10, (uint8_t)205, (uint8_t)7, (uint8_t)46, (uint8_t)202, (uint8_t)135, (uint8_t)162, (uint8_t)80, (uint8_t)99, (uint8_t)18, (uint8_t)221, (uint8_t)186, (uint8_t)221, (uint8_t)153, (uint8_t)159, (uint8_t)189, (uint8_t)13, (uint8_t)21, (uint8_t)50, (uint8_t)73, (uint8_t)153, (uint8_t)205, (uint8_t)15, (uint8_t)29, (uint8_t)22, (uint8_t)40, (uint8_t)215, (uint8_t)50, (uint8_t)253, (uint8_t)45, (uint8_t)172, (uint8_t)195, (uint8_t)72, (uint8_t)228, (uint8_t)251, (uint8_t)117, (uint8_t)215, (uint8_t)110, (uint8_t)43, (uint8_t)41, (uint8_t)113, (uint8_t)83, (uint8_t)12, (uint8_t)203, (uint8_t)141, (uint8_t)116, (uint8_t)196, (uint8_t)111, (uint8_t)79, (uint8_t)95, (uint8_t)238, (uint8_t)70, (uint8_t)1, (uint8_t)144, (uint8_t)203, (uint8_t)215, (uint8_t)251, (uint8_t)250, (uint8_t)204, (uint8_t)224, (uint8_t)163, (uint8_t)77, (uint8_t)172, (uint8_t)4, (uint8_t)104, (uint8_t)87, (uint8_t)168, (uint8_t)140, (uint8_t)131, (uint8_t)193, (uint8_t)97, (uint8_t)49, (uint8_t)190, (uint8_t)248, (uint8_t)187, (uint8_t)155, (uint8_t)10, (uint8_t)117, (uint8_t)223, (uint8_t)31, (uint8_t)206, (uint8_t)34, (uint8_t)206, (uint8_t)17, (uint8_t)167, (uint8_t)87, (uint8_t)31, (uint8_t)146, (uint8_t)3, (uint8_t)251, (uint8_t)4, (uint8_t)25, (uint8_t)240, (uint8_t)109, (uint8_t)41, (uint8_t)188, (uint8_t)77, (uint8_t)191, (uint8_t)60, (uint8_t)66, (uint8_t)179, (uint8_t)107, (uint8_t)83, (uint8_t)255, (uint8_t)130, (uint8_t)211, (uint8_t)219, (uint8_t)128, (uint8_t)181, (uint8_t)147, (uint8_t)107, (uint8_t)115, (uint8_t)19, (uint8_t)178, (uint8_t)231, (uint8_t)241} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)34314);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)13, (uint8_t)125, (uint8_t)148, (uint8_t)166, (uint8_t)164, (uint8_t)252, (uint8_t)218, (uint8_t)232, (uint8_t)9, (uint8_t)122, (uint8_t)112, (uint8_t)13, (uint8_t)185, (uint8_t)48, (uint8_t)2, (uint8_t)56, (uint8_t)136, (uint8_t)183, (uint8_t)195, (uint8_t)7, (uint8_t)138, (uint8_t)149, (uint8_t)80, (uint8_t)131, (uint8_t)151, (uint8_t)39, (uint8_t)232, (uint8_t)240, (uint8_t)69, (uint8_t)162, (uint8_t)30, (uint8_t)9, (uint8_t)219, (uint8_t)114, (uint8_t)135, (uint8_t)71, (uint8_t)85, (uint8_t)20, (uint8_t)168, (uint8_t)252, (uint8_t)143, (uint8_t)173, (uint8_t)246, (uint8_t)76, (uint8_t)138, (uint8_t)92, (uint8_t)2, (uint8_t)190, (uint8_t)41, (uint8_t)134, (uint8_t)156, (uint8_t)91, (uint8_t)160, (uint8_t)220, (uint8_t)58, (uint8_t)82, (uint8_t)145, (uint8_t)216, (uint8_t)224, (uint8_t)125, (uint8_t)191, (uint8_t)109, (uint8_t)166, (uint8_t)174, (uint8_t)231, (uint8_t)29, (uint8_t)238, (uint8_t)240, (uint8_t)37, (uint8_t)99, (uint8_t)38, (uint8_t)178, (uint8_t)153, (uint8_t)121, (uint8_t)173, (uint8_t)188, (uint8_t)7, (uint8_t)170, (uint8_t)149, (uint8_t)52, (uint8_t)137, (uint8_t)249, (uint8_t)240, (uint8_t)180, (uint8_t)214, (uint8_t)106, (uint8_t)199, (uint8_t)91, (uint8_t)209, (uint8_t)111, (uint8_t)217, (uint8_t)7, (uint8_t)28, (uint8_t)69, (uint8_t)211, (uint8_t)78, (uint8_t)210, (uint8_t)58, (uint8_t)31, (uint8_t)245, (uint8_t)162, (uint8_t)31, (uint8_t)211, (uint8_t)209, (uint8_t)238, (uint8_t)115, (uint8_t)73, (uint8_t)8, (uint8_t)154, (uint8_t)151, (uint8_t)83, (uint8_t)68, (uint8_t)110, (uint8_t)34, (uint8_t)192, (uint8_t)71, (uint8_t)139, (uint8_t)166, (uint8_t)98, (uint8_t)243, (uint8_t)138, (uint8_t)56, (uint8_t)148, (uint8_t)161, (uint8_t)97, (uint8_t)64, (uint8_t)97, (uint8_t)92, (uint8_t)40, (uint8_t)151, (uint8_t)104, (uint8_t)165, (uint8_t)100, (uint8_t)244, (uint8_t)17, (uint8_t)207, (uint8_t)23, (uint8_t)32, (uint8_t)97, (uint8_t)8, (uint8_t)54, (uint8_t)243, (uint8_t)34, (uint8_t)248, (uint8_t)76, (uint8_t)223, (uint8_t)191, (uint8_t)95, (uint8_t)23, (uint8_t)67, (uint8_t)230, (uint8_t)33, (uint8_t)67, (uint8_t)169, (uint8_t)106, (uint8_t)137, (uint8_t)114, (uint8_t)86, (uint8_t)24, (uint8_t)48, (uint8_t)74, (uint8_t)201, (uint8_t)76, (uint8_t)98, (uint8_t)52, (uint8_t)29, (uint8_t)250, (uint8_t)102, (uint8_t)67, (uint8_t)21, (uint8_t)66, (uint8_t)65, (uint8_t)219, (uint8_t)96, (uint8_t)227, (uint8_t)152, (uint8_t)21, (uint8_t)0, (uint8_t)11, (uint8_t)121, (uint8_t)43, (uint8_t)95, (uint8_t)149, (uint8_t)233, (uint8_t)35, (uint8_t)134, (uint8_t)139, (uint8_t)123, (uint8_t)44, (uint8_t)28, (uint8_t)137, (uint8_t)95, (uint8_t)116, (uint8_t)250, (uint8_t)75, (uint8_t)159, (uint8_t)241, (uint8_t)69, (uint8_t)67, (uint8_t)198, (uint8_t)225, (uint8_t)235, (uint8_t)60, (uint8_t)77, (uint8_t)50, (uint8_t)124, (uint8_t)90, (uint8_t)209, (uint8_t)61, (uint8_t)218, (uint8_t)24, (uint8_t)31, (uint8_t)125, (uint8_t)117, (uint8_t)27, (uint8_t)162, (uint8_t)128, (uint8_t)96, (uint8_t)81, (uint8_t)203, (uint8_t)57, (uint8_t)86, (uint8_t)122, (uint8_t)0, (uint8_t)106, (uint8_t)30, (uint8_t)10, (uint8_t)248, (uint8_t)66, (uint8_t)89, (uint8_t)114, (uint8_t)174, (uint8_t)102, (uint8_t)77, (uint8_t)229, (uint8_t)6, (uint8_t)4, (uint8_t)234, (uint8_t)137, (uint8_t)87, (uint8_t)202, (uint8_t)103, (uint8_t)101, (uint8_t)46, (uint8_t)208, (uint8_t)194, (uint8_t)169, (uint8_t)189, (uint8_t)164} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)11590);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)62928);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)98);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_uri_LEN(ph) == 152);
    {
        char16_t * exemplary = u"xhqzmBnQZrvrgypzyPlmMusQuKuhipljlklrqzcziujlsdntuktvxntihfzanjcuhgxwccbcukrlqFtXllnykjWcvzjgwjigxEoovtrlJwzeMiuvjFxGgwvgxtpjtrfwsbjjNxfvlftlaufwKsiwxdrX";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 304);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)25987);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)15619);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p269_bitrate_GET(pack) == (uint32_t)2002045447L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)46077);
    assert(p269_framerate_GET(pack) == (float)1.6705992E38F);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)17957);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)49777);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)29389);
    assert(p270_uri_LEN(ph) == 136);
    {
        char16_t * exemplary = u"wnwbkhgxniqwxtkZxqxtuDsxydgxzmbjGgygnahgmdjqpsfhjtzehtQzoInvcrivpstlvqgebpmryrsdXncvsbarfqpfvglxddwghqpuoVrbxcwQkfyywvatqmbxyXemcHdrdjWy";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 272);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p270_framerate_GET(pack) == (float) -1.5060466E38F);
    assert(p270_bitrate_GET(pack) == (uint32_t)3045996843L);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 64);
    {
        char16_t * exemplary = u"nqzjpyFYeOtQwuyaifrOdjmhPxyvwMJiTiXeyynljzaCceOgIdpjKdasqyemecrx";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 19);
    {
        char16_t * exemplary = u"gmwvntbPyjokiRbqunr";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)233, (uint8_t)166, (uint8_t)77, (uint8_t)127, (uint8_t)230, (uint8_t)225, (uint8_t)169, (uint8_t)177} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)61060);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)50852);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)28235);
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)95, (uint8_t)47, (uint8_t)213, (uint8_t)19, (uint8_t)240, (uint8_t)79, (uint8_t)239} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)15797);
    assert(p310_time_usec_GET(pack) == (uint64_t)1249056681501338864L);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)52483073L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p311_time_usec_GET(pack) == (uint64_t)2579408316857718861L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p311_name_LEN(ph) == 12);
    {
        char16_t * exemplary = u"ujbifrhEimhe";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2689980859L);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2502447664L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)58);
    {
        uint8_t exemplary[] =  {(uint8_t)41, (uint8_t)184, (uint8_t)192, (uint8_t)32, (uint8_t)93, (uint8_t)110, (uint8_t)77, (uint8_t)92, (uint8_t)19, (uint8_t)195, (uint8_t)210, (uint8_t)209, (uint8_t)23, (uint8_t)88, (uint8_t)153, (uint8_t)201} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -21318);
    assert(p320_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"CazPbpzjbblsaw";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)236);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)151);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 29);
    {
        char16_t * exemplary = u"ukfqdvohcwSzgdqwqjoDmtkltigxd";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 58);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)25035);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)17048);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p322_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"hodn";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p323_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Cs";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p323_param_value_LEN(ph) == 14);
    {
        char16_t * exemplary = u"zqoeoutfctitol";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"hevbgnbysXaAls";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 125);
    {
        char16_t * exemplary = u"badfmsscaelmvfrpzIwdthRertpfwgqnstukadTzxayKbdcnjauhnbwikqovqwaoippnccMlzvsqtarjezdnxejbfthzocbepdweHzkfslxfblmwnxmavfPscdkRd";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 250);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_time_usec_GET(pack) == (uint64_t)4988847116169182049L);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)13594);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)50564);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)16);
    {
        uint16_t exemplary[] =  {(uint16_t)62940, (uint16_t)18996, (uint16_t)50926, (uint16_t)24201, (uint16_t)36839, (uint16_t)55368, (uint16_t)9105, (uint16_t)26552, (uint16_t)59620, (uint16_t)13011, (uint16_t)10202, (uint16_t)40069, (uint16_t)37951, (uint16_t)11260, (uint16_t)44854, (uint16_t)36107, (uint16_t)58298, (uint16_t)34312, (uint16_t)7631, (uint16_t)7040, (uint16_t)34000, (uint16_t)26094, (uint16_t)22093, (uint16_t)20172, (uint16_t)33375, (uint16_t)4511, (uint16_t)2886, (uint16_t)15677, (uint16_t)16595, (uint16_t)47338, (uint16_t)57578, (uint16_t)30970, (uint16_t)36337, (uint16_t)33966, (uint16_t)14399, (uint16_t)23100, (uint16_t)57701, (uint16_t)50056, (uint16_t)15357, (uint16_t)31833, (uint16_t)18077, (uint16_t)57105, (uint16_t)60735, (uint16_t)59711, (uint16_t)35235, (uint16_t)48014, (uint16_t)58386, (uint16_t)1664, (uint16_t)7651, (uint16_t)10619, (uint16_t)54096, (uint16_t)35177, (uint16_t)60871, (uint16_t)13344, (uint16_t)42868, (uint16_t)52215, (uint16_t)3177, (uint16_t)27621, (uint16_t)3906, (uint16_t)21040, (uint16_t)57337, (uint16_t)59895, (uint16_t)10615, (uint16_t)36507, (uint16_t)39817, (uint16_t)31648, (uint16_t)19072, (uint16_t)6437, (uint16_t)15040, (uint16_t)35938, (uint16_t)49728, (uint16_t)675} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    assert(p10001_callsign_LEN(ph) == 3);
    {
        char16_t * exemplary = u"zqz";
        char16_t * sample = p10001_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p10001_aircraftSize_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M);
    assert(p10001_gpsOffsetLon_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
    assert(p10001_rfSelect_GET(pack) == e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
    assert(p10001_emitterType_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
    assert(p10001_ICAO_GET(pack) == (uint32_t)2633229907L);
    assert(p10001_stallSpeed_GET(pack) == (uint16_t)(uint16_t)38609);
    assert(p10001_gpsOffsetLat_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    assert(p10002_emergencyStatus_GET(pack) == e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_EMERGENCY);
    assert(p10002_accuracyVel_GET(pack) == (uint16_t)(uint16_t)13783);
    assert(p10002_VelEW_GET(pack) == (int16_t)(int16_t)8272);
    assert(p10002_velNS_GET(pack) == (int16_t)(int16_t) -12356);
    assert(p10002_gpsLon_GET(pack) == (int32_t)205261065);
    assert(p10002_state_GET(pack) == (e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND));
    assert(p10002_baroAltMSL_GET(pack) == (int32_t) -235367561);
    assert(p10002_gpsFix_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1);
    assert(p10002_gpsLat_GET(pack) == (int32_t)1253127399);
    assert(p10002_numSats_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p10002_accuracyVert_GET(pack) == (uint16_t)(uint16_t)4618);
    assert(p10002_gpsAlt_GET(pack) == (int32_t)305936887);
    assert(p10002_squawk_GET(pack) == (uint16_t)(uint16_t)38837);
    assert(p10002_accuracyHor_GET(pack) == (uint32_t)2618932642L);
    assert(p10002_utcTime_GET(pack) == (uint32_t)4100021455L);
    assert(p10002_velVert_GET(pack) == (int16_t)(int16_t)13642);
};


void c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    assert(p10003_rfHealth_GET(pack) == e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_RX);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    assert(p11000_bus_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p11000_regstart_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p11000_busname_LEN(ph) == 35);
    {
        char16_t * exemplary = u"bnaqBvUzqworpcyvinqkxakmjcpeyavrqob";
        char16_t * sample = p11000_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11000_target_system_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p11000_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11000_count_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p11000_address_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p11000_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p11000_request_id_GET(pack) == (uint32_t)2077538683L);
};


void c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)47, (uint8_t)44, (uint8_t)230, (uint8_t)19, (uint8_t)24, (uint8_t)145, (uint8_t)228, (uint8_t)140, (uint8_t)67, (uint8_t)54, (uint8_t)169, (uint8_t)43, (uint8_t)244, (uint8_t)188, (uint8_t)160, (uint8_t)199, (uint8_t)164, (uint8_t)34, (uint8_t)69, (uint8_t)101, (uint8_t)184, (uint8_t)122, (uint8_t)36, (uint8_t)97, (uint8_t)195, (uint8_t)40, (uint8_t)136, (uint8_t)199, (uint8_t)39, (uint8_t)132, (uint8_t)110, (uint8_t)145, (uint8_t)177, (uint8_t)21, (uint8_t)135, (uint8_t)49, (uint8_t)98, (uint8_t)11, (uint8_t)131, (uint8_t)42, (uint8_t)44, (uint8_t)121, (uint8_t)62, (uint8_t)157, (uint8_t)130, (uint8_t)136, (uint8_t)152, (uint8_t)128, (uint8_t)67, (uint8_t)179, (uint8_t)242, (uint8_t)162, (uint8_t)157, (uint8_t)170, (uint8_t)45, (uint8_t)97, (uint8_t)131, (uint8_t)175, (uint8_t)219, (uint8_t)121, (uint8_t)7, (uint8_t)206, (uint8_t)176, (uint8_t)254, (uint8_t)228, (uint8_t)211, (uint8_t)37, (uint8_t)209, (uint8_t)75, (uint8_t)122, (uint8_t)234, (uint8_t)80, (uint8_t)204, (uint8_t)218, (uint8_t)97, (uint8_t)16, (uint8_t)248, (uint8_t)162, (uint8_t)186, (uint8_t)124, (uint8_t)115, (uint8_t)51, (uint8_t)97, (uint8_t)45, (uint8_t)17, (uint8_t)167, (uint8_t)6, (uint8_t)213, (uint8_t)184, (uint8_t)47, (uint8_t)77, (uint8_t)244, (uint8_t)126, (uint8_t)137, (uint8_t)133, (uint8_t)99, (uint8_t)193, (uint8_t)140, (uint8_t)141, (uint8_t)45, (uint8_t)62, (uint8_t)20, (uint8_t)24, (uint8_t)174, (uint8_t)32, (uint8_t)117, (uint8_t)48, (uint8_t)155, (uint8_t)101, (uint8_t)137, (uint8_t)148, (uint8_t)9, (uint8_t)187, (uint8_t)78, (uint8_t)15, (uint8_t)94, (uint8_t)207, (uint8_t)119, (uint8_t)146, (uint8_t)185, (uint8_t)64, (uint8_t)62, (uint8_t)164, (uint8_t)146, (uint8_t)219, (uint8_t)163, (uint8_t)69, (uint8_t)27} ;
        uint8_t*  sample = p11001_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11001_count_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p11001_result_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p11001_regstart_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p11001_request_id_GET(pack) == (uint32_t)1126125765L);
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    assert(p11002_regstart_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p11002_request_id_GET(pack) == (uint32_t)3636707651L);
    assert(p11002_busname_LEN(ph) == 20);
    {
        char16_t * exemplary = u"jdfdrHbtouwuRfJunjpt";
        char16_t * sample = p11002_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_address_GET(pack) == (uint8_t)(uint8_t)167);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)107, (uint8_t)181, (uint8_t)182, (uint8_t)160, (uint8_t)102, (uint8_t)35, (uint8_t)135, (uint8_t)47, (uint8_t)221, (uint8_t)103, (uint8_t)17, (uint8_t)232, (uint8_t)191, (uint8_t)131, (uint8_t)17, (uint8_t)224, (uint8_t)104, (uint8_t)46, (uint8_t)174, (uint8_t)151, (uint8_t)23, (uint8_t)214, (uint8_t)226, (uint8_t)29, (uint8_t)18, (uint8_t)97, (uint8_t)248, (uint8_t)11, (uint8_t)224, (uint8_t)125, (uint8_t)3, (uint8_t)176, (uint8_t)36, (uint8_t)133, (uint8_t)236, (uint8_t)252, (uint8_t)134, (uint8_t)163, (uint8_t)149, (uint8_t)58, (uint8_t)94, (uint8_t)171, (uint8_t)144, (uint8_t)236, (uint8_t)19, (uint8_t)40, (uint8_t)133, (uint8_t)56, (uint8_t)209, (uint8_t)67, (uint8_t)54, (uint8_t)250, (uint8_t)168, (uint8_t)222, (uint8_t)80, (uint8_t)206, (uint8_t)36, (uint8_t)192, (uint8_t)194, (uint8_t)173, (uint8_t)156, (uint8_t)50, (uint8_t)80, (uint8_t)233, (uint8_t)100, (uint8_t)77, (uint8_t)98, (uint8_t)236, (uint8_t)99, (uint8_t)69, (uint8_t)36, (uint8_t)59, (uint8_t)218, (uint8_t)152, (uint8_t)120, (uint8_t)4, (uint8_t)165, (uint8_t)58, (uint8_t)194, (uint8_t)189, (uint8_t)182, (uint8_t)121, (uint8_t)236, (uint8_t)174, (uint8_t)47, (uint8_t)106, (uint8_t)174, (uint8_t)93, (uint8_t)136, (uint8_t)29, (uint8_t)42, (uint8_t)10, (uint8_t)117, (uint8_t)217, (uint8_t)129, (uint8_t)143, (uint8_t)203, (uint8_t)230, (uint8_t)114, (uint8_t)126, (uint8_t)232, (uint8_t)127, (uint8_t)22, (uint8_t)159, (uint8_t)176, (uint8_t)78, (uint8_t)44, (uint8_t)42, (uint8_t)87, (uint8_t)207, (uint8_t)38, (uint8_t)206, (uint8_t)86, (uint8_t)116, (uint8_t)179, (uint8_t)77, (uint8_t)229, (uint8_t)165, (uint8_t)245, (uint8_t)165, (uint8_t)44, (uint8_t)140, (uint8_t)159, (uint8_t)54, (uint8_t)206, (uint8_t)201, (uint8_t)179} ;
        uint8_t*  sample = p11002_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_target_component_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p11002_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p11002_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11002_bus_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p11002_count_GET(pack) == (uint8_t)(uint8_t)206);
};


void c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    assert(p11003_request_id_GET(pack) == (uint32_t)935165781L);
    assert(p11003_result_GET(pack) == (uint8_t)(uint8_t)203);
};


void c_CommunicationChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    assert(p11010_achieved_GET(pack) == (float)3.0783045E38F);
    assert(p11010_omega_GET(pack) == (float)2.4811135E38F);
    assert(p11010_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_PITCH);
    assert(p11010_f_dot_GET(pack) == (float)1.4240276E38F);
    assert(p11010_u_GET(pack) == (float) -1.3658877E38F);
    assert(p11010_omega_dot_GET(pack) == (float)3.020351E38F);
    assert(p11010_theta_dot_GET(pack) == (float) -8.170701E37F);
    assert(p11010_theta_GET(pack) == (float) -1.92436E38F);
    assert(p11010_error_GET(pack) == (float) -1.2493096E38F);
    assert(p11010_sigma_dot_GET(pack) == (float)1.3209549E38F);
    assert(p11010_f_GET(pack) == (float)1.9595826E38F);
    assert(p11010_desired_GET(pack) == (float) -3.174889E38F);
    assert(p11010_sigma_GET(pack) == (float)2.8186976E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    assert(p11011_time_delta_usec_GET(pack) == (uint64_t)2232829211584213443L);
    assert(p11011_confidence_GET(pack) == (float) -3.2751358E38F);
    {
        float exemplary[] =  {3.0978067E38F, -1.3817194E38F, -2.8077698E37F} ;
        float*  sample = p11011_position_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.0976641E38F, -2.9963323E38F, 3.291935E38F} ;
        float*  sample = p11011_angle_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11011_time_usec_GET(pack) == (uint64_t)7588275623408616166L);
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_GENERIC, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)1507917181L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)29702, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)56053, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL), PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)58618, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)22957, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)41243, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)59800, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)5397, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -4938, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)49310, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)34, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)3235677747L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)9113139124641270543L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float) -1.3551482E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)59169, PH.base.pack) ;
        p3_afy_SET((float) -2.1962967E38F, PH.base.pack) ;
        p3_afx_SET((float) -1.8687326E38F, PH.base.pack) ;
        p3_y_SET((float) -4.3472445E37F, PH.base.pack) ;
        p3_yaw_SET((float) -3.6022302E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)14708040L, PH.base.pack) ;
        p3_vx_SET((float) -2.632605E38F, PH.base.pack) ;
        p3_x_SET((float) -2.6014653E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p3_afz_SET((float) -2.3980548E38F, PH.base.pack) ;
        p3_z_SET((float) -3.2615399E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.8300678E38F, PH.base.pack) ;
        p3_vy_SET((float) -8.177676E37F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p4_seq_SET((uint32_t)3152826195L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)3880396579108295343L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"ojgdkqjvudnqxvzusptuYpxh";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"demprzujwspbngqirjbdqpgbpnasruu";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)1490703464L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t) -26042, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        {
            char16_t* param_id = u"bfcBvlxxgsynkbF";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"mvwbdfz";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_index_SET((uint16_t)(uint16_t)31342, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p22_param_value_SET((float) -3.2911002E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)54714, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        {
            char16_t* param_id = u"xefozgvu";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_value_SET((float) -2.1598708E38F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_vel_acc_SET((uint32_t)1012811980L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)55627, PH.base.pack) ;
        p24_lon_SET((int32_t)1409349401, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)812081883450896751L, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)32838, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)41018, PH.base.pack) ;
        p24_alt_SET((int32_t) -990944789, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -184536551, &PH) ;
        p24_h_acc_SET((uint32_t)2325586932L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1491266432L, &PH) ;
        p24_lat_SET((int32_t)563736395, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)45854, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)2425708132L, &PH) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)70, (uint8_t)219, (uint8_t)102, (uint8_t)10, (uint8_t)247, (uint8_t)4, (uint8_t)200, (uint8_t)215, (uint8_t)91, (uint8_t)13, (uint8_t)14, (uint8_t)57, (uint8_t)191, (uint8_t)58, (uint8_t)116, (uint8_t)73, (uint8_t)217, (uint8_t)32, (uint8_t)173, (uint8_t)211};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)180, (uint8_t)139, (uint8_t)77, (uint8_t)197, (uint8_t)181, (uint8_t)145, (uint8_t)55, (uint8_t)6, (uint8_t)250, (uint8_t)54, (uint8_t)164, (uint8_t)249, (uint8_t)47, (uint8_t)41, (uint8_t)65, (uint8_t)55, (uint8_t)70, (uint8_t)231, (uint8_t)151, (uint8_t)30};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        {
            uint8_t satellite_elevation[] =  {(uint8_t)245, (uint8_t)35, (uint8_t)28, (uint8_t)20, (uint8_t)204, (uint8_t)158, (uint8_t)95, (uint8_t)44, (uint8_t)202, (uint8_t)191, (uint8_t)81, (uint8_t)69, (uint8_t)146, (uint8_t)71, (uint8_t)128, (uint8_t)152, (uint8_t)63, (uint8_t)58, (uint8_t)252, (uint8_t)38};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)66, (uint8_t)7, (uint8_t)82, (uint8_t)36, (uint8_t)9, (uint8_t)133, (uint8_t)52, (uint8_t)206, (uint8_t)217, (uint8_t)175, (uint8_t)203, (uint8_t)180, (uint8_t)121, (uint8_t)65, (uint8_t)2, (uint8_t)62, (uint8_t)215, (uint8_t)246, (uint8_t)140, (uint8_t)230};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)217, (uint8_t)242, (uint8_t)28, (uint8_t)9, (uint8_t)226, (uint8_t)36, (uint8_t)232, (uint8_t)228, (uint8_t)5, (uint8_t)125, (uint8_t)53, (uint8_t)164, (uint8_t)11, (uint8_t)165, (uint8_t)239, (uint8_t)38, (uint8_t)202, (uint8_t)48, (uint8_t)55, (uint8_t)114};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_xmag_SET((int16_t)(int16_t) -11962, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -27678, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -31972, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)7573, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)14972, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)3532, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)16340, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)17871, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)23578, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)2670636110L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_time_usec_SET((uint64_t)5336946355204920090L, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)12291, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)4814, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)27653, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -28460, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -11358, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)3839, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)22472, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)17432, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -4189, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)6116483033102451736L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -4931, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)29670, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -11700, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -11624, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)10156, PH.base.pack) ;
        p29_press_abs_SET((float)1.1376476E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)4169372646L, PH.base.pack) ;
        p29_press_diff_SET((float)9.754039E35F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float) -2.2428131E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)335549377L, PH.base.pack) ;
        p30_rollspeed_SET((float)1.6218839E38F, PH.base.pack) ;
        p30_roll_SET((float)1.3467523E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)3.4802348E37F, PH.base.pack) ;
        p30_pitchspeed_SET((float)1.366672E38F, PH.base.pack) ;
        p30_pitch_SET((float) -2.699166E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -1.5157329E38F, PH.base.pack) ;
        p31_q1_SET((float)1.8940521E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1252572554L, PH.base.pack) ;
        p31_pitchspeed_SET((float) -1.2037808E38F, PH.base.pack) ;
        p31_rollspeed_SET((float)2.8015342E38F, PH.base.pack) ;
        p31_q3_SET((float)1.8936846E38F, PH.base.pack) ;
        p31_q4_SET((float) -2.902244E38F, PH.base.pack) ;
        p31_q2_SET((float)1.977852E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vy_SET((float) -4.103674E36F, PH.base.pack) ;
        p32_z_SET((float)3.2366209E38F, PH.base.pack) ;
        p32_vz_SET((float) -2.0809572E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)2119784049L, PH.base.pack) ;
        p32_x_SET((float)2.0022598E38F, PH.base.pack) ;
        p32_y_SET((float)3.3296914E38F, PH.base.pack) ;
        p32_vx_SET((float) -7.539541E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lon_SET((int32_t) -1433095348, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)9541, PH.base.pack) ;
        p33_alt_SET((int32_t) -500480532, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)23942, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)2125111211, PH.base.pack) ;
        p33_lat_SET((int32_t) -490945291, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)3104226334L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)37133, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -24038, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_rssi_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -11687, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -13506, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -15591, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -20634, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -22260, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)12547, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)13254, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2781093093L, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -26058, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3565101656L, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)65410, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)2792, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)7920, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)47550, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)62225, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)58728, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)30329, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)65376, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_port_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)55555, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)32500, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)57075, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)6058, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)60202, &PH) ;
        p36_time_usec_SET((uint32_t)3007888727L, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)36590, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)50194, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)4780, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)82, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)7688, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)52962, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)37572, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)55580, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)5260, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)59001, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)15114, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)29644, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -19629, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t)10850, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)540, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_command_SET(e_MAV_CMD_MAV_CMD_START_RX_PAIR, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p39_param4_SET((float) -1.7461544E38F, PH.base.pack) ;
        p39_x_SET((float)2.9298906E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p39_y_SET((float)3.3412876E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p39_param2_SET((float)4.771834E37F, PH.base.pack) ;
        p39_param3_SET((float) -2.7440234E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)19781, PH.base.pack) ;
        p39_z_SET((float)1.4202114E38F, PH.base.pack) ;
        p39_param1_SET((float)2.5913364E37F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)17087, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)50857, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)45527, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)44085, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)51891, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM1, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p48_altitude_SET((int32_t) -470351972, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)2829818730738694054L, &PH) ;
        p48_latitude_SET((int32_t) -915142343, PH.base.pack) ;
        p48_longitude_SET((int32_t)1128570699, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t)2119238268, PH.base.pack) ;
        p49_longitude_SET((int32_t)878439586, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)305731949494471015L, &PH) ;
        p49_altitude_SET((int32_t) -756052815, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value0_SET((float)3.1534168E38F, PH.base.pack) ;
        p50_scale_SET((float)2.497076E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.3263042E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"ebmfwJixgzvQi";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -19495, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p50_param_value_min_SET((float)3.1828877E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)37188, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1x_SET((float)5.979763E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p54_p1y_SET((float)1.762506E38F, PH.base.pack) ;
        p54_p2x_SET((float)1.4746301E38F, PH.base.pack) ;
        p54_p2y_SET((float) -3.2924186E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p54_p1z_SET((float)1.618148E38F, PH.base.pack) ;
        p54_p2z_SET((float) -2.6795347E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1z_SET((float) -1.5931289E38F, PH.base.pack) ;
        p55_p2y_SET((float) -7.585071E37F, PH.base.pack) ;
        p55_p1y_SET((float)2.7264589E38F, PH.base.pack) ;
        p55_p2z_SET((float)2.5445569E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p55_p1x_SET((float) -2.451315E38F, PH.base.pack) ;
        p55_p2x_SET((float) -2.7728822E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)3434135610052113093L, PH.base.pack) ;
        {
            float covariance[] =  {-2.3120016E38F, -1.3924764E38F, 7.203817E37F, -1.7756672E38F, 1.0527848E38F, 1.805208E38F, 2.7960529E38F, -1.5685595E38F, -2.3839447E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float) -3.3957637E37F, PH.base.pack) ;
        p61_rollspeed_SET((float) -2.3735532E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float)1.3826753E38F, PH.base.pack) ;
        {
            float q[] =  {2.8531728E36F, 1.6117041E38F, -2.6416076E38F, -6.8527904E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_roll_SET((float)2.7056244E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -9732, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)22947, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)2074, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.7191612E38F, PH.base.pack) ;
        p62_aspd_error_SET((float)1.9368425E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -3.1653018E38F, PH.base.pack) ;
        p62_alt_error_SET((float)3.2057809E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_alt_SET((int32_t)248815890, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)4826881826863284526L, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p63_lat_SET((int32_t) -403303500, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -973687288, PH.base.pack) ;
        p63_vy_SET((float) -2.8229015E38F, PH.base.pack) ;
        p63_lon_SET((int32_t) -868806138, PH.base.pack) ;
        p63_vz_SET((float) -1.652108E38F, PH.base.pack) ;
        p63_vx_SET((float) -1.6030246E38F, PH.base.pack) ;
        {
            float covariance[] =  {-3.239034E38F, -1.2033711E38F, 2.5007235E38F, -2.5752476E38F, -2.3235232E38F, -1.6642371E38F, 1.4975653E38F, -3.342737E38F, -2.056897E38F, 6.7402965E37F, 2.5237786E38F, 5.44866E37F, 3.0737951E38F, -2.3864056E38F, -1.2236092E38F, -1.6901441E38F, 3.2037553E38F, 1.4200998E37F, -4.9988214E37F, 1.2144409E38F, 2.2459767E38F, -1.0323024E38F, 2.7316197E38F, -4.066143E37F, -3.177473E38F, 1.0734106E38F, -7.1379443E37F, 2.2348323E38F, -1.2298958E38F, -1.4754168E38F, 1.8674923E38F, -1.8441298E38F, -4.5068604E37F, -1.1949669E38F, 1.993312E38F, -1.5676032E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_y_SET((float) -2.5287611E38F, PH.base.pack) ;
        p64_x_SET((float)2.727631E38F, PH.base.pack) ;
        p64_vy_SET((float)4.500147E37F, PH.base.pack) ;
        p64_ay_SET((float)2.7114265E37F, PH.base.pack) ;
        p64_az_SET((float)1.2321267E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)6652723940646630943L, PH.base.pack) ;
        p64_ax_SET((float)5.4818137E36F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p64_vx_SET((float) -1.1269235E38F, PH.base.pack) ;
        p64_vz_SET((float)1.3942385E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.4572496E38F, -1.3927415E37F, -2.0380216E37F, -3.4929633E37F, 3.196413E37F, -2.5114801E38F, 2.6954083E38F, 6.1510367E37F, -2.7663475E37F, -6.205452E37F, -1.7751019E38F, 2.2543432E38F, 2.784362E38F, -5.8358045E37F, 3.3321543E38F, -2.2846433E38F, -2.0583723E38F, 1.62246E38F, 5.149354E37F, 2.8352184E38F, -5.2137775E37F, 5.280439E37F, 1.6853268E38F, 3.2967728E38F, 8.469143E37F, -1.046969E38F, 8.644893E37F, 2.6563065E38F, -2.6433764E38F, 2.352082E38F, -2.0665003E38F, -3.346764E38F, -1.2072818E38F, -1.877937E38F, -4.596367E37F, -1.8761286E38F, 3.190952E38F, 7.2946107E37F, 1.3592209E37F, 2.5025461E38F, -6.6362594E37F, -2.3309074E38F, 1.301947E38F, -1.288473E38F, -1.7244368E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_z_SET((float)2.6468491E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan4_raw_SET((uint16_t)(uint16_t)525, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)36120, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)23145, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)59948, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)13080, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)28249, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)14791, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)36356, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)47323, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)64427, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)33434, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)49633, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)6976, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)65416, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)19564, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)25898, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)31094, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2188519992L, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)61066, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)55704, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)33592, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)8489, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)28536, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)23266, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)3651, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)21184, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan6_raw_SET((uint16_t)(uint16_t)24823, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)18453, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)16043, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)31568, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)58749, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)8710, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)48000, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)18519, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p73_param1_SET((float) -1.6222878E38F, PH.base.pack) ;
        p73_x_SET((int32_t) -709876902, PH.base.pack) ;
        p73_z_SET((float) -3.23091E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)18223, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p73_y_SET((int32_t)140690130, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, PH.base.pack) ;
        p73_param2_SET((float)1.0296012E38F, PH.base.pack) ;
        p73_param3_SET((float)1.260575E38F, PH.base.pack) ;
        p73_param4_SET((float) -2.4459255E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_heading_SET((int16_t)(int16_t) -22389, PH.base.pack) ;
        p74_groundspeed_SET((float) -3.1275636E38F, PH.base.pack) ;
        p74_alt_SET((float)1.473918E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)44875, PH.base.pack) ;
        p74_airspeed_SET((float)9.358248E36F, PH.base.pack) ;
        p74_climb_SET((float) -2.72466E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param1_SET((float) -1.516049E38F, PH.base.pack) ;
        p75_z_SET((float) -1.2734928E37F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p75_x_SET((int32_t)487020625, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p75_y_SET((int32_t) -1252689007, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_LOGGING_STOP, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p75_param2_SET((float)1.859429E38F, PH.base.pack) ;
        p75_param4_SET((float)1.3499299E38F, PH.base.pack) ;
        p75_param3_SET((float)4.1519188E37F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p76_param2_SET((float)2.3891985E38F, PH.base.pack) ;
        p76_param7_SET((float)5.629998E37F, PH.base.pack) ;
        p76_param3_SET((float)2.2488908E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p76_param6_SET((float) -2.3954525E37F, PH.base.pack) ;
        p76_param1_SET((float)2.6181997E37F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p76_param4_SET((float)3.0607724E38F, PH.base.pack) ;
        p76_param5_SET((float) -2.101496E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)0, &PH) ;
        p77_result_param2_SET((int32_t) -1071525431, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)138, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)160, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p81_roll_SET((float)1.0686201E37F, PH.base.pack) ;
        p81_yaw_SET((float) -1.0723589E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2375137648L, PH.base.pack) ;
        p81_pitch_SET((float)2.3611834E38F, PH.base.pack) ;
        p81_thrust_SET((float)7.806374E37F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)9.355765E37F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)1865019388L, PH.base.pack) ;
        p82_body_roll_rate_SET((float)2.2191887E38F, PH.base.pack) ;
        p82_thrust_SET((float) -1.6834304E38F, PH.base.pack) ;
        {
            float q[] =  {-2.110495E38F, 1.8283688E38F, -1.5858954E38F, -2.8520218E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_type_mask_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)1.5138876E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float) -7.1474704E37F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)6.399244E37F, PH.base.pack) ;
        {
            float q[] =  {-1.6234445E38F, 2.5620551E38F, 9.460931E37F, -2.8050946E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_thrust_SET((float)2.52084E37F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)7.0352747E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)1388107930L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afz_SET((float)2.7866063E38F, PH.base.pack) ;
        p84_yaw_SET((float)1.7210981E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -2.7698837E37F, PH.base.pack) ;
        p84_y_SET((float) -4.7472935E36F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p84_afx_SET((float)1.9510337E38F, PH.base.pack) ;
        p84_x_SET((float) -1.02488E38F, PH.base.pack) ;
        p84_afy_SET((float)3.2945758E38F, PH.base.pack) ;
        p84_z_SET((float)1.1099538E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p84_vy_SET((float)1.8584514E38F, PH.base.pack) ;
        p84_vx_SET((float)3.179521E38F, PH.base.pack) ;
        p84_vz_SET((float)2.3886864E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)37263, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)4033295809L, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_time_boot_ms_SET((uint32_t)2162802560L, PH.base.pack) ;
        p86_afy_SET((float)2.7821444E38F, PH.base.pack) ;
        p86_vy_SET((float) -2.2250706E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)1559058010, PH.base.pack) ;
        p86_vz_SET((float)1.399556E38F, PH.base.pack) ;
        p86_vx_SET((float)7.973448E37F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1567362979, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p86_afz_SET((float) -6.416846E37F, PH.base.pack) ;
        p86_afx_SET((float)1.718975E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.389407E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)60130, PH.base.pack) ;
        p86_alt_SET((float) -2.738635E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p86_yaw_SET((float)1.5041396E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_lat_int_SET((int32_t) -1824438050, PH.base.pack) ;
        p87_afy_SET((float)3.2144699E38F, PH.base.pack) ;
        p87_vx_SET((float) -1.8053442E38F, PH.base.pack) ;
        p87_yaw_SET((float) -2.150952E38F, PH.base.pack) ;
        p87_afz_SET((float)9.170917E37F, PH.base.pack) ;
        p87_alt_SET((float) -1.6659315E38F, PH.base.pack) ;
        p87_afx_SET((float)7.8328714E37F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)1569448406, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.0333026E36F, PH.base.pack) ;
        p87_vz_SET((float)1.4028186E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)42231, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)4235700299L, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p87_vy_SET((float)3.3450225E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_x_SET((float)6.6979377E37F, PH.base.pack) ;
        p89_roll_SET((float) -8.545645E37F, PH.base.pack) ;
        p89_z_SET((float)3.2487988E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)234747090L, PH.base.pack) ;
        p89_y_SET((float)1.5250185E38F, PH.base.pack) ;
        p89_pitch_SET((float)2.8056473E38F, PH.base.pack) ;
        p89_yaw_SET((float) -2.0320524E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_roll_SET((float) -1.978632E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)1284337630584615164L, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -29280, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)3548, PH.base.pack) ;
        p90_pitchspeed_SET((float) -1.4483352E38F, PH.base.pack) ;
        p90_pitch_SET((float)1.7210468E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -6655, PH.base.pack) ;
        p90_lat_SET((int32_t)324814523, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)29939, PH.base.pack) ;
        p90_yawspeed_SET((float)6.167556E37F, PH.base.pack) ;
        p90_lon_SET((int32_t)289033223, PH.base.pack) ;
        p90_rollspeed_SET((float)3.3793549E38F, PH.base.pack) ;
        p90_alt_SET((int32_t)2102524764, PH.base.pack) ;
        p90_yaw_SET((float)1.5827087E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)30648, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)247, PH.base.pack) ;
        c_TEST_Channel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p91_throttle_SET((float) -6.7706385E37F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.6928678E37F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.5672295E38F, PH.base.pack) ;
        p91_aux3_SET((float)2.6312098E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)5757903766957816839L, PH.base.pack) ;
        p91_aux4_SET((float) -2.7828103E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.2904425E38F, PH.base.pack) ;
        p91_aux1_SET((float)3.2588591E38F, PH.base.pack) ;
        p91_aux2_SET((float) -1.4915871E38F, PH.base.pack) ;
        c_TEST_Channel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_rssi_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)35116, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)7172, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)51820, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)51574, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)48977, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)34362, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)6059012416357554919L, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)2185, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)1317, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)46431, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)25791, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)48363, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)47400, PH.base.pack) ;
        c_TEST_Channel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)4580394412636514916L, PH.base.pack) ;
        {
            float controls[] =  {3.2585622E38F, -3.363136E38F, 1.8356508E38F, -2.123291E38F, 2.8708684E38F, -9.103146E37F, -1.7228568E38F, -6.4989597E37F, -1.6172763E38F, 8.618825E37F, -5.750507E37F, 2.8468954E38F, 2.7720515E38F, -2.379066E38F, 4.2452158E37F, 3.0816034E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)4861710728761731785L, PH.base.pack) ;
        c_TEST_Channel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.9185402E38F, &PH) ;
        p100_flow_x_SET((int16_t)(int16_t) -9888, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)6.2161443E37F, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)4674736177544946472L, PH.base.pack) ;
        p100_ground_distance_SET((float)2.9896874E38F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)2.1452295E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p100_flow_rate_y_SET((float)1.5015584E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)21900, PH.base.pack) ;
        c_TEST_Channel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float) -8.618382E37F, PH.base.pack) ;
        p101_pitch_SET((float) -3.05495E38F, PH.base.pack) ;
        p101_x_SET((float) -2.0929502E38F, PH.base.pack) ;
        p101_yaw_SET((float)2.7658473E38F, PH.base.pack) ;
        p101_roll_SET((float)2.5556797E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)6086547414861448083L, PH.base.pack) ;
        p101_z_SET((float) -1.297407E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float)9.608518E37F, PH.base.pack) ;
        p102_roll_SET((float)2.664251E37F, PH.base.pack) ;
        p102_pitch_SET((float) -1.5696092E38F, PH.base.pack) ;
        p102_y_SET((float) -6.5425115E35F, PH.base.pack) ;
        p102_z_SET((float)1.0085688E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)378144909300573238L, PH.base.pack) ;
        p102_yaw_SET((float) -1.5608023E38F, PH.base.pack) ;
        c_TEST_Channel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float)2.1608978E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)3483613725074284266L, PH.base.pack) ;
        p103_x_SET((float) -3.80839E37F, PH.base.pack) ;
        p103_y_SET((float)2.2293767E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)5921116010459568060L, PH.base.pack) ;
        p104_pitch_SET((float)9.739904E37F, PH.base.pack) ;
        p104_roll_SET((float)1.0015415E38F, PH.base.pack) ;
        p104_y_SET((float)1.089077E38F, PH.base.pack) ;
        p104_x_SET((float)2.7888045E38F, PH.base.pack) ;
        p104_z_SET((float)1.1727845E38F, PH.base.pack) ;
        p104_yaw_SET((float)2.6499134E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_diff_pressure_SET((float)2.1878992E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)817051026694375629L, PH.base.pack) ;
        p105_zacc_SET((float)3.3638346E38F, PH.base.pack) ;
        p105_yacc_SET((float)2.971699E38F, PH.base.pack) ;
        p105_xmag_SET((float)5.2948493E37F, PH.base.pack) ;
        p105_xacc_SET((float) -1.7256002E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -1.5347194E38F, PH.base.pack) ;
        p105_zgyro_SET((float)1.9640729E38F, PH.base.pack) ;
        p105_zmag_SET((float)1.3177413E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.6076913E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)55504, PH.base.pack) ;
        p105_abs_pressure_SET((float) -4.0822598E37F, PH.base.pack) ;
        p105_xgyro_SET((float)3.0429239E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)3.327981E38F, PH.base.pack) ;
        p105_temperature_SET((float)1.3940721E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_y_SET((float) -8.687961E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -2440, PH.base.pack) ;
        p106_integrated_x_SET((float)3.0941276E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -4.6513843E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6622896616186263140L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -4.775582E37F, PH.base.pack) ;
        p106_distance_SET((float) -9.799405E37F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3484616038L, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3730843564L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.2569668E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_yacc_SET((float) -3.0329307E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)241717256L, PH.base.pack) ;
        p107_temperature_SET((float)3.3519027E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.5759528E38F, PH.base.pack) ;
        p107_xacc_SET((float) -6.774448E37F, PH.base.pack) ;
        p107_zmag_SET((float)3.2615453E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)8856169662564523389L, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.4816222E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)1.6570601E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.878419E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -2.3116116E38F, PH.base.pack) ;
        p107_xmag_SET((float)1.1837633E38F, PH.base.pack) ;
        p107_zacc_SET((float)3.2412842E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.2360308E38F, PH.base.pack) ;
        p107_ygyro_SET((float)9.664411E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_zacc_SET((float)1.2866834E38F, PH.base.pack) ;
        p108_q1_SET((float)2.5988707E38F, PH.base.pack) ;
        p108_lon_SET((float)1.5591941E38F, PH.base.pack) ;
        p108_lat_SET((float)8.547024E37F, PH.base.pack) ;
        p108_q3_SET((float) -3.9577668E36F, PH.base.pack) ;
        p108_q4_SET((float) -2.0973673E38F, PH.base.pack) ;
        p108_vd_SET((float) -1.4110689E38F, PH.base.pack) ;
        p108_zgyro_SET((float)2.4962868E38F, PH.base.pack) ;
        p108_pitch_SET((float) -1.7488926E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -9.390899E37F, PH.base.pack) ;
        p108_ve_SET((float)8.9798144E36F, PH.base.pack) ;
        p108_yaw_SET((float)2.7203127E38F, PH.base.pack) ;
        p108_xacc_SET((float)2.0649278E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -2.0625973E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.0651798E38F, PH.base.pack) ;
        p108_yacc_SET((float)1.3250589E38F, PH.base.pack) ;
        p108_vn_SET((float)2.974551E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.2133751E38F, PH.base.pack) ;
        p108_alt_SET((float) -1.6529827E38F, PH.base.pack) ;
        p108_roll_SET((float)3.1632888E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -2.453613E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_noise_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)19146, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)63072, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)141, (uint8_t)243, (uint8_t)166, (uint8_t)144, (uint8_t)138, (uint8_t)131, (uint8_t)206, (uint8_t)38, (uint8_t)39, (uint8_t)59, (uint8_t)156, (uint8_t)98, (uint8_t)41, (uint8_t)212, (uint8_t)21, (uint8_t)182, (uint8_t)134, (uint8_t)63, (uint8_t)222, (uint8_t)104, (uint8_t)66, (uint8_t)140, (uint8_t)88, (uint8_t)248, (uint8_t)150, (uint8_t)114, (uint8_t)115, (uint8_t)78, (uint8_t)24, (uint8_t)210, (uint8_t)208, (uint8_t)0, (uint8_t)104, (uint8_t)67, (uint8_t)103, (uint8_t)134, (uint8_t)6, (uint8_t)95, (uint8_t)9, (uint8_t)107, (uint8_t)249, (uint8_t)189, (uint8_t)56, (uint8_t)102, (uint8_t)222, (uint8_t)218, (uint8_t)220, (uint8_t)246, (uint8_t)103, (uint8_t)20, (uint8_t)160, (uint8_t)26, (uint8_t)136, (uint8_t)187, (uint8_t)132, (uint8_t)196, (uint8_t)224, (uint8_t)26, (uint8_t)58, (uint8_t)243, (uint8_t)12, (uint8_t)184, (uint8_t)183, (uint8_t)48, (uint8_t)151, (uint8_t)19, (uint8_t)49, (uint8_t)33, (uint8_t)25, (uint8_t)189, (uint8_t)117, (uint8_t)181, (uint8_t)131, (uint8_t)69, (uint8_t)225, (uint8_t)63, (uint8_t)63, (uint8_t)254, (uint8_t)4, (uint8_t)121, (uint8_t)224, (uint8_t)227, (uint8_t)233, (uint8_t)74, (uint8_t)115, (uint8_t)99, (uint8_t)108, (uint8_t)30, (uint8_t)22, (uint8_t)196, (uint8_t)201, (uint8_t)21, (uint8_t)38, (uint8_t)82, (uint8_t)206, (uint8_t)227, (uint8_t)110, (uint8_t)110, (uint8_t)166, (uint8_t)63, (uint8_t)94, (uint8_t)229, (uint8_t)127, (uint8_t)80, (uint8_t)106, (uint8_t)27, (uint8_t)7, (uint8_t)59, (uint8_t)224, (uint8_t)56, (uint8_t)235, (uint8_t)137, (uint8_t)143, (uint8_t)82, (uint8_t)132, (uint8_t)226, (uint8_t)192, (uint8_t)23, (uint8_t)145, (uint8_t)27, (uint8_t)13, (uint8_t)125, (uint8_t)49, (uint8_t)78, (uint8_t)128, (uint8_t)30, (uint8_t)240, (uint8_t)100, (uint8_t)34, (uint8_t)82, (uint8_t)71, (uint8_t)3, (uint8_t)49, (uint8_t)252, (uint8_t)14, (uint8_t)60, (uint8_t)111, (uint8_t)29, (uint8_t)86, (uint8_t)66, (uint8_t)111, (uint8_t)98, (uint8_t)187, (uint8_t)251, (uint8_t)140, (uint8_t)55, (uint8_t)153, (uint8_t)33, (uint8_t)144, (uint8_t)113, (uint8_t)186, (uint8_t)212, (uint8_t)228, (uint8_t)176, (uint8_t)180, (uint8_t)132, (uint8_t)172, (uint8_t)182, (uint8_t)234, (uint8_t)140, (uint8_t)25, (uint8_t)173, (uint8_t)190, (uint8_t)147, (uint8_t)156, (uint8_t)31, (uint8_t)103, (uint8_t)92, (uint8_t)87, (uint8_t)150, (uint8_t)174, (uint8_t)241, (uint8_t)53, (uint8_t)196, (uint8_t)173, (uint8_t)119, (uint8_t)42, (uint8_t)216, (uint8_t)47, (uint8_t)111, (uint8_t)101, (uint8_t)64, (uint8_t)147, (uint8_t)235, (uint8_t)23, (uint8_t)83, (uint8_t)113, (uint8_t)174, (uint8_t)204, (uint8_t)100, (uint8_t)113, (uint8_t)105, (uint8_t)53, (uint8_t)172, (uint8_t)112, (uint8_t)203, (uint8_t)235, (uint8_t)14, (uint8_t)131, (uint8_t)186, (uint8_t)236, (uint8_t)42, (uint8_t)10, (uint8_t)128, (uint8_t)37, (uint8_t)171, (uint8_t)113, (uint8_t)213, (uint8_t)135, (uint8_t)25, (uint8_t)107, (uint8_t)129, (uint8_t)154, (uint8_t)223, (uint8_t)162, (uint8_t)30, (uint8_t)208, (uint8_t)181, (uint8_t)63, (uint8_t)0, (uint8_t)32, (uint8_t)12, (uint8_t)233, (uint8_t)218, (uint8_t)251, (uint8_t)236, (uint8_t)176, (uint8_t)6, (uint8_t)216, (uint8_t)1, (uint8_t)15, (uint8_t)170, (uint8_t)103, (uint8_t)147, (uint8_t)174, (uint8_t)119, (uint8_t)26, (uint8_t)54, (uint8_t)217, (uint8_t)143, (uint8_t)213, (uint8_t)99, (uint8_t)217, (uint8_t)77, (uint8_t)210, (uint8_t)24, (uint8_t)229, (uint8_t)161, (uint8_t)137, (uint8_t)16, (uint8_t)56};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t) -9018038205928207881L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -4531486100787339264L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)704945899L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)3710759477213164168L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_lat_SET((int32_t)956735742, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -13671, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p113_lon_SET((int32_t) -1658080600, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -25816, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)16881, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)42145, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)27102, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)53593, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)114870830184988021L, PH.base.pack) ;
        p113_alt_SET((int32_t)2042698332, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)2246, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_time_delta_distance_us_SET((uint32_t)861976574L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -29949, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p114_distance_SET((float)1.0443086E37F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)370485846L, PH.base.pack) ;
        p114_integrated_x_SET((float)3.1109383E37F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -3.0582078E37F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.800972E37F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -3.0068013E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)2.6493324E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)4269665010210588135L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_vz_SET((int16_t)(int16_t) -23725, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -8988, PH.base.pack) ;
        p115_rollspeed_SET((float)2.7413259E38F, PH.base.pack) ;
        p115_pitchspeed_SET((float) -2.7374696E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -11477, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)44256, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -20426, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -25426, PH.base.pack) ;
        p115_lat_SET((int32_t) -1908724081, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {3.3041836E38F, 3.368426E38F, -1.7960792E38F, 3.0419215E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_true_airspeed_SET((uint16_t)(uint16_t)62680, PH.base.pack) ;
        p115_lon_SET((int32_t)1412966736, PH.base.pack) ;
        p115_yawspeed_SET((float)2.2026764E38F, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)4156186339292580402L, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)30447, PH.base.pack) ;
        p115_alt_SET((int32_t)832105460, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t) -18253, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)8886, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -27049, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)14337, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -22630, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)4070955076L, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)19524, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)17260, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)18208, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -21932, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)37076, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)29198, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_time_utc_SET((uint32_t)4205203566L, PH.base.pack) ;
        p118_size_SET((uint32_t)3369408209L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)20915, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)13177, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)22081, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)553840313L, PH.base.pack) ;
        p119_ofs_SET((uint32_t)607567072L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)61585, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)4928, PH.base.pack) ;
        p120_ofs_SET((uint32_t)148672024L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)74, (uint8_t)3, (uint8_t)32, (uint8_t)176, (uint8_t)70, (uint8_t)136, (uint8_t)137, (uint8_t)55, (uint8_t)155, (uint8_t)96, (uint8_t)79, (uint8_t)219, (uint8_t)200, (uint8_t)78, (uint8_t)253, (uint8_t)37, (uint8_t)161, (uint8_t)133, (uint8_t)221, (uint8_t)32, (uint8_t)100, (uint8_t)196, (uint8_t)126, (uint8_t)51, (uint8_t)81, (uint8_t)76, (uint8_t)230, (uint8_t)160, (uint8_t)29, (uint8_t)253, (uint8_t)92, (uint8_t)19, (uint8_t)248, (uint8_t)144, (uint8_t)113, (uint8_t)160, (uint8_t)216, (uint8_t)202, (uint8_t)195, (uint8_t)74, (uint8_t)240, (uint8_t)168, (uint8_t)17, (uint8_t)72, (uint8_t)156, (uint8_t)166, (uint8_t)106, (uint8_t)5, (uint8_t)48, (uint8_t)180, (uint8_t)145, (uint8_t)7, (uint8_t)127, (uint8_t)176, (uint8_t)140, (uint8_t)243, (uint8_t)179, (uint8_t)184, (uint8_t)107, (uint8_t)16, (uint8_t)149, (uint8_t)166, (uint8_t)86, (uint8_t)233, (uint8_t)5, (uint8_t)87, (uint8_t)177, (uint8_t)155, (uint8_t)219, (uint8_t)46, (uint8_t)108, (uint8_t)35, (uint8_t)40, (uint8_t)250, (uint8_t)42, (uint8_t)68, (uint8_t)27, (uint8_t)198, (uint8_t)60, (uint8_t)187, (uint8_t)229, (uint8_t)71, (uint8_t)164, (uint8_t)165, (uint8_t)235, (uint8_t)88, (uint8_t)198, (uint8_t)15, (uint8_t)87, (uint8_t)76};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)132, (uint8_t)49, (uint8_t)194, (uint8_t)187, (uint8_t)216, (uint8_t)43, (uint8_t)66, (uint8_t)229, (uint8_t)98, (uint8_t)93, (uint8_t)9, (uint8_t)105, (uint8_t)137, (uint8_t)55, (uint8_t)4, (uint8_t)184, (uint8_t)147, (uint8_t)211, (uint8_t)22, (uint8_t)42, (uint8_t)185, (uint8_t)153, (uint8_t)208, (uint8_t)87, (uint8_t)241, (uint8_t)63, (uint8_t)123, (uint8_t)213, (uint8_t)186, (uint8_t)238, (uint8_t)85, (uint8_t)216, (uint8_t)167, (uint8_t)207, (uint8_t)145, (uint8_t)104, (uint8_t)201, (uint8_t)23, (uint8_t)5, (uint8_t)111, (uint8_t)175, (uint8_t)209, (uint8_t)160, (uint8_t)195, (uint8_t)136, (uint8_t)192, (uint8_t)245, (uint8_t)72, (uint8_t)13, (uint8_t)15, (uint8_t)134, (uint8_t)127, (uint8_t)116, (uint8_t)103, (uint8_t)59, (uint8_t)167, (uint8_t)193, (uint8_t)41, (uint8_t)116, (uint8_t)3, (uint8_t)93, (uint8_t)244, (uint8_t)200, (uint8_t)28, (uint8_t)232, (uint8_t)82, (uint8_t)98, (uint8_t)106, (uint8_t)100, (uint8_t)237, (uint8_t)194, (uint8_t)26, (uint8_t)3, (uint8_t)82, (uint8_t)209, (uint8_t)48, (uint8_t)158, (uint8_t)96, (uint8_t)177, (uint8_t)108, (uint8_t)148, (uint8_t)60, (uint8_t)146, (uint8_t)250, (uint8_t)25, (uint8_t)92, (uint8_t)124, (uint8_t)164, (uint8_t)28, (uint8_t)251, (uint8_t)22, (uint8_t)14, (uint8_t)28, (uint8_t)147, (uint8_t)116, (uint8_t)102, (uint8_t)122, (uint8_t)243, (uint8_t)122, (uint8_t)34, (uint8_t)164, (uint8_t)142, (uint8_t)13, (uint8_t)217, (uint8_t)104, (uint8_t)194, (uint8_t)140, (uint8_t)251, (uint8_t)121, (uint8_t)73};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_lon_SET((int32_t) -941543304, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)61587, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)45678, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)26970, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)1131950209486071480L, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)3818283698L, PH.base.pack) ;
        p124_lat_SET((int32_t) -1220777752, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)25641, PH.base.pack) ;
        p124_alt_SET((int32_t)1646134069, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)16079, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)52439, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)1977294454L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)21, (uint8_t)230, (uint8_t)191, (uint8_t)199, (uint8_t)245, (uint8_t)191, (uint8_t)35, (uint8_t)155, (uint8_t)147, (uint8_t)198, (uint8_t)124, (uint8_t)120, (uint8_t)20, (uint8_t)109, (uint8_t)235, (uint8_t)228, (uint8_t)71, (uint8_t)98, (uint8_t)80, (uint8_t)250, (uint8_t)139, (uint8_t)168, (uint8_t)173, (uint8_t)213, (uint8_t)119, (uint8_t)215, (uint8_t)54, (uint8_t)196, (uint8_t)250, (uint8_t)74, (uint8_t)53, (uint8_t)23, (uint8_t)138, (uint8_t)220, (uint8_t)81, (uint8_t)109, (uint8_t)15, (uint8_t)146, (uint8_t)59, (uint8_t)223, (uint8_t)150, (uint8_t)93, (uint8_t)95, (uint8_t)200, (uint8_t)129, (uint8_t)49, (uint8_t)189, (uint8_t)223, (uint8_t)216, (uint8_t)70, (uint8_t)107, (uint8_t)51, (uint8_t)89, (uint8_t)98, (uint8_t)24, (uint8_t)233, (uint8_t)249, (uint8_t)149, (uint8_t)188, (uint8_t)49, (uint8_t)149, (uint8_t)255, (uint8_t)83, (uint8_t)125, (uint8_t)235, (uint8_t)129, (uint8_t)207, (uint8_t)215, (uint8_t)134, (uint8_t)77};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)29413, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1936516311, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -1786774486, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)2016806904, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)1345958897L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)31214, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)3906102190L, PH.base.pack) ;
        p127_tow_SET((uint32_t)775965332L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)836759433, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -805299359, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)1290874176L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)60403, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)1916497142L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1685412661, PH.base.pack) ;
        p128_tow_SET((uint32_t)535457869L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -163644426, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1007744307, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_xacc_SET((int16_t)(int16_t) -8351, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -7600, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)8928, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -95, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1958407190L, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -22720, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)8762, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -27100, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -24656, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -24924, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_packets_SET((uint16_t)(uint16_t)40212, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)27403, PH.base.pack) ;
        p130_size_SET((uint32_t)2357108665L, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)16517, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)29687, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)180, (uint8_t)153, (uint8_t)59, (uint8_t)162, (uint8_t)193, (uint8_t)213, (uint8_t)15, (uint8_t)69, (uint8_t)30, (uint8_t)217, (uint8_t)77, (uint8_t)85, (uint8_t)26, (uint8_t)7, (uint8_t)221, (uint8_t)195, (uint8_t)65, (uint8_t)170, (uint8_t)216, (uint8_t)208, (uint8_t)21, (uint8_t)199, (uint8_t)134, (uint8_t)182, (uint8_t)43, (uint8_t)206, (uint8_t)132, (uint8_t)199, (uint8_t)92, (uint8_t)36, (uint8_t)139, (uint8_t)202, (uint8_t)29, (uint8_t)182, (uint8_t)103, (uint8_t)164, (uint8_t)79, (uint8_t)170, (uint8_t)32, (uint8_t)97, (uint8_t)250, (uint8_t)88, (uint8_t)219, (uint8_t)31, (uint8_t)44, (uint8_t)164, (uint8_t)142, (uint8_t)104, (uint8_t)220, (uint8_t)209, (uint8_t)82, (uint8_t)214, (uint8_t)198, (uint8_t)107, (uint8_t)207, (uint8_t)89, (uint8_t)138, (uint8_t)82, (uint8_t)14, (uint8_t)212, (uint8_t)80, (uint8_t)239, (uint8_t)0, (uint8_t)79, (uint8_t)106, (uint8_t)218, (uint8_t)96, (uint8_t)79, (uint8_t)255, (uint8_t)123, (uint8_t)234, (uint8_t)218, (uint8_t)193, (uint8_t)91, (uint8_t)111, (uint8_t)20, (uint8_t)9, (uint8_t)249, (uint8_t)177, (uint8_t)133, (uint8_t)2, (uint8_t)70, (uint8_t)213, (uint8_t)183, (uint8_t)22, (uint8_t)58, (uint8_t)213, (uint8_t)41, (uint8_t)96, (uint8_t)42, (uint8_t)213, (uint8_t)250, (uint8_t)139, (uint8_t)90, (uint8_t)219, (uint8_t)206, (uint8_t)148, (uint8_t)177, (uint8_t)205, (uint8_t)96, (uint8_t)171, (uint8_t)203, (uint8_t)157, (uint8_t)137, (uint8_t)215, (uint8_t)179, (uint8_t)46, (uint8_t)78, (uint8_t)125, (uint8_t)89, (uint8_t)184, (uint8_t)95, (uint8_t)4, (uint8_t)130, (uint8_t)175, (uint8_t)156, (uint8_t)249, (uint8_t)14, (uint8_t)100, (uint8_t)157, (uint8_t)152, (uint8_t)122, (uint8_t)31, (uint8_t)76, (uint8_t)23, (uint8_t)111, (uint8_t)68, (uint8_t)196, (uint8_t)19, (uint8_t)72, (uint8_t)156, (uint8_t)219, (uint8_t)248, (uint8_t)74, (uint8_t)218, (uint8_t)93, (uint8_t)0, (uint8_t)194, (uint8_t)167, (uint8_t)5, (uint8_t)138, (uint8_t)249, (uint8_t)189, (uint8_t)144, (uint8_t)115, (uint8_t)241, (uint8_t)231, (uint8_t)87, (uint8_t)224, (uint8_t)125, (uint8_t)179, (uint8_t)195, (uint8_t)49, (uint8_t)180, (uint8_t)160, (uint8_t)239, (uint8_t)90, (uint8_t)222, (uint8_t)36, (uint8_t)59, (uint8_t)184, (uint8_t)171, (uint8_t)33, (uint8_t)93, (uint8_t)155, (uint8_t)231, (uint8_t)224, (uint8_t)2, (uint8_t)73, (uint8_t)35, (uint8_t)105, (uint8_t)129, (uint8_t)77, (uint8_t)245, (uint8_t)101, (uint8_t)158, (uint8_t)67, (uint8_t)166, (uint8_t)172, (uint8_t)239, (uint8_t)7, (uint8_t)167, (uint8_t)153, (uint8_t)182, (uint8_t)112, (uint8_t)106, (uint8_t)108, (uint8_t)208, (uint8_t)33, (uint8_t)146, (uint8_t)53, (uint8_t)46, (uint8_t)255, (uint8_t)207, (uint8_t)31, (uint8_t)181, (uint8_t)225, (uint8_t)116, (uint8_t)216, (uint8_t)43, (uint8_t)67, (uint8_t)141, (uint8_t)130, (uint8_t)69, (uint8_t)116, (uint8_t)240, (uint8_t)210, (uint8_t)236, (uint8_t)172, (uint8_t)31, (uint8_t)104, (uint8_t)11, (uint8_t)180, (uint8_t)254, (uint8_t)88, (uint8_t)205, (uint8_t)220, (uint8_t)169, (uint8_t)46, (uint8_t)247, (uint8_t)90, (uint8_t)94, (uint8_t)96, (uint8_t)255, (uint8_t)62, (uint8_t)84, (uint8_t)84, (uint8_t)224, (uint8_t)81, (uint8_t)99, (uint8_t)80, (uint8_t)255, (uint8_t)253, (uint8_t)175, (uint8_t)174, (uint8_t)204, (uint8_t)73, (uint8_t)213, (uint8_t)3, (uint8_t)45, (uint8_t)140, (uint8_t)213, (uint8_t)166, (uint8_t)181, (uint8_t)130, (uint8_t)99, (uint8_t)229, (uint8_t)202, (uint8_t)84, (uint8_t)159, (uint8_t)188, (uint8_t)12, (uint8_t)44};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_min_distance_SET((uint16_t)(uint16_t)52494, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)36267, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)49139, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1806799806L, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_90, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lon_SET((int32_t) -265485409, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)60286, PH.base.pack) ;
        p133_mask_SET((uint64_t)226161708013557847L, PH.base.pack) ;
        p133_lat_SET((int32_t)700250427, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t) -2063664651, PH.base.pack) ;
        p134_lon_SET((int32_t) -858460987, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)48639, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)16424, (int16_t) -11260, (int16_t)20187, (int16_t)4880, (int16_t) -28203, (int16_t) -24626, (int16_t)24512, (int16_t) -18363, (int16_t) -20329, (int16_t)18125, (int16_t)27339, (int16_t)372, (int16_t)10559, (int16_t) -4121, (int16_t)11913, (int16_t) -11405};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -155611062, PH.base.pack) ;
        p135_lon_SET((int32_t)216688408, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_loaded_SET((uint16_t)(uint16_t)34357, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)6766, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)10718, PH.base.pack) ;
        p136_lon_SET((int32_t)1327804119, PH.base.pack) ;
        p136_lat_SET((int32_t)1272809585, PH.base.pack) ;
        p136_current_height_SET((float)3.333893E37F, PH.base.pack) ;
        p136_terrain_height_SET((float) -6.980262E37F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float) -9.4548346E35F, PH.base.pack) ;
        p137_press_abs_SET((float) -5.0367394E37F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)2767625860L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -27537, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_z_SET((float) -9.81521E37F, PH.base.pack) ;
        p138_y_SET((float)1.200075E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)2398244052133605070L, PH.base.pack) ;
        {
            float q[] =  {-1.6825519E38F, -5.7302203E37F, -1.2200169E38F, 1.0237102E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)2.0221032E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-1.0809938E38F, 2.953559E38F, -2.2081037E38F, 1.7026683E38F, 2.4622782E38F, 3.0976422E38F, 1.0592633E38F, -2.9436056E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_time_usec_SET((uint64_t)4812012534207840281L, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-7.965937E36F, -3.2916335E38F, -1.3553233E38F, -3.2250295E38F, -2.6435632E38F, 2.3159084E38F, -2.3341163E38F, -2.0981883E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)1951597053655921652L, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float)1.5789403E37F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.5503402E38F, PH.base.pack) ;
        p141_altitude_local_SET((float)2.8037407E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -1.5307039E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -3.366431E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.4512852E36F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)3310766920882334836L, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)118, (uint8_t)136, (uint8_t)225, (uint8_t)234, (uint8_t)65, (uint8_t)223, (uint8_t)0, (uint8_t)137, (uint8_t)7, (uint8_t)89, (uint8_t)211, (uint8_t)135, (uint8_t)193, (uint8_t)25, (uint8_t)151, (uint8_t)45, (uint8_t)190, (uint8_t)21, (uint8_t)100, (uint8_t)54, (uint8_t)1, (uint8_t)26, (uint8_t)19, (uint8_t)116, (uint8_t)22, (uint8_t)28, (uint8_t)110, (uint8_t)98, (uint8_t)199, (uint8_t)235, (uint8_t)203, (uint8_t)97, (uint8_t)148, (uint8_t)220, (uint8_t)58, (uint8_t)164, (uint8_t)200, (uint8_t)144, (uint8_t)177, (uint8_t)216, (uint8_t)110, (uint8_t)136, (uint8_t)249, (uint8_t)151, (uint8_t)144, (uint8_t)126, (uint8_t)213, (uint8_t)145, (uint8_t)123, (uint8_t)176, (uint8_t)130, (uint8_t)126, (uint8_t)213, (uint8_t)112, (uint8_t)194, (uint8_t)244, (uint8_t)233, (uint8_t)129, (uint8_t)47, (uint8_t)29, (uint8_t)3, (uint8_t)97, (uint8_t)9, (uint8_t)130, (uint8_t)73, (uint8_t)27, (uint8_t)192, (uint8_t)6, (uint8_t)155, (uint8_t)83, (uint8_t)58, (uint8_t)142, (uint8_t)163, (uint8_t)62, (uint8_t)234, (uint8_t)133, (uint8_t)15, (uint8_t)238, (uint8_t)107, (uint8_t)78, (uint8_t)57, (uint8_t)104, (uint8_t)54, (uint8_t)124, (uint8_t)124, (uint8_t)203, (uint8_t)250, (uint8_t)57, (uint8_t)201, (uint8_t)90, (uint8_t)18, (uint8_t)103, (uint8_t)164, (uint8_t)220, (uint8_t)250, (uint8_t)185, (uint8_t)30, (uint8_t)91, (uint8_t)73, (uint8_t)35, (uint8_t)106, (uint8_t)12, (uint8_t)206, (uint8_t)150, (uint8_t)175, (uint8_t)166, (uint8_t)212, (uint8_t)173, (uint8_t)215, (uint8_t)64, (uint8_t)229, (uint8_t)49, (uint8_t)76, (uint8_t)13, (uint8_t)148, (uint8_t)150, (uint8_t)196, (uint8_t)155, (uint8_t)80, (uint8_t)10};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)150, (uint8_t)128, (uint8_t)129, (uint8_t)107, (uint8_t)49, (uint8_t)126, (uint8_t)11, (uint8_t)107, (uint8_t)107, (uint8_t)130, (uint8_t)61, (uint8_t)184, (uint8_t)82, (uint8_t)125, (uint8_t)239, (uint8_t)142, (uint8_t)53, (uint8_t)249, (uint8_t)193, (uint8_t)208, (uint8_t)138, (uint8_t)163, (uint8_t)136, (uint8_t)195, (uint8_t)112, (uint8_t)138, (uint8_t)146, (uint8_t)172, (uint8_t)205, (uint8_t)239, (uint8_t)26, (uint8_t)225, (uint8_t)31, (uint8_t)80, (uint8_t)124, (uint8_t)196, (uint8_t)154, (uint8_t)112, (uint8_t)185, (uint8_t)154, (uint8_t)97, (uint8_t)155, (uint8_t)31, (uint8_t)144, (uint8_t)61, (uint8_t)109, (uint8_t)219, (uint8_t)178, (uint8_t)119, (uint8_t)221, (uint8_t)169, (uint8_t)43, (uint8_t)38, (uint8_t)15, (uint8_t)164, (uint8_t)243, (uint8_t)177, (uint8_t)140, (uint8_t)147, (uint8_t)240, (uint8_t)124, (uint8_t)99, (uint8_t)220, (uint8_t)175, (uint8_t)68, (uint8_t)151, (uint8_t)24, (uint8_t)125, (uint8_t)239, (uint8_t)246, (uint8_t)59, (uint8_t)2, (uint8_t)127, (uint8_t)71, (uint8_t)228, (uint8_t)6, (uint8_t)179, (uint8_t)1, (uint8_t)242, (uint8_t)8, (uint8_t)9, (uint8_t)164, (uint8_t)128, (uint8_t)101, (uint8_t)51, (uint8_t)79, (uint8_t)16, (uint8_t)99, (uint8_t)129, (uint8_t)211, (uint8_t)79, (uint8_t)37, (uint8_t)90, (uint8_t)3, (uint8_t)18, (uint8_t)195, (uint8_t)91, (uint8_t)153, (uint8_t)157, (uint8_t)135, (uint8_t)53, (uint8_t)199, (uint8_t)18, (uint8_t)54, (uint8_t)178, (uint8_t)56, (uint8_t)184, (uint8_t)19, (uint8_t)226, (uint8_t)49, (uint8_t)186, (uint8_t)93, (uint8_t)232, (uint8_t)127, (uint8_t)241, (uint8_t)227, (uint8_t)160, (uint8_t)166, (uint8_t)104, (uint8_t)239};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t) -18076, PH.base.pack) ;
        p143_press_abs_SET((float)2.0003854E37F, PH.base.pack) ;
        p143_press_diff_SET((float)1.9896304E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3240972552L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_alt_SET((float)1.4237775E38F, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        {
            float attitude_q[] =  {2.2767098E38F, -1.0610788E38F, -4.7190484E37F, 2.9480178E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-3.5204995E37F, 2.6464125E38F, -1.5242135E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-2.2384128E38F, 1.6639044E38F, -1.2413826E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)340614057, PH.base.pack) ;
        {
            float rates[] =  {2.410588E38F, 1.9170337E38F, -1.5112452E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {9.404818E37F, -1.7179199E38F, 2.6446526E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)3817421245373467765L, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)2933913172635936189L, PH.base.pack) ;
        p144_lat_SET((int32_t) -818899896, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_acc_SET((float) -8.0365463E37F, PH.base.pack) ;
        p146_x_acc_SET((float)2.155039E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.4099232E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float)1.3322482E38F, PH.base.pack) ;
        {
            float q[] =  {-2.0764512E38F, 8.510974E37F, 2.473523E38F, -3.3946685E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float pos_variance[] =  {3.8466939E37F, 1.9940373E37F, -1.6056338E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        {
            float vel_variance[] =  {1.6276689E38F, 1.3297273E38F, -2.428628E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_yaw_rate_SET((float) -1.507402E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -2.9230636E38F, PH.base.pack) ;
        p146_z_vel_SET((float)3.2140311E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -2.6546812E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)2018212447498306088L, PH.base.pack) ;
        p146_airspeed_SET((float) -1.739689E38F, PH.base.pack) ;
        p146_y_acc_SET((float)7.3436045E36F, PH.base.pack) ;
        p146_y_pos_SET((float)2.2540122E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -3.7477015E37F, PH.base.pack) ;
        p146_x_pos_SET((float) -3.2032087E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_id_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -1192368493, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)102835190, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)9718, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)90, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)12530, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)13654, (uint16_t)20284, (uint16_t)17962, (uint16_t)13063, (uint16_t)27123, (uint16_t)41567, (uint16_t)57755, (uint16_t)10987, (uint16_t)21801, (uint16_t)12750};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t uid2[] =  {(uint8_t)154, (uint8_t)193, (uint8_t)25, (uint8_t)162, (uint8_t)217, (uint8_t)25, (uint8_t)35, (uint8_t)145, (uint8_t)224, (uint8_t)20, (uint8_t)197, (uint8_t)45, (uint8_t)105, (uint8_t)62, (uint8_t)140, (uint8_t)146, (uint8_t)86, (uint8_t)87};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_uid_SET((uint64_t)8134379424959737666L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)26686, PH.base.pack) ;
        p148_board_version_SET((uint32_t)2159590131L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)990375664L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)829175315L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)386754083L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)37477, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)25, (uint8_t)169, (uint8_t)100, (uint8_t)66, (uint8_t)247, (uint8_t)100, (uint8_t)237, (uint8_t)5};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT), PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)57, (uint8_t)121, (uint8_t)167, (uint8_t)148, (uint8_t)236, (uint8_t)80, (uint8_t)242, (uint8_t)34};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t os_custom_version[] =  {(uint8_t)186, (uint8_t)23, (uint8_t)177, (uint8_t)227, (uint8_t)125, (uint8_t)224, (uint8_t)171, (uint8_t)109};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)228, &PH) ;
        p149_y_SET((float)8.793153E37F, &PH) ;
        p149_size_x_SET((float)2.1897512E38F, PH.base.pack) ;
        p149_size_y_SET((float) -2.3204162E38F, PH.base.pack) ;
        {
            float q[] =  {3.7198185E37F, 1.0602378E38F, -2.2280415E37F, -1.1215918E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_time_usec_SET((uint64_t)2318874480956988797L, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p149_z_SET((float)1.5471252E38F, &PH) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_angle_y_SET((float) -1.9481183E38F, PH.base.pack) ;
        p149_distance_SET((float) -2.8338082E38F, PH.base.pack) ;
        p149_x_SET((float)1.1190085E38F, &PH) ;
        p149_angle_x_SET((float) -7.2453087E37F, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_OFFSETS_150(), &PH);
        p150_mag_ofs_z_SET((int16_t)(int16_t)9311, PH.base.pack) ;
        p150_gyro_cal_y_SET((float)1.3704507E37F, PH.base.pack) ;
        p150_mag_declination_SET((float)2.414507E38F, PH.base.pack) ;
        p150_raw_press_SET((int32_t)503066702, PH.base.pack) ;
        p150_raw_temp_SET((int32_t)14578107, PH.base.pack) ;
        p150_accel_cal_x_SET((float) -2.6226803E38F, PH.base.pack) ;
        p150_mag_ofs_x_SET((int16_t)(int16_t) -9261, PH.base.pack) ;
        p150_accel_cal_z_SET((float) -2.72759E38F, PH.base.pack) ;
        p150_accel_cal_y_SET((float) -3.3353135E38F, PH.base.pack) ;
        p150_gyro_cal_x_SET((float) -1.5491786E38F, PH.base.pack) ;
        p150_gyro_cal_z_SET((float) -2.295566E38F, PH.base.pack) ;
        p150_mag_ofs_y_SET((int16_t)(int16_t)22394, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_OFFSETS_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_MAG_OFFSETS_151(), &PH);
        p151_mag_ofs_z_SET((int16_t)(int16_t)24110, PH.base.pack) ;
        p151_mag_ofs_x_SET((int16_t)(int16_t) -27626, PH.base.pack) ;
        p151_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p151_mag_ofs_y_SET((int16_t)(int16_t)6611, PH.base.pack) ;
        c_CommunicationChannel_on_SET_MAG_OFFSETS_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMINFO_152(), &PH);
        p152_brkval_SET((uint16_t)(uint16_t)46330, PH.base.pack) ;
        p152_freemem_SET((uint16_t)(uint16_t)23606, PH.base.pack) ;
        p152_freemem32_SET((uint32_t)2206115995L, &PH) ;
        c_CommunicationChannel_on_MEMINFO_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AP_ADC_153(), &PH);
        p153_adc4_SET((uint16_t)(uint16_t)10236, PH.base.pack) ;
        p153_adc3_SET((uint16_t)(uint16_t)57669, PH.base.pack) ;
        p153_adc2_SET((uint16_t)(uint16_t)39303, PH.base.pack) ;
        p153_adc1_SET((uint16_t)(uint16_t)47928, PH.base.pack) ;
        p153_adc5_SET((uint16_t)(uint16_t)2807, PH.base.pack) ;
        p153_adc6_SET((uint16_t)(uint16_t)51293, PH.base.pack) ;
        c_CommunicationChannel_on_AP_ADC_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONFIGURE_154(), &PH);
        p154_iso_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p154_command_id_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p154_engine_cut_off_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p154_mode_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p154_extra_param_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p154_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p154_aperture_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p154_extra_value_SET((float)1.1613895E37F, PH.base.pack) ;
        p154_shutter_speed_SET((uint16_t)(uint16_t)19377, PH.base.pack) ;
        p154_target_component_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p154_exposure_type_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIGICAM_CONTROL_155(), &PH);
        p155_focus_lock_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p155_zoom_step_SET((int8_t)(int8_t) -66, PH.base.pack) ;
        p155_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p155_extra_value_SET((float) -3.294074E38F, PH.base.pack) ;
        p155_extra_param_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p155_command_id_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p155_zoom_pos_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p155_shot_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p155_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p155_session_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_CommunicationChannel_on_DIGICAM_CONTROL_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONFIGURE_156(), &PH);
        p156_stab_yaw_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p156_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL, PH.base.pack) ;
        p156_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p156_stab_pitch_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p156_stab_roll_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONFIGURE_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_CONTROL_157(), &PH);
        p157_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p157_input_b_SET((int32_t) -1418641959, PH.base.pack) ;
        p157_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p157_save_position_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p157_input_c_SET((int32_t)1736843062, PH.base.pack) ;
        p157_input_a_SET((int32_t) -1410429504, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_CONTROL_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_STATUS_158(), &PH);
        p158_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p158_pointing_a_SET((int32_t)1521328363, PH.base.pack) ;
        p158_pointing_b_SET((int32_t)5041733, PH.base.pack) ;
        p158_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p158_pointing_c_SET((int32_t) -640303132, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_STATUS_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_POINT_160(), &PH);
        p160_lng_SET((float)1.85313E38F, PH.base.pack) ;
        p160_count_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p160_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p160_idx_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p160_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p160_lat_SET((float) -1.2652158E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_POINT_160(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_FETCH_POINT_161(), &PH);
        p161_idx_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p161_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p161_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_FETCH_POINT_161(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FENCE_STATUS_162(), &PH);
        p162_breach_time_SET((uint32_t)1056811203L, PH.base.pack) ;
        p162_breach_status_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p162_breach_count_SET((uint16_t)(uint16_t)12368, PH.base.pack) ;
        p162_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_MAXALT, PH.base.pack) ;
        c_CommunicationChannel_on_FENCE_STATUS_162(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS_163(), &PH);
        p163_omegaIz_SET((float) -1.7242366E38F, PH.base.pack) ;
        p163_omegaIx_SET((float) -5.477533E37F, PH.base.pack) ;
        p163_accel_weight_SET((float) -1.3689269E38F, PH.base.pack) ;
        p163_error_yaw_SET((float)3.341284E38F, PH.base.pack) ;
        p163_omegaIy_SET((float)3.845038E37F, PH.base.pack) ;
        p163_renorm_val_SET((float)2.5822647E38F, PH.base.pack) ;
        p163_error_rp_SET((float) -2.3232701E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS_163(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SIMSTATE_164(), &PH);
        p164_lng_SET((int32_t) -1875901161, PH.base.pack) ;
        p164_yaw_SET((float)3.275304E38F, PH.base.pack) ;
        p164_zacc_SET((float)1.7711585E37F, PH.base.pack) ;
        p164_yacc_SET((float) -1.5755294E38F, PH.base.pack) ;
        p164_xacc_SET((float)1.1208787E37F, PH.base.pack) ;
        p164_pitch_SET((float)2.7210398E38F, PH.base.pack) ;
        p164_zgyro_SET((float)2.423396E38F, PH.base.pack) ;
        p164_ygyro_SET((float) -2.3258208E38F, PH.base.pack) ;
        p164_roll_SET((float) -1.4262921E38F, PH.base.pack) ;
        p164_xgyro_SET((float)1.5215825E38F, PH.base.pack) ;
        p164_lat_SET((int32_t)1205239707, PH.base.pack) ;
        c_CommunicationChannel_on_SIMSTATE_164(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HWSTATUS_165(), &PH);
        p165_I2Cerr_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p165_Vcc_SET((uint16_t)(uint16_t)47956, PH.base.pack) ;
        c_CommunicationChannel_on_HWSTATUS_165(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RADIO_166(), &PH);
        p166_rssi_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p166_noise_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p166_remrssi_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p166_rxerrors_SET((uint16_t)(uint16_t)6362, PH.base.pack) ;
        p166_remnoise_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p166_fixed__SET((uint16_t)(uint16_t)18137, PH.base.pack) ;
        p166_txbuf_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_166(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LIMITS_STATUS_167(), &PH);
        p167_last_clear_SET((uint32_t)775621687L, PH.base.pack) ;
        p167_last_action_SET((uint32_t)1777506543L, PH.base.pack) ;
        p167_mods_triggered_SET((e_LIMIT_MODULE_LIMIT_GPSLOCK), PH.base.pack) ;
        p167_mods_enabled_SET((e_LIMIT_MODULE_LIMIT_GEOFENCE), PH.base.pack) ;
        p167_last_trigger_SET((uint32_t)781938888L, PH.base.pack) ;
        p167_breach_count_SET((uint16_t)(uint16_t)39664, PH.base.pack) ;
        p167_limits_state_SET(e_LIMITS_STATE_LIMITS_INIT, PH.base.pack) ;
        p167_last_recovery_SET((uint32_t)2414151283L, PH.base.pack) ;
        p167_mods_required_SET((e_LIMIT_MODULE_LIMIT_GEOFENCE |
                                e_LIMIT_MODULE_LIMIT_GPSLOCK), PH.base.pack) ;
        c_CommunicationChannel_on_LIMITS_STATUS_167(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_168(), &PH);
        p168_direction_SET((float)2.4293781E38F, PH.base.pack) ;
        p168_speed_z_SET((float)1.9983788E37F, PH.base.pack) ;
        p168_speed_SET((float) -1.1065599E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_168(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA16_169(), &PH);
        p169_len_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)123, (uint8_t)64, (uint8_t)6, (uint8_t)70, (uint8_t)107, (uint8_t)134, (uint8_t)89, (uint8_t)100, (uint8_t)177, (uint8_t)122, (uint8_t)84, (uint8_t)44, (uint8_t)242, (uint8_t)82, (uint8_t)135, (uint8_t)147};
            p169_data__SET(&data_, 0, PH.base.pack) ;
        }
        p169_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        c_CommunicationChannel_on_DATA16_169(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA32_170(), &PH);
        p170_type_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)73, (uint8_t)156, (uint8_t)35, (uint8_t)219, (uint8_t)14, (uint8_t)82, (uint8_t)36, (uint8_t)118, (uint8_t)146, (uint8_t)61, (uint8_t)177, (uint8_t)141, (uint8_t)74, (uint8_t)111, (uint8_t)10, (uint8_t)72, (uint8_t)207, (uint8_t)75, (uint8_t)43, (uint8_t)62, (uint8_t)3, (uint8_t)161, (uint8_t)213, (uint8_t)193, (uint8_t)143, (uint8_t)43, (uint8_t)211, (uint8_t)183, (uint8_t)2, (uint8_t)3, (uint8_t)62, (uint8_t)46};
            p170_data__SET(&data_, 0, PH.base.pack) ;
        }
        p170_len_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_CommunicationChannel_on_DATA32_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA64_171(), &PH);
        p171_len_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)22, (uint8_t)23, (uint8_t)3, (uint8_t)239, (uint8_t)50, (uint8_t)200, (uint8_t)26, (uint8_t)210, (uint8_t)86, (uint8_t)91, (uint8_t)138, (uint8_t)125, (uint8_t)41, (uint8_t)4, (uint8_t)87, (uint8_t)13, (uint8_t)40, (uint8_t)105, (uint8_t)237, (uint8_t)95, (uint8_t)62, (uint8_t)101, (uint8_t)31, (uint8_t)192, (uint8_t)94, (uint8_t)243, (uint8_t)103, (uint8_t)55, (uint8_t)209, (uint8_t)65, (uint8_t)96, (uint8_t)237, (uint8_t)47, (uint8_t)189, (uint8_t)25, (uint8_t)167, (uint8_t)137, (uint8_t)13, (uint8_t)169, (uint8_t)136, (uint8_t)159, (uint8_t)73, (uint8_t)236, (uint8_t)174, (uint8_t)102, (uint8_t)212, (uint8_t)31, (uint8_t)3, (uint8_t)170, (uint8_t)175, (uint8_t)185, (uint8_t)57, (uint8_t)190, (uint8_t)106, (uint8_t)65, (uint8_t)190, (uint8_t)193, (uint8_t)76, (uint8_t)196, (uint8_t)141, (uint8_t)206, (uint8_t)108, (uint8_t)53, (uint8_t)235};
            p171_data__SET(&data_, 0, PH.base.pack) ;
        }
        p171_type_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        c_CommunicationChannel_on_DATA64_171(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA96_172(), &PH);
        p172_type_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p172_len_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)232, (uint8_t)214, (uint8_t)173, (uint8_t)36, (uint8_t)237, (uint8_t)70, (uint8_t)63, (uint8_t)228, (uint8_t)89, (uint8_t)220, (uint8_t)34, (uint8_t)165, (uint8_t)13, (uint8_t)253, (uint8_t)136, (uint8_t)79, (uint8_t)198, (uint8_t)79, (uint8_t)1, (uint8_t)160, (uint8_t)83, (uint8_t)200, (uint8_t)224, (uint8_t)26, (uint8_t)169, (uint8_t)75, (uint8_t)249, (uint8_t)86, (uint8_t)67, (uint8_t)0, (uint8_t)203, (uint8_t)175, (uint8_t)206, (uint8_t)8, (uint8_t)81, (uint8_t)104, (uint8_t)165, (uint8_t)140, (uint8_t)77, (uint8_t)245, (uint8_t)100, (uint8_t)238, (uint8_t)223, (uint8_t)24, (uint8_t)96, (uint8_t)19, (uint8_t)57, (uint8_t)108, (uint8_t)16, (uint8_t)69, (uint8_t)229, (uint8_t)245, (uint8_t)0, (uint8_t)26, (uint8_t)71, (uint8_t)95, (uint8_t)243, (uint8_t)55, (uint8_t)193, (uint8_t)110, (uint8_t)196, (uint8_t)14, (uint8_t)104, (uint8_t)112, (uint8_t)94, (uint8_t)57, (uint8_t)241, (uint8_t)164, (uint8_t)91, (uint8_t)36, (uint8_t)169, (uint8_t)49, (uint8_t)248, (uint8_t)107, (uint8_t)189, (uint8_t)95, (uint8_t)12, (uint8_t)85, (uint8_t)222, (uint8_t)222, (uint8_t)228, (uint8_t)193, (uint8_t)139, (uint8_t)71, (uint8_t)59, (uint8_t)37, (uint8_t)16, (uint8_t)144, (uint8_t)237, (uint8_t)70, (uint8_t)189, (uint8_t)214, (uint8_t)25, (uint8_t)188, (uint8_t)76, (uint8_t)251};
            p172_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DATA96_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RANGEFINDER_173(), &PH);
        p173_voltage_SET((float) -1.8790527E38F, PH.base.pack) ;
        p173_distance_SET((float)2.561586E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RANGEFINDER_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AIRSPEED_AUTOCAL_174(), &PH);
        p174_EAS2TAS_SET((float) -1.9346427E38F, PH.base.pack) ;
        p174_ratio_SET((float) -2.1917476E38F, PH.base.pack) ;
        p174_Pcz_SET((float) -2.3267228E38F, PH.base.pack) ;
        p174_vx_SET((float)1.1364388E38F, PH.base.pack) ;
        p174_Pby_SET((float) -1.5351117E38F, PH.base.pack) ;
        p174_Pax_SET((float) -2.5706285E38F, PH.base.pack) ;
        p174_vy_SET((float)2.430164E38F, PH.base.pack) ;
        p174_vz_SET((float)3.0419742E38F, PH.base.pack) ;
        p174_state_z_SET((float)1.981293E38F, PH.base.pack) ;
        p174_state_y_SET((float) -1.4023957E38F, PH.base.pack) ;
        p174_diff_pressure_SET((float) -7.262932E37F, PH.base.pack) ;
        p174_state_x_SET((float)3.111261E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_POINT_175(), &PH);
        p175_lng_SET((int32_t) -844794889, PH.base.pack) ;
        p175_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p175_flags_SET(e_RALLY_FLAGS_FAVORABLE_WIND, PH.base.pack) ;
        p175_break_alt_SET((int16_t)(int16_t) -13976, PH.base.pack) ;
        p175_lat_SET((int32_t)103367418, PH.base.pack) ;
        p175_count_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p175_alt_SET((int16_t)(int16_t) -15879, PH.base.pack) ;
        p175_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p175_land_dir_SET((uint16_t)(uint16_t)48746, PH.base.pack) ;
        p175_idx_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_POINT_175(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RALLY_FETCH_POINT_176(), &PH);
        p176_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p176_idx_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p176_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        c_CommunicationChannel_on_RALLY_FETCH_POINT_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COMPASSMOT_STATUS_177(), &PH);
        p177_interference_SET((uint16_t)(uint16_t)55005, PH.base.pack) ;
        p177_throttle_SET((uint16_t)(uint16_t)31857, PH.base.pack) ;
        p177_CompensationX_SET((float) -1.1624441E38F, PH.base.pack) ;
        p177_CompensationY_SET((float) -9.063226E37F, PH.base.pack) ;
        p177_CompensationZ_SET((float) -3.1142157E38F, PH.base.pack) ;
        p177_current_SET((float) -3.0409922E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMPASSMOT_STATUS_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS2_178(), &PH);
        p178_lng_SET((int32_t)1436887039, PH.base.pack) ;
        p178_pitch_SET((float) -2.3437722E38F, PH.base.pack) ;
        p178_altitude_SET((float)3.3275465E38F, PH.base.pack) ;
        p178_lat_SET((int32_t)2049905234, PH.base.pack) ;
        p178_yaw_SET((float) -2.6190503E37F, PH.base.pack) ;
        p178_roll_SET((float)1.0544186E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS2_178(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_STATUS_179(), &PH);
        p179_cam_idx_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p179_time_usec_SET((uint64_t)2993184925173995351L, PH.base.pack) ;
        p179_img_idx_SET((uint16_t)(uint16_t)20356, PH.base.pack) ;
        p179_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER, PH.base.pack) ;
        p179_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p179_p2_SET((float) -2.9105925E38F, PH.base.pack) ;
        p179_p1_SET((float)5.6325494E37F, PH.base.pack) ;
        p179_p4_SET((float)1.8058774E38F, PH.base.pack) ;
        p179_p3_SET((float)1.3779528E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_STATUS_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_FEEDBACK_180(), &PH);
        p180_foc_len_SET((float) -1.5871107E38F, PH.base.pack) ;
        p180_pitch_SET((float) -1.47088E38F, PH.base.pack) ;
        p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_BADEXPOSURE, PH.base.pack) ;
        p180_yaw_SET((float) -2.1199742E38F, PH.base.pack) ;
        p180_img_idx_SET((uint16_t)(uint16_t)61261, PH.base.pack) ;
        p180_roll_SET((float)2.5728873E38F, PH.base.pack) ;
        p180_lat_SET((int32_t) -938431042, PH.base.pack) ;
        p180_time_usec_SET((uint64_t)8155565993618783924L, PH.base.pack) ;
        p180_alt_rel_SET((float)2.922496E38F, PH.base.pack) ;
        p180_lng_SET((int32_t) -1930845716, PH.base.pack) ;
        p180_cam_idx_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p180_target_system_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p180_alt_msl_SET((float) -2.7157475E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_FEEDBACK_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY2_181(), &PH);
        p181_voltage_SET((uint16_t)(uint16_t)64954, PH.base.pack) ;
        p181_current_battery_SET((int16_t)(int16_t) -24475, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY2_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AHRS3_182(), &PH);
        p182_pitch_SET((float) -2.8939772E38F, PH.base.pack) ;
        p182_v2_SET((float)1.0528232E38F, PH.base.pack) ;
        p182_roll_SET((float)2.0357053E38F, PH.base.pack) ;
        p182_v1_SET((float)2.549625E38F, PH.base.pack) ;
        p182_v4_SET((float) -2.1188601E38F, PH.base.pack) ;
        p182_altitude_SET((float) -4.5911354E37F, PH.base.pack) ;
        p182_lat_SET((int32_t) -2031478790, PH.base.pack) ;
        p182_v3_SET((float) -1.0307818E38F, PH.base.pack) ;
        p182_lng_SET((int32_t)274290649, PH.base.pack) ;
        p182_yaw_SET((float) -2.7070143E38F, PH.base.pack) ;
        c_CommunicationChannel_on_AHRS3_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_REQUEST_183(), &PH);
        p183_target_system_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p183_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_DATA_BLOCK_184(), &PH);
        p184_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p184_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)241, (uint8_t)227, (uint8_t)127, (uint8_t)40, (uint8_t)197, (uint8_t)116, (uint8_t)160, (uint8_t)254, (uint8_t)59, (uint8_t)42, (uint8_t)251, (uint8_t)6, (uint8_t)124, (uint8_t)140, (uint8_t)28, (uint8_t)93, (uint8_t)77, (uint8_t)41, (uint8_t)245, (uint8_t)92, (uint8_t)45, (uint8_t)106, (uint8_t)12, (uint8_t)155, (uint8_t)171, (uint8_t)120, (uint8_t)91, (uint8_t)178, (uint8_t)239, (uint8_t)229, (uint8_t)254, (uint8_t)190, (uint8_t)32, (uint8_t)53, (uint8_t)178, (uint8_t)174, (uint8_t)139, (uint8_t)236, (uint8_t)252, (uint8_t)232, (uint8_t)180, (uint8_t)111, (uint8_t)13, (uint8_t)73, (uint8_t)183, (uint8_t)195, (uint8_t)57, (uint8_t)43, (uint8_t)54, (uint8_t)202, (uint8_t)245, (uint8_t)169, (uint8_t)32, (uint8_t)86, (uint8_t)33, (uint8_t)164, (uint8_t)92, (uint8_t)228, (uint8_t)77, (uint8_t)145, (uint8_t)224, (uint8_t)39, (uint8_t)187, (uint8_t)86, (uint8_t)219, (uint8_t)226, (uint8_t)105, (uint8_t)192, (uint8_t)132, (uint8_t)241, (uint8_t)4, (uint8_t)164, (uint8_t)168, (uint8_t)38, (uint8_t)137, (uint8_t)123, (uint8_t)173, (uint8_t)220, (uint8_t)227, (uint8_t)40, (uint8_t)57, (uint8_t)203, (uint8_t)37, (uint8_t)124, (uint8_t)128, (uint8_t)140, (uint8_t)170, (uint8_t)11, (uint8_t)236, (uint8_t)208, (uint8_t)65, (uint8_t)178, (uint8_t)208, (uint8_t)46, (uint8_t)175, (uint8_t)208, (uint8_t)32, (uint8_t)88, (uint8_t)168, (uint8_t)52, (uint8_t)248, (uint8_t)203, (uint8_t)30, (uint8_t)231, (uint8_t)176, (uint8_t)11, (uint8_t)36, (uint8_t)234, (uint8_t)187, (uint8_t)202, (uint8_t)152, (uint8_t)146, (uint8_t)106, (uint8_t)194, (uint8_t)113, (uint8_t)2, (uint8_t)241, (uint8_t)37, (uint8_t)106, (uint8_t)245, (uint8_t)27, (uint8_t)177, (uint8_t)120, (uint8_t)252, (uint8_t)156, (uint8_t)249, (uint8_t)175, (uint8_t)117, (uint8_t)181, (uint8_t)203, (uint8_t)99, (uint8_t)115, (uint8_t)47, (uint8_t)2, (uint8_t)75, (uint8_t)145, (uint8_t)104, (uint8_t)66, (uint8_t)169, (uint8_t)153, (uint8_t)222, (uint8_t)58, (uint8_t)73, (uint8_t)161, (uint8_t)174, (uint8_t)22, (uint8_t)39, (uint8_t)78, (uint8_t)34, (uint8_t)7, (uint8_t)39, (uint8_t)70, (uint8_t)149, (uint8_t)165, (uint8_t)144, (uint8_t)71, (uint8_t)53, (uint8_t)25, (uint8_t)219, (uint8_t)97, (uint8_t)12, (uint8_t)99, (uint8_t)21, (uint8_t)186, (uint8_t)185, (uint8_t)168, (uint8_t)10, (uint8_t)99, (uint8_t)198, (uint8_t)1, (uint8_t)191, (uint8_t)96, (uint8_t)86, (uint8_t)42, (uint8_t)70, (uint8_t)109, (uint8_t)131, (uint8_t)248, (uint8_t)199, (uint8_t)229, (uint8_t)31, (uint8_t)83, (uint8_t)179, (uint8_t)171, (uint8_t)66, (uint8_t)247, (uint8_t)230, (uint8_t)124, (uint8_t)187, (uint8_t)239, (uint8_t)183, (uint8_t)237, (uint8_t)236, (uint8_t)247, (uint8_t)31, (uint8_t)243, (uint8_t)188, (uint8_t)55, (uint8_t)203, (uint8_t)196};
            p184_data__SET(&data_, 0, PH.base.pack) ;
        }
        p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, PH.base.pack) ;
        c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_REMOTE_LOG_BLOCK_STATUS_185(), &PH);
        p185_seqno_SET((uint32_t)3253929960L, PH.base.pack) ;
        p185_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p185_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_NACK, PH.base.pack) ;
        c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LED_CONTROL_186(), &PH);
        p186_pattern_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p186_custom_len_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p186_instance_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p186_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            uint8_t custom_bytes[] =  {(uint8_t)241, (uint8_t)169, (uint8_t)132, (uint8_t)10, (uint8_t)218, (uint8_t)73, (uint8_t)8, (uint8_t)69, (uint8_t)54, (uint8_t)144, (uint8_t)121, (uint8_t)200, (uint8_t)142, (uint8_t)45, (uint8_t)230, (uint8_t)250, (uint8_t)246, (uint8_t)70, (uint8_t)121, (uint8_t)105, (uint8_t)18, (uint8_t)43, (uint8_t)124, (uint8_t)0};
            p186_custom_bytes_SET(&custom_bytes, 0, PH.base.pack) ;
        }
        p186_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        c_CommunicationChannel_on_LED_CONTROL_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_PROGRESS_191(), &PH);
        p191_completion_pct_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        {
            uint8_t completion_mask[] =  {(uint8_t)112, (uint8_t)217, (uint8_t)77, (uint8_t)144, (uint8_t)97, (uint8_t)229, (uint8_t)91, (uint8_t)124, (uint8_t)33, (uint8_t)183};
            p191_completion_mask_SET(&completion_mask, 0, PH.base.pack) ;
        }
        p191_direction_x_SET((float) -2.8114162E38F, PH.base.pack) ;
        p191_attempt_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p191_compass_id_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p191_direction_y_SET((float) -6.188112E37F, PH.base.pack) ;
        p191_cal_mask_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p191_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, PH.base.pack) ;
        p191_direction_z_SET((float)3.041614E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MAG_CAL_REPORT_192(), &PH);
        p192_offdiag_y_SET((float) -2.857091E38F, PH.base.pack) ;
        p192_cal_mask_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p192_ofs_x_SET((float)9.731662E37F, PH.base.pack) ;
        p192_compass_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p192_fitness_SET((float) -9.897789E37F, PH.base.pack) ;
        p192_diag_x_SET((float) -9.16103E36F, PH.base.pack) ;
        p192_diag_y_SET((float) -2.1910896E38F, PH.base.pack) ;
        p192_autosaved_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p192_ofs_z_SET((float) -1.7167431E38F, PH.base.pack) ;
        p192_offdiag_z_SET((float) -2.5587872E38F, PH.base.pack) ;
        p192_diag_z_SET((float) -1.3230701E38F, PH.base.pack) ;
        p192_ofs_y_SET((float)1.3234556E38F, PH.base.pack) ;
        p192_offdiag_x_SET((float)1.760406E38F, PH.base.pack) ;
        p192_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_NOT_STARTED, PH.base.pack) ;
        c_CommunicationChannel_on_MAG_CAL_REPORT_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EKF_STATUS_REPORT_193(), &PH);
        p193_flags_SET((e_EKF_STATUS_FLAGS_EKF_ATTITUDE |
                        e_EKF_STATUS_FLAGS_EKF_VELOCITY_VERT |
                        e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_REL |
                        e_EKF_STATUS_FLAGS_EKF_CONST_POS_MODE |
                        e_EKF_STATUS_FLAGS_EKF_POS_HORIZ_ABS), PH.base.pack) ;
        p193_pos_vert_variance_SET((float) -2.9666584E37F, PH.base.pack) ;
        p193_terrain_alt_variance_SET((float) -1.540064E38F, PH.base.pack) ;
        p193_compass_variance_SET((float) -1.6123477E38F, PH.base.pack) ;
        p193_pos_horiz_variance_SET((float) -3.1226725E38F, PH.base.pack) ;
        p193_velocity_variance_SET((float)1.5838299E38F, PH.base.pack) ;
        c_CommunicationChannel_on_EKF_STATUS_REPORT_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PID_TUNING_194(), &PH);
        p194_FF_SET((float) -2.8814376E38F, PH.base.pack) ;
        p194_I_SET((float)2.2600286E37F, PH.base.pack) ;
        p194_D_SET((float) -1.7362124E38F, PH.base.pack) ;
        p194_P_SET((float) -1.6272534E38F, PH.base.pack) ;
        p194_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_STEER, PH.base.pack) ;
        p194_desired_SET((float) -3.150835E38F, PH.base.pack) ;
        p194_achieved_SET((float)1.118314E38F, PH.base.pack) ;
        c_CommunicationChannel_on_PID_TUNING_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_REPORT_200(), &PH);
        p200_delta_angle_y_SET((float)1.0392357E38F, PH.base.pack) ;
        p200_delta_velocity_z_SET((float) -2.5588347E38F, PH.base.pack) ;
        p200_delta_angle_x_SET((float)1.5241133E38F, PH.base.pack) ;
        p200_delta_angle_z_SET((float)2.0236332E38F, PH.base.pack) ;
        p200_delta_velocity_y_SET((float)1.739594E38F, PH.base.pack) ;
        p200_delta_velocity_x_SET((float)2.2162705E38F, PH.base.pack) ;
        p200_joint_roll_SET((float) -2.4109295E38F, PH.base.pack) ;
        p200_joint_el_SET((float)2.3766834E38F, PH.base.pack) ;
        p200_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p200_delta_time_SET((float) -1.6660262E38F, PH.base.pack) ;
        p200_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p200_joint_az_SET((float) -3.2481993E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_REPORT_200(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_CONTROL_201(), &PH);
        p201_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p201_demanded_rate_x_SET((float)2.8767014E38F, PH.base.pack) ;
        p201_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p201_demanded_rate_y_SET((float)2.2219242E38F, PH.base.pack) ;
        p201_demanded_rate_z_SET((float) -1.2787064E37F, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_CONTROL_201(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GIMBAL_TORQUE_CMD_REPORT_214(), &PH);
        p214_az_torque_cmd_SET((int16_t)(int16_t) -20845, PH.base.pack) ;
        p214_el_torque_cmd_SET((int16_t)(int16_t)3552, PH.base.pack) ;
        p214_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p214_rl_torque_cmd_SET((int16_t)(int16_t)7137, PH.base.pack) ;
        p214_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_HEARTBEAT_215(), &PH);
        p215_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_DISCONNECTED, PH.base.pack) ;
        p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_TIME_LAPSE, PH.base.pack) ;
        p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_REQUEST_216(), &PH);
        p216_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p216_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p216_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_GET_RESPONSE_217(), &PH);
        {
            uint8_t value[] =  {(uint8_t)201, (uint8_t)219, (uint8_t)171, (uint8_t)159};
            p217_value_SET(&value, 0, PH.base.pack) ;
        }
        p217_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE, PH.base.pack) ;
        p217_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_REQUEST_218(), &PH);
        p218_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p218_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p218_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_GAIN, PH.base.pack) ;
        {
            uint8_t value[] =  {(uint8_t)230, (uint8_t)240, (uint8_t)220, (uint8_t)97};
            p218_value_SET(&value, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GOPRO_SET_RESPONSE_219(), &PH);
        p219_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_VIDEO_SETTINGS, PH.base.pack) ;
        p219_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
        c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RPM_226(), &PH);
        p226_rpm1_SET((float)7.4162626E37F, PH.base.pack) ;
        p226_rpm2_SET((float)1.0434025E38F, PH.base.pack) ;
        c_CommunicationChannel_on_RPM_226(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_ratio_SET((float) -3.298805E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -8.193581E37F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL), PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -1.9911106E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)1.992366E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float)4.1985894E37F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.176877E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -2.4293223E37F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -6.4971383E37F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)6506622306616413683L, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_time_usec_SET((uint64_t)2590716585105715623L, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -2.2426982E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -1.941195E38F, PH.base.pack) ;
        p231_wind_x_SET((float)2.2652934E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -2.8532318E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -1.9095134E38F, PH.base.pack) ;
        p231_wind_z_SET((float) -2.2825828E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -2.657095E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.1495783E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_hdop_SET((float)7.80131E37F, PH.base.pack) ;
        p232_ve_SET((float) -2.4769426E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p232_alt_SET((float)2.0383698E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)444591517L, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)39986, PH.base.pack) ;
        p232_lat_SET((int32_t)617627158, PH.base.pack) ;
        p232_vd_SET((float)1.6731291E38F, PH.base.pack) ;
        p232_vn_SET((float)5.155052E37F, PH.base.pack) ;
        p232_lon_SET((int32_t)816464614, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY), PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1232513044434789164L, PH.base.pack) ;
        p232_vert_accuracy_SET((float)2.8893491E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)2.9227922E38F, PH.base.pack) ;
        p232_vdop_SET((float)1.834544E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.7083424E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)249, (uint8_t)57, (uint8_t)73, (uint8_t)236, (uint8_t)67, (uint8_t)30, (uint8_t)88, (uint8_t)108, (uint8_t)85, (uint8_t)28, (uint8_t)20, (uint8_t)87, (uint8_t)0, (uint8_t)121, (uint8_t)2, (uint8_t)3, (uint8_t)37, (uint8_t)97, (uint8_t)213, (uint8_t)124, (uint8_t)81, (uint8_t)160, (uint8_t)151, (uint8_t)173, (uint8_t)38, (uint8_t)37, (uint8_t)112, (uint8_t)218, (uint8_t)209, (uint8_t)125, (uint8_t)170, (uint8_t)105, (uint8_t)76, (uint8_t)55, (uint8_t)52, (uint8_t)249, (uint8_t)60, (uint8_t)100, (uint8_t)154, (uint8_t)195, (uint8_t)68, (uint8_t)140, (uint8_t)127, (uint8_t)215, (uint8_t)64, (uint8_t)98, (uint8_t)18, (uint8_t)191, (uint8_t)231, (uint8_t)228, (uint8_t)17, (uint8_t)180, (uint8_t)241, (uint8_t)97, (uint8_t)82, (uint8_t)216, (uint8_t)230, (uint8_t)187, (uint8_t)223, (uint8_t)219, (uint8_t)202, (uint8_t)118, (uint8_t)145, (uint8_t)12, (uint8_t)176, (uint8_t)68, (uint8_t)181, (uint8_t)33, (uint8_t)98, (uint8_t)215, (uint8_t)117, (uint8_t)59, (uint8_t)124, (uint8_t)0, (uint8_t)43, (uint8_t)251, (uint8_t)1, (uint8_t)137, (uint8_t)35, (uint8_t)163, (uint8_t)167, (uint8_t)15, (uint8_t)75, (uint8_t)253, (uint8_t)175, (uint8_t)77, (uint8_t)232, (uint8_t)18, (uint8_t)186, (uint8_t)49, (uint8_t)52, (uint8_t)66, (uint8_t)46, (uint8_t)67, (uint8_t)54, (uint8_t)107, (uint8_t)29, (uint8_t)60, (uint8_t)202, (uint8_t)34, (uint8_t)206, (uint8_t)159, (uint8_t)249, (uint8_t)29, (uint8_t)4, (uint8_t)57, (uint8_t)29, (uint8_t)105, (uint8_t)35, (uint8_t)212, (uint8_t)188, (uint8_t)13, (uint8_t)5, (uint8_t)200, (uint8_t)65, (uint8_t)162, (uint8_t)128, (uint8_t)19, (uint8_t)59, (uint8_t)197, (uint8_t)230, (uint8_t)75, (uint8_t)240, (uint8_t)78, (uint8_t)174, (uint8_t)36, (uint8_t)79, (uint8_t)73, (uint8_t)16, (uint8_t)222, (uint8_t)91, (uint8_t)156, (uint8_t)234, (uint8_t)188, (uint8_t)45, (uint8_t)212, (uint8_t)138, (uint8_t)111, (uint8_t)133, (uint8_t)70, (uint8_t)120, (uint8_t)13, (uint8_t)160, (uint8_t)248, (uint8_t)82, (uint8_t)127, (uint8_t)84, (uint8_t)188, (uint8_t)231, (uint8_t)75, (uint8_t)111, (uint8_t)84, (uint8_t)174, (uint8_t)139, (uint8_t)144, (uint8_t)145, (uint8_t)129, (uint8_t)25, (uint8_t)14, (uint8_t)244, (uint8_t)80, (uint8_t)75, (uint8_t)60, (uint8_t)143, (uint8_t)53, (uint8_t)67, (uint8_t)158, (uint8_t)36, (uint8_t)245, (uint8_t)129, (uint8_t)154, (uint8_t)224, (uint8_t)48, (uint8_t)243, (uint8_t)92, (uint8_t)183, (uint8_t)184, (uint8_t)105, (uint8_t)35, (uint8_t)185};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
        p234_wp_distance_SET((uint16_t)(uint16_t)56236, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)12132, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -82, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)40, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)15791, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -9627, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)30972, PH.base.pack) ;
        p234_longitude_SET((int32_t)1148647161, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)42, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -7, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1808589629L, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)19873, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED), PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p234_latitude_SET((int32_t)587638534, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)7648, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)1005456643005828106L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.6855466E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3466782382L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)774564120L, PH.base.pack) ;
        p241_vibration_z_SET((float)2.8700246E38F, PH.base.pack) ;
        p241_vibration_x_SET((float) -2.7269454E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3889224229L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {-1.9081152E37F, 6.5504897E37F, 8.051274E37F, -1.3140046E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_x_SET((float) -3.2759329E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1974913983, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)8978235579722084327L, &PH) ;
        p242_approach_x_SET((float) -8.0141444E37F, PH.base.pack) ;
        p242_longitude_SET((int32_t) -1695465939, PH.base.pack) ;
        p242_z_SET((float)3.31791E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1456905929, PH.base.pack) ;
        p242_y_SET((float) -1.115745E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -1.9842188E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.4935021E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_longitude_SET((int32_t)1854400914, PH.base.pack) ;
        p243_approach_y_SET((float) -1.1507413E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)683515314, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)6595750134801838430L, &PH) ;
        p243_latitude_SET((int32_t)291712217, PH.base.pack) ;
        p243_x_SET((float)2.462426E38F, PH.base.pack) ;
        p243_y_SET((float)1.695032E38F, PH.base.pack) ;
        p243_approach_x_SET((float)3.1214488E38F, PH.base.pack) ;
        p243_approach_z_SET((float)8.710898E37F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p243_z_SET((float)1.5431788E38F, PH.base.pack) ;
        {
            float q[] =  {1.278645E38F, 7.362372E37F, -1.4122965E38F, 2.7135035E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)58349, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -1833239760, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_ICAO_address_SET((uint32_t)2519749146L, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)62136, PH.base.pack) ;
        p246_lat_SET((int32_t) -1612996993, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)5978, PH.base.pack) ;
        p246_lon_SET((int32_t)1132951292, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED), PH.base.pack) ;
        p246_altitude_SET((int32_t)615650320, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -32193, PH.base.pack) ;
        {
            char16_t* callsign = u"iarwn";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_tslc_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)46408, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
        p247_time_to_minimum_delta_SET((float) -2.9761791E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -9.669867E37F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_id_SET((uint32_t)2913219049L, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)1.2449657E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
        p248_message_type_SET((uint16_t)(uint16_t)22618, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)215, (uint8_t)16, (uint8_t)91, (uint8_t)39, (uint8_t)3, (uint8_t)98, (uint8_t)222, (uint8_t)153, (uint8_t)63, (uint8_t)195, (uint8_t)155, (uint8_t)102, (uint8_t)31, (uint8_t)125, (uint8_t)87, (uint8_t)38, (uint8_t)166, (uint8_t)22, (uint8_t)228, (uint8_t)136, (uint8_t)243, (uint8_t)254, (uint8_t)138, (uint8_t)46, (uint8_t)97, (uint8_t)44, (uint8_t)161, (uint8_t)116, (uint8_t)164, (uint8_t)182, (uint8_t)153, (uint8_t)100, (uint8_t)251, (uint8_t)255, (uint8_t)116, (uint8_t)96, (uint8_t)127, (uint8_t)134, (uint8_t)176, (uint8_t)222, (uint8_t)85, (uint8_t)183, (uint8_t)231, (uint8_t)199, (uint8_t)143, (uint8_t)77, (uint8_t)161, (uint8_t)122, (uint8_t)50, (uint8_t)213, (uint8_t)252, (uint8_t)177, (uint8_t)38, (uint8_t)11, (uint8_t)118, (uint8_t)222, (uint8_t)248, (uint8_t)102, (uint8_t)143, (uint8_t)56, (uint8_t)244, (uint8_t)9, (uint8_t)193, (uint8_t)175, (uint8_t)173, (uint8_t)239, (uint8_t)77, (uint8_t)78, (uint8_t)103, (uint8_t)125, (uint8_t)17, (uint8_t)203, (uint8_t)22, (uint8_t)68, (uint8_t)248, (uint8_t)241, (uint8_t)237, (uint8_t)86, (uint8_t)203, (uint8_t)147, (uint8_t)138, (uint8_t)33, (uint8_t)211, (uint8_t)231, (uint8_t)57, (uint8_t)144, (uint8_t)206, (uint8_t)211, (uint8_t)149, (uint8_t)163, (uint8_t)111, (uint8_t)180, (uint8_t)196, (uint8_t)41, (uint8_t)183, (uint8_t)88, (uint8_t)138, (uint8_t)38, (uint8_t)111, (uint8_t)141, (uint8_t)212, (uint8_t)86, (uint8_t)175, (uint8_t)187, (uint8_t)189, (uint8_t)138, (uint8_t)243, (uint8_t)174, (uint8_t)53, (uint8_t)187, (uint8_t)146, (uint8_t)136, (uint8_t)106, (uint8_t)188, (uint8_t)227, (uint8_t)178, (uint8_t)36, (uint8_t)93, (uint8_t)72, (uint8_t)194, (uint8_t)49, (uint8_t)193, (uint8_t)31, (uint8_t)176, (uint8_t)203, (uint8_t)92, (uint8_t)206, (uint8_t)169, (uint8_t)218, (uint8_t)50, (uint8_t)67, (uint8_t)8, (uint8_t)22, (uint8_t)122, (uint8_t)6, (uint8_t)156, (uint8_t)103, (uint8_t)200, (uint8_t)133, (uint8_t)208, (uint8_t)194, (uint8_t)95, (uint8_t)73, (uint8_t)211, (uint8_t)158, (uint8_t)236, (uint8_t)243, (uint8_t)201, (uint8_t)193, (uint8_t)136, (uint8_t)168, (uint8_t)49, (uint8_t)234, (uint8_t)221, (uint8_t)121, (uint8_t)50, (uint8_t)186, (uint8_t)48, (uint8_t)13, (uint8_t)225, (uint8_t)76, (uint8_t)88, (uint8_t)166, (uint8_t)136, (uint8_t)180, (uint8_t)176, (uint8_t)225, (uint8_t)207, (uint8_t)220, (uint8_t)148, (uint8_t)213, (uint8_t)22, (uint8_t)6, (uint8_t)252, (uint8_t)172, (uint8_t)39, (uint8_t)196, (uint8_t)242, (uint8_t)99, (uint8_t)40, (uint8_t)149, (uint8_t)147, (uint8_t)6, (uint8_t)59, (uint8_t)154, (uint8_t)177, (uint8_t)255, (uint8_t)233, (uint8_t)199, (uint8_t)10, (uint8_t)112, (uint8_t)238, (uint8_t)5, (uint8_t)97, (uint8_t)91, (uint8_t)228, (uint8_t)96, (uint8_t)12, (uint8_t)95, (uint8_t)68, (uint8_t)123, (uint8_t)116, (uint8_t)95, (uint8_t)151, (uint8_t)161, (uint8_t)129, (uint8_t)168, (uint8_t)214, (uint8_t)239, (uint8_t)88, (uint8_t)182, (uint8_t)255, (uint8_t)165, (uint8_t)175, (uint8_t)102, (uint8_t)47, (uint8_t)204, (uint8_t)99, (uint8_t)56, (uint8_t)162, (uint8_t)246, (uint8_t)255, (uint8_t)193, (uint8_t)11, (uint8_t)214, (uint8_t)17, (uint8_t)3, (uint8_t)118, (uint8_t)82, (uint8_t)179, (uint8_t)67, (uint8_t)20, (uint8_t)55, (uint8_t)29, (uint8_t)103, (uint8_t)26, (uint8_t)15, (uint8_t)202, (uint8_t)27, (uint8_t)11, (uint8_t)120, (uint8_t)143, (uint8_t)42, (uint8_t)142, (uint8_t)147, (uint8_t)221, (uint8_t)87, (uint8_t)10, (uint8_t)191};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -48, (int8_t) -119, (int8_t) -47, (int8_t) -117, (int8_t)63, (int8_t)54, (int8_t) -14, (int8_t) -39, (int8_t)53, (int8_t) -76, (int8_t)64, (int8_t)3, (int8_t) -39, (int8_t) -77, (int8_t)51, (int8_t) -15, (int8_t) -74, (int8_t)123, (int8_t)23, (int8_t) -8, (int8_t)61, (int8_t) -50, (int8_t) -108, (int8_t) -127, (int8_t) -2, (int8_t) -121, (int8_t)59, (int8_t) -106, (int8_t)88, (int8_t) -104, (int8_t) -84, (int8_t)120};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)58513, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float)1.2399032E38F, PH.base.pack) ;
        p250_z_SET((float) -1.5535985E38F, PH.base.pack) ;
        p250_y_SET((float)3.4171162E37F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)5334953935723241872L, PH.base.pack) ;
        {
            char16_t* name = u"pizwgi";
            p250_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)3568728030L, PH.base.pack) ;
        p251_value_SET((float) -1.834151E38F, PH.base.pack) ;
        {
            char16_t* name = u"rah";
            p251_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)1478118449, PH.base.pack) ;
        {
            char16_t* name = u"zhRjz";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)1479923066L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
        {
            char16_t* text = u"ixdnitKbpNZlhxjneyVmxBrjpgsrnDjdkpvcnwuo";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
        p254_value_SET((float)8.0777536E37F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)630124574L, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)111944636187843693L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)85, (uint8_t)201, (uint8_t)226, (uint8_t)152, (uint8_t)167, (uint8_t)46, (uint8_t)146, (uint8_t)136, (uint8_t)168, (uint8_t)136, (uint8_t)21, (uint8_t)105, (uint8_t)47, (uint8_t)185, (uint8_t)0, (uint8_t)197, (uint8_t)225, (uint8_t)182, (uint8_t)14, (uint8_t)138, (uint8_t)168, (uint8_t)210, (uint8_t)97, (uint8_t)237, (uint8_t)58, (uint8_t)19, (uint8_t)194, (uint8_t)241, (uint8_t)2, (uint8_t)73, (uint8_t)160, (uint8_t)111};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)1667509278L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)2098271176L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p258_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        {
            char16_t* tune = u"Dy";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_v_SET((uint16_t)(uint16_t)16769, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"mcnfxtYvzuyuUcpjtmq";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)50341, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE), PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)1868413401L, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)223, (uint8_t)82, (uint8_t)186, (uint8_t)196, (uint8_t)75, (uint8_t)10, (uint8_t)91, (uint8_t)2, (uint8_t)107, (uint8_t)160, (uint8_t)92, (uint8_t)96, (uint8_t)164, (uint8_t)174, (uint8_t)101, (uint8_t)84, (uint8_t)55, (uint8_t)254, (uint8_t)63, (uint8_t)64, (uint8_t)66, (uint8_t)216, (uint8_t)80, (uint8_t)211, (uint8_t)76, (uint8_t)101, (uint8_t)29, (uint8_t)5, (uint8_t)149, (uint8_t)159, (uint8_t)87, (uint8_t)202};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_h_SET((float)4.708464E37F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)36593, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)233, (uint8_t)232, (uint8_t)236, (uint8_t)242, (uint8_t)201, (uint8_t)178, (uint8_t)19, (uint8_t)114, (uint8_t)109, (uint8_t)172, (uint8_t)12, (uint8_t)84, (uint8_t)164, (uint8_t)80, (uint8_t)78, (uint8_t)110, (uint8_t)89, (uint8_t)103, (uint8_t)25, (uint8_t)2, (uint8_t)194, (uint8_t)91, (uint8_t)244, (uint8_t)113, (uint8_t)236, (uint8_t)128, (uint8_t)37, (uint8_t)155, (uint8_t)202, (uint8_t)55, (uint8_t)49, (uint8_t)159};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)2.0366379E38F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1064213378L, PH.base.pack) ;
        p259_focal_length_SET((float)1.6628022E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)1187317329L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float)1.2498824E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p261_write_speed_SET((float)1.0586661E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)2.7667637E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -2.4270088E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1638140553L, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p261_total_capacity_SET((float) -1.8929161E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_time_boot_ms_SET((uint32_t)4244118059L, PH.base.pack) ;
        p262_available_capacity_SET((float)1.1505174E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p262_image_interval_SET((float) -6.954145E36F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2450410825L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)799257849446727705L, PH.base.pack) ;
        p263_lon_SET((int32_t) -806503323, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1081433290L, PH.base.pack) ;
        {
            float q[] =  {-8.0872457E37F, 4.4570255E37F, 1.0402474E38F, 1.1138858E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_image_index_SET((int32_t) -2020076572, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p263_lat_SET((int32_t) -1345415774, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -1198654167, PH.base.pack) ;
        p263_alt_SET((int32_t)1254304400, PH.base.pack) ;
        {
            char16_t* file_url = u"cfuvcrfvwdflmlusivighGycgaKjaByhhvybsxcfrkoonodwkjnglizfjykrbtmussjemuvcsiMjjsuxjLxqrar";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_capture_result_SET((int8_t)(int8_t)57, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)7492000588839895190L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2006634689L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)3995619622899308902L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)4852189231299334848L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float)3.7231905E37F, PH.base.pack) ;
        p265_roll_SET((float)2.6047674E38F, PH.base.pack) ;
        p265_pitch_SET((float)1.1071837E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)2003472730L, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)245, (uint8_t)93, (uint8_t)143, (uint8_t)107, (uint8_t)188, (uint8_t)207, (uint8_t)137, (uint8_t)18, (uint8_t)181, (uint8_t)2, (uint8_t)71, (uint8_t)169, (uint8_t)249, (uint8_t)187, (uint8_t)246, (uint8_t)203, (uint8_t)72, (uint8_t)61, (uint8_t)9, (uint8_t)174, (uint8_t)221, (uint8_t)142, (uint8_t)206, (uint8_t)104, (uint8_t)181, (uint8_t)162, (uint8_t)149, (uint8_t)145, (uint8_t)55, (uint8_t)215, (uint8_t)60, (uint8_t)204, (uint8_t)217, (uint8_t)254, (uint8_t)252, (uint8_t)133, (uint8_t)254, (uint8_t)166, (uint8_t)120, (uint8_t)205, (uint8_t)41, (uint8_t)47, (uint8_t)227, (uint8_t)121, (uint8_t)7, (uint8_t)89, (uint8_t)7, (uint8_t)202, (uint8_t)226, (uint8_t)218, (uint8_t)37, (uint8_t)128, (uint8_t)138, (uint8_t)210, (uint8_t)138, (uint8_t)78, (uint8_t)235, (uint8_t)183, (uint8_t)199, (uint8_t)103, (uint8_t)168, (uint8_t)26, (uint8_t)134, (uint8_t)200, (uint8_t)123, (uint8_t)31, (uint8_t)192, (uint8_t)207, (uint8_t)177, (uint8_t)80, (uint8_t)176, (uint8_t)73, (uint8_t)181, (uint8_t)111, (uint8_t)61, (uint8_t)10, (uint8_t)196, (uint8_t)104, (uint8_t)38, (uint8_t)255, (uint8_t)147, (uint8_t)79, (uint8_t)222, (uint8_t)80, (uint8_t)0, (uint8_t)197, (uint8_t)251, (uint8_t)190, (uint8_t)132, (uint8_t)173, (uint8_t)234, (uint8_t)155, (uint8_t)84, (uint8_t)91, (uint8_t)72, (uint8_t)228, (uint8_t)184, (uint8_t)155, (uint8_t)125, (uint8_t)245, (uint8_t)150, (uint8_t)218, (uint8_t)124, (uint8_t)236, (uint8_t)111, (uint8_t)9, (uint8_t)236, (uint8_t)148, (uint8_t)194, (uint8_t)201, (uint8_t)96, (uint8_t)21, (uint8_t)87, (uint8_t)103, (uint8_t)199, (uint8_t)107, (uint8_t)51, (uint8_t)255, (uint8_t)81, (uint8_t)136, (uint8_t)130, (uint8_t)27, (uint8_t)145, (uint8_t)108, (uint8_t)96, (uint8_t)174, (uint8_t)203, (uint8_t)155, (uint8_t)188, (uint8_t)60, (uint8_t)16, (uint8_t)62, (uint8_t)145, (uint8_t)10, (uint8_t)205, (uint8_t)7, (uint8_t)46, (uint8_t)202, (uint8_t)135, (uint8_t)162, (uint8_t)80, (uint8_t)99, (uint8_t)18, (uint8_t)221, (uint8_t)186, (uint8_t)221, (uint8_t)153, (uint8_t)159, (uint8_t)189, (uint8_t)13, (uint8_t)21, (uint8_t)50, (uint8_t)73, (uint8_t)153, (uint8_t)205, (uint8_t)15, (uint8_t)29, (uint8_t)22, (uint8_t)40, (uint8_t)215, (uint8_t)50, (uint8_t)253, (uint8_t)45, (uint8_t)172, (uint8_t)195, (uint8_t)72, (uint8_t)228, (uint8_t)251, (uint8_t)117, (uint8_t)215, (uint8_t)110, (uint8_t)43, (uint8_t)41, (uint8_t)113, (uint8_t)83, (uint8_t)12, (uint8_t)203, (uint8_t)141, (uint8_t)116, (uint8_t)196, (uint8_t)111, (uint8_t)79, (uint8_t)95, (uint8_t)238, (uint8_t)70, (uint8_t)1, (uint8_t)144, (uint8_t)203, (uint8_t)215, (uint8_t)251, (uint8_t)250, (uint8_t)204, (uint8_t)224, (uint8_t)163, (uint8_t)77, (uint8_t)172, (uint8_t)4, (uint8_t)104, (uint8_t)87, (uint8_t)168, (uint8_t)140, (uint8_t)131, (uint8_t)193, (uint8_t)97, (uint8_t)49, (uint8_t)190, (uint8_t)248, (uint8_t)187, (uint8_t)155, (uint8_t)10, (uint8_t)117, (uint8_t)223, (uint8_t)31, (uint8_t)206, (uint8_t)34, (uint8_t)206, (uint8_t)17, (uint8_t)167, (uint8_t)87, (uint8_t)31, (uint8_t)146, (uint8_t)3, (uint8_t)251, (uint8_t)4, (uint8_t)25, (uint8_t)240, (uint8_t)109, (uint8_t)41, (uint8_t)188, (uint8_t)77, (uint8_t)191, (uint8_t)60, (uint8_t)66, (uint8_t)179, (uint8_t)107, (uint8_t)83, (uint8_t)255, (uint8_t)130, (uint8_t)211, (uint8_t)219, (uint8_t)128, (uint8_t)181, (uint8_t)147, (uint8_t)107, (uint8_t)115, (uint8_t)19, (uint8_t)178, (uint8_t)231, (uint8_t)241};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_first_message_offset_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)34314, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)13, (uint8_t)125, (uint8_t)148, (uint8_t)166, (uint8_t)164, (uint8_t)252, (uint8_t)218, (uint8_t)232, (uint8_t)9, (uint8_t)122, (uint8_t)112, (uint8_t)13, (uint8_t)185, (uint8_t)48, (uint8_t)2, (uint8_t)56, (uint8_t)136, (uint8_t)183, (uint8_t)195, (uint8_t)7, (uint8_t)138, (uint8_t)149, (uint8_t)80, (uint8_t)131, (uint8_t)151, (uint8_t)39, (uint8_t)232, (uint8_t)240, (uint8_t)69, (uint8_t)162, (uint8_t)30, (uint8_t)9, (uint8_t)219, (uint8_t)114, (uint8_t)135, (uint8_t)71, (uint8_t)85, (uint8_t)20, (uint8_t)168, (uint8_t)252, (uint8_t)143, (uint8_t)173, (uint8_t)246, (uint8_t)76, (uint8_t)138, (uint8_t)92, (uint8_t)2, (uint8_t)190, (uint8_t)41, (uint8_t)134, (uint8_t)156, (uint8_t)91, (uint8_t)160, (uint8_t)220, (uint8_t)58, (uint8_t)82, (uint8_t)145, (uint8_t)216, (uint8_t)224, (uint8_t)125, (uint8_t)191, (uint8_t)109, (uint8_t)166, (uint8_t)174, (uint8_t)231, (uint8_t)29, (uint8_t)238, (uint8_t)240, (uint8_t)37, (uint8_t)99, (uint8_t)38, (uint8_t)178, (uint8_t)153, (uint8_t)121, (uint8_t)173, (uint8_t)188, (uint8_t)7, (uint8_t)170, (uint8_t)149, (uint8_t)52, (uint8_t)137, (uint8_t)249, (uint8_t)240, (uint8_t)180, (uint8_t)214, (uint8_t)106, (uint8_t)199, (uint8_t)91, (uint8_t)209, (uint8_t)111, (uint8_t)217, (uint8_t)7, (uint8_t)28, (uint8_t)69, (uint8_t)211, (uint8_t)78, (uint8_t)210, (uint8_t)58, (uint8_t)31, (uint8_t)245, (uint8_t)162, (uint8_t)31, (uint8_t)211, (uint8_t)209, (uint8_t)238, (uint8_t)115, (uint8_t)73, (uint8_t)8, (uint8_t)154, (uint8_t)151, (uint8_t)83, (uint8_t)68, (uint8_t)110, (uint8_t)34, (uint8_t)192, (uint8_t)71, (uint8_t)139, (uint8_t)166, (uint8_t)98, (uint8_t)243, (uint8_t)138, (uint8_t)56, (uint8_t)148, (uint8_t)161, (uint8_t)97, (uint8_t)64, (uint8_t)97, (uint8_t)92, (uint8_t)40, (uint8_t)151, (uint8_t)104, (uint8_t)165, (uint8_t)100, (uint8_t)244, (uint8_t)17, (uint8_t)207, (uint8_t)23, (uint8_t)32, (uint8_t)97, (uint8_t)8, (uint8_t)54, (uint8_t)243, (uint8_t)34, (uint8_t)248, (uint8_t)76, (uint8_t)223, (uint8_t)191, (uint8_t)95, (uint8_t)23, (uint8_t)67, (uint8_t)230, (uint8_t)33, (uint8_t)67, (uint8_t)169, (uint8_t)106, (uint8_t)137, (uint8_t)114, (uint8_t)86, (uint8_t)24, (uint8_t)48, (uint8_t)74, (uint8_t)201, (uint8_t)76, (uint8_t)98, (uint8_t)52, (uint8_t)29, (uint8_t)250, (uint8_t)102, (uint8_t)67, (uint8_t)21, (uint8_t)66, (uint8_t)65, (uint8_t)219, (uint8_t)96, (uint8_t)227, (uint8_t)152, (uint8_t)21, (uint8_t)0, (uint8_t)11, (uint8_t)121, (uint8_t)43, (uint8_t)95, (uint8_t)149, (uint8_t)233, (uint8_t)35, (uint8_t)134, (uint8_t)139, (uint8_t)123, (uint8_t)44, (uint8_t)28, (uint8_t)137, (uint8_t)95, (uint8_t)116, (uint8_t)250, (uint8_t)75, (uint8_t)159, (uint8_t)241, (uint8_t)69, (uint8_t)67, (uint8_t)198, (uint8_t)225, (uint8_t)235, (uint8_t)60, (uint8_t)77, (uint8_t)50, (uint8_t)124, (uint8_t)90, (uint8_t)209, (uint8_t)61, (uint8_t)218, (uint8_t)24, (uint8_t)31, (uint8_t)125, (uint8_t)117, (uint8_t)27, (uint8_t)162, (uint8_t)128, (uint8_t)96, (uint8_t)81, (uint8_t)203, (uint8_t)57, (uint8_t)86, (uint8_t)122, (uint8_t)0, (uint8_t)106, (uint8_t)30, (uint8_t)10, (uint8_t)248, (uint8_t)66, (uint8_t)89, (uint8_t)114, (uint8_t)174, (uint8_t)102, (uint8_t)77, (uint8_t)229, (uint8_t)6, (uint8_t)4, (uint8_t)234, (uint8_t)137, (uint8_t)87, (uint8_t)202, (uint8_t)103, (uint8_t)101, (uint8_t)46, (uint8_t)208, (uint8_t)194, (uint8_t)169, (uint8_t)189, (uint8_t)164};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)11590, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)62928, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        {
            char16_t* uri = u"xhqzmBnQZrvrgypzyPlmMusQuKuhipljlklrqzcziujlsdntuktvxntihfzanjcuhgxwccbcukrlqFtXllnykjWcvzjgwjigxEoovtrlJwzeMiuvjFxGgwvgxtpjtrfwsbjjNxfvlftlaufwKsiwxdrX";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_rotation_SET((uint16_t)(uint16_t)25987, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)15619, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)46077, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)2002045447L, PH.base.pack) ;
        p269_framerate_SET((float)1.6705992E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_h_SET((uint16_t)(uint16_t)29389, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)17957, PH.base.pack) ;
        p270_framerate_SET((float) -1.5060466E38F, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)3045996843L, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)49777, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        {
            char16_t* uri = u"wnwbkhgxniqwxtkZxqxtuDsxydgxzmbjGgygnahgmdjqpsfhjtzehtQzoInvcrivpstlvqgebpmryrsdXncvsbarfqpfvglxddwghqpuoVrbxcwQkfyywvatqmbxyXemcHdrdjWy";
            p270_uri_SET_(uri, &PH) ;
        }
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"gmwvntbPyjokiRbqunr";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"nqzjpyFYeOtQwuyaifrOdjmhPxyvwMJiTiXeyynljzaCceOgIdpjKdasqyemecrx";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        {
            uint8_t library_version_hash[] =  {(uint8_t)23, (uint8_t)95, (uint8_t)47, (uint8_t)213, (uint8_t)19, (uint8_t)240, (uint8_t)79, (uint8_t)239};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)28235, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)233, (uint8_t)166, (uint8_t)77, (uint8_t)127, (uint8_t)230, (uint8_t)225, (uint8_t)169, (uint8_t)177};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)61060, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)50852, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)52483073L, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)1249056681501338864L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)15797, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            char16_t* name = u"ujbifrhEimhe";
            p311_name_SET_(name, &PH) ;
        }
        p311_time_usec_SET((uint64_t)2579408316857718861L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)2689980859L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)41, (uint8_t)184, (uint8_t)192, (uint8_t)32, (uint8_t)93, (uint8_t)110, (uint8_t)77, (uint8_t)92, (uint8_t)19, (uint8_t)195, (uint8_t)210, (uint8_t)209, (uint8_t)23, (uint8_t)88, (uint8_t)153, (uint8_t)201};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2502447664L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"CazPbpzjbblsaw";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t) -21318, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)17048, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        {
            char16_t* param_id = u"hodn";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)25035, PH.base.pack) ;
        {
            char16_t* param_value = u"ukfqdvohcwSzgdqwqjoDmtkltigxd";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        {
            char16_t* param_id = u"Cs";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_value = u"zqoeoutfctitol";
            p323_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"hevbgnbysXaAls";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        {
            char16_t* param_value = u"badfmsscaelmvfrpzIwdthRertpfwgqnstukadTzxayKbdcnjauhnbwikqovqwaoippnccMlzvsqtarjezdnxejbfthzocbepdweHzkfslxfblmwnxmavfPscdkRd";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_increment_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)62940, (uint16_t)18996, (uint16_t)50926, (uint16_t)24201, (uint16_t)36839, (uint16_t)55368, (uint16_t)9105, (uint16_t)26552, (uint16_t)59620, (uint16_t)13011, (uint16_t)10202, (uint16_t)40069, (uint16_t)37951, (uint16_t)11260, (uint16_t)44854, (uint16_t)36107, (uint16_t)58298, (uint16_t)34312, (uint16_t)7631, (uint16_t)7040, (uint16_t)34000, (uint16_t)26094, (uint16_t)22093, (uint16_t)20172, (uint16_t)33375, (uint16_t)4511, (uint16_t)2886, (uint16_t)15677, (uint16_t)16595, (uint16_t)47338, (uint16_t)57578, (uint16_t)30970, (uint16_t)36337, (uint16_t)33966, (uint16_t)14399, (uint16_t)23100, (uint16_t)57701, (uint16_t)50056, (uint16_t)15357, (uint16_t)31833, (uint16_t)18077, (uint16_t)57105, (uint16_t)60735, (uint16_t)59711, (uint16_t)35235, (uint16_t)48014, (uint16_t)58386, (uint16_t)1664, (uint16_t)7651, (uint16_t)10619, (uint16_t)54096, (uint16_t)35177, (uint16_t)60871, (uint16_t)13344, (uint16_t)42868, (uint16_t)52215, (uint16_t)3177, (uint16_t)27621, (uint16_t)3906, (uint16_t)21040, (uint16_t)57337, (uint16_t)59895, (uint16_t)10615, (uint16_t)36507, (uint16_t)39817, (uint16_t)31648, (uint16_t)19072, (uint16_t)6437, (uint16_t)15040, (uint16_t)35938, (uint16_t)49728, (uint16_t)675};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_min_distance_SET((uint16_t)(uint16_t)13594, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)50564, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)4988847116169182049L, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
        {
            char16_t* callsign = u"zqz";
            p10001_callsign_SET_(callsign, &PH) ;
        }
        p10001_ICAO_SET((uint32_t)2633229907L, PH.base.pack) ;
        p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE, PH.base.pack) ;
        p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, PH.base.pack) ;
        p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M, PH.base.pack) ;
        p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M, PH.base.pack) ;
        p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA, PH.base.pack) ;
        p10001_stallSpeed_SET((uint16_t)(uint16_t)38609, PH.base.pack) ;
        c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
        p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_NO_EMERGENCY, PH.base.pack) ;
        p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1, PH.base.pack) ;
        p10002_accuracyVel_SET((uint16_t)(uint16_t)13783, PH.base.pack) ;
        p10002_accuracyHor_SET((uint32_t)2618932642L, PH.base.pack) ;
        p10002_velNS_SET((int16_t)(int16_t) -12356, PH.base.pack) ;
        p10002_numSats_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p10002_gpsLon_SET((int32_t)205261065, PH.base.pack) ;
        p10002_utcTime_SET((uint32_t)4100021455L, PH.base.pack) ;
        p10002_gpsAlt_SET((int32_t)305936887, PH.base.pack) ;
        p10002_squawk_SET((uint16_t)(uint16_t)38837, PH.base.pack) ;
        p10002_accuracyVert_SET((uint16_t)(uint16_t)4618, PH.base.pack) ;
        p10002_gpsLat_SET((int32_t)1253127399, PH.base.pack) ;
        p10002_velVert_SET((int16_t)(int16_t)13642, PH.base.pack) ;
        p10002_baroAltMSL_SET((int32_t) -235367561, PH.base.pack) ;
        p10002_state_SET((e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND), PH.base.pack) ;
        p10002_VelEW_SET((int16_t)(int16_t)8272, PH.base.pack) ;
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
        p11000_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p11000_request_id_SET((uint32_t)2077538683L, PH.base.pack) ;
        p11000_count_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p11000_bus_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        {
            char16_t* busname = u"bnaqBvUzqworpcyvinqkxakmjcpeyavrqob";
            p11000_busname_SET_(busname, &PH) ;
        }
        p11000_regstart_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p11000_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p11000_address_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p11000_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_READ_11000(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_READ_REPLY_11001(), &PH);
        p11001_count_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p11001_request_id_SET((uint32_t)1126125765L, PH.base.pack) ;
        p11001_regstart_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p11001_result_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)47, (uint8_t)44, (uint8_t)230, (uint8_t)19, (uint8_t)24, (uint8_t)145, (uint8_t)228, (uint8_t)140, (uint8_t)67, (uint8_t)54, (uint8_t)169, (uint8_t)43, (uint8_t)244, (uint8_t)188, (uint8_t)160, (uint8_t)199, (uint8_t)164, (uint8_t)34, (uint8_t)69, (uint8_t)101, (uint8_t)184, (uint8_t)122, (uint8_t)36, (uint8_t)97, (uint8_t)195, (uint8_t)40, (uint8_t)136, (uint8_t)199, (uint8_t)39, (uint8_t)132, (uint8_t)110, (uint8_t)145, (uint8_t)177, (uint8_t)21, (uint8_t)135, (uint8_t)49, (uint8_t)98, (uint8_t)11, (uint8_t)131, (uint8_t)42, (uint8_t)44, (uint8_t)121, (uint8_t)62, (uint8_t)157, (uint8_t)130, (uint8_t)136, (uint8_t)152, (uint8_t)128, (uint8_t)67, (uint8_t)179, (uint8_t)242, (uint8_t)162, (uint8_t)157, (uint8_t)170, (uint8_t)45, (uint8_t)97, (uint8_t)131, (uint8_t)175, (uint8_t)219, (uint8_t)121, (uint8_t)7, (uint8_t)206, (uint8_t)176, (uint8_t)254, (uint8_t)228, (uint8_t)211, (uint8_t)37, (uint8_t)209, (uint8_t)75, (uint8_t)122, (uint8_t)234, (uint8_t)80, (uint8_t)204, (uint8_t)218, (uint8_t)97, (uint8_t)16, (uint8_t)248, (uint8_t)162, (uint8_t)186, (uint8_t)124, (uint8_t)115, (uint8_t)51, (uint8_t)97, (uint8_t)45, (uint8_t)17, (uint8_t)167, (uint8_t)6, (uint8_t)213, (uint8_t)184, (uint8_t)47, (uint8_t)77, (uint8_t)244, (uint8_t)126, (uint8_t)137, (uint8_t)133, (uint8_t)99, (uint8_t)193, (uint8_t)140, (uint8_t)141, (uint8_t)45, (uint8_t)62, (uint8_t)20, (uint8_t)24, (uint8_t)174, (uint8_t)32, (uint8_t)117, (uint8_t)48, (uint8_t)155, (uint8_t)101, (uint8_t)137, (uint8_t)148, (uint8_t)9, (uint8_t)187, (uint8_t)78, (uint8_t)15, (uint8_t)94, (uint8_t)207, (uint8_t)119, (uint8_t)146, (uint8_t)185, (uint8_t)64, (uint8_t)62, (uint8_t)164, (uint8_t)146, (uint8_t)219, (uint8_t)163, (uint8_t)69, (uint8_t)27};
            p11001_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_11002(), &PH);
        {
            char16_t* busname = u"jdfdrHbtouwuRfJunjpt";
            p11002_busname_SET_(busname, &PH) ;
        }
        p11002_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)245, (uint8_t)107, (uint8_t)181, (uint8_t)182, (uint8_t)160, (uint8_t)102, (uint8_t)35, (uint8_t)135, (uint8_t)47, (uint8_t)221, (uint8_t)103, (uint8_t)17, (uint8_t)232, (uint8_t)191, (uint8_t)131, (uint8_t)17, (uint8_t)224, (uint8_t)104, (uint8_t)46, (uint8_t)174, (uint8_t)151, (uint8_t)23, (uint8_t)214, (uint8_t)226, (uint8_t)29, (uint8_t)18, (uint8_t)97, (uint8_t)248, (uint8_t)11, (uint8_t)224, (uint8_t)125, (uint8_t)3, (uint8_t)176, (uint8_t)36, (uint8_t)133, (uint8_t)236, (uint8_t)252, (uint8_t)134, (uint8_t)163, (uint8_t)149, (uint8_t)58, (uint8_t)94, (uint8_t)171, (uint8_t)144, (uint8_t)236, (uint8_t)19, (uint8_t)40, (uint8_t)133, (uint8_t)56, (uint8_t)209, (uint8_t)67, (uint8_t)54, (uint8_t)250, (uint8_t)168, (uint8_t)222, (uint8_t)80, (uint8_t)206, (uint8_t)36, (uint8_t)192, (uint8_t)194, (uint8_t)173, (uint8_t)156, (uint8_t)50, (uint8_t)80, (uint8_t)233, (uint8_t)100, (uint8_t)77, (uint8_t)98, (uint8_t)236, (uint8_t)99, (uint8_t)69, (uint8_t)36, (uint8_t)59, (uint8_t)218, (uint8_t)152, (uint8_t)120, (uint8_t)4, (uint8_t)165, (uint8_t)58, (uint8_t)194, (uint8_t)189, (uint8_t)182, (uint8_t)121, (uint8_t)236, (uint8_t)174, (uint8_t)47, (uint8_t)106, (uint8_t)174, (uint8_t)93, (uint8_t)136, (uint8_t)29, (uint8_t)42, (uint8_t)10, (uint8_t)117, (uint8_t)217, (uint8_t)129, (uint8_t)143, (uint8_t)203, (uint8_t)230, (uint8_t)114, (uint8_t)126, (uint8_t)232, (uint8_t)127, (uint8_t)22, (uint8_t)159, (uint8_t)176, (uint8_t)78, (uint8_t)44, (uint8_t)42, (uint8_t)87, (uint8_t)207, (uint8_t)38, (uint8_t)206, (uint8_t)86, (uint8_t)116, (uint8_t)179, (uint8_t)77, (uint8_t)229, (uint8_t)165, (uint8_t)245, (uint8_t)165, (uint8_t)44, (uint8_t)140, (uint8_t)159, (uint8_t)54, (uint8_t)206, (uint8_t)201, (uint8_t)179};
            p11002_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11002_bus_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p11002_count_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p11002_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p11002_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        p11002_request_id_SET((uint32_t)3636707651L, PH.base.pack) ;
        p11002_address_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p11002_regstart_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEVICE_OP_WRITE_REPLY_11003(), &PH);
        p11003_request_id_SET((uint32_t)935165781L, PH.base.pack) ;
        p11003_result_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADAP_TUNING_11010(), &PH);
        p11010_omega_SET((float)2.4811135E38F, PH.base.pack) ;
        p11010_achieved_SET((float)3.0783045E38F, PH.base.pack) ;
        p11010_sigma_dot_SET((float)1.3209549E38F, PH.base.pack) ;
        p11010_omega_dot_SET((float)3.020351E38F, PH.base.pack) ;
        p11010_f_SET((float)1.9595826E38F, PH.base.pack) ;
        p11010_theta_SET((float) -1.92436E38F, PH.base.pack) ;
        p11010_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_PITCH, PH.base.pack) ;
        p11010_desired_SET((float) -3.174889E38F, PH.base.pack) ;
        p11010_u_SET((float) -1.3658877E38F, PH.base.pack) ;
        p11010_sigma_SET((float)2.8186976E38F, PH.base.pack) ;
        p11010_error_SET((float) -1.2493096E38F, PH.base.pack) ;
        p11010_f_dot_SET((float)1.4240276E38F, PH.base.pack) ;
        p11010_theta_dot_SET((float) -8.170701E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ADAP_TUNING_11010(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VISION_POSITION_DELTA_11011(), &PH);
        p11011_time_delta_usec_SET((uint64_t)2232829211584213443L, PH.base.pack) ;
        p11011_confidence_SET((float) -3.2751358E38F, PH.base.pack) ;
        {
            float position_delta[] =  {3.0978067E38F, -1.3817194E38F, -2.8077698E37F};
            p11011_position_delta_SET(&position_delta, 0, PH.base.pack) ;
        }
        p11011_time_usec_SET((uint64_t)7588275623408616166L, PH.base.pack) ;
        {
            float angle_delta[] =  {1.0976641E38F, -2.9963323E38F, 3.291935E38F};
            p11011_angle_delta_SET(&angle_delta, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


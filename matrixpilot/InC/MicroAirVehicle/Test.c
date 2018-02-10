
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
    return  _en__a(get_bits(data, 40, 4));
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
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 14:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
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
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 69:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 70:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 71:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 72:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 73:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 83:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 84:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 85:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 94:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 96:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 97:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 98:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 99:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 100:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 102:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 103:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 110:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_5;
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
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 14:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
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
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 69:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 70:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 71:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 72:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 73:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 83:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 84:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 85:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 94:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 96:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 97:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 98:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 99:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 100:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 102:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 103:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 110:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_5;
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
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 14:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
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
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 69:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 70:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 71:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 72:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 73:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 83:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 84:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 85:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 94:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 96:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 97:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 98:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 99:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 100:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 102:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 103:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 110:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_5;
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
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 14:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
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
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 69:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 70:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 71:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 72:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 73:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 83:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 84:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 85:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 94:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 96:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 97:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 98:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 99:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 100:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 102:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 103:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 110:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_5;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_CMD p77_command_GET(Pack * src)//Command ID, as defined by MAV_CMD enum.
{
    uint8_t * data = src->data;
    switch(get_bits(data, 0, 8))
    {
        case 0:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE_ADVANCED;
        case 1:
            return e_MAV_CMD_MAV_CMD_NAV_WAYPOINT;
        case 2:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM;
        case 3:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS;
        case 4:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TIME;
        case 5:
            return e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH;
        case 6:
            return e_MAV_CMD_MAV_CMD_NAV_LAND;
        case 7:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF;
        case 8:
            return e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL;
        case 9:
            return e_MAV_CMD_MAV_CMD_NAV_TAKEOFF_LOCAL;
        case 10:
            return e_MAV_CMD_MAV_CMD_NAV_FOLLOW;
        case 11:
            return e_MAV_CMD_MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        case 12:
            return e_MAV_CMD_MAV_CMD_NAV_LOITER_TO_ALT;
        case 13:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW;
        case 14:
            return e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION;
        case 15:
            return e_MAV_CMD_MAV_CMD_NAV_ROI;
        case 16:
            return e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING;
        case 17:
            return e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT;
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
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 63:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 68:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 69:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 70:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 71:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 72:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 73:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 74:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 80:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 81:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 83:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 84:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 85:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 89:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 94:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 96:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 97:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 98:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 99:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 100:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 102:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 103:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 109:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 110:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 117:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_USER_5;
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
INLINER void p150_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p150_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_SET_150()
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
INLINER void p151_read_req_type_SET(int16_t  src, Pack * dst)//Type of flexifunction data requested
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p151_data_index_SET(int16_t  src, Pack * dst)//index into data where needed
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_READ_REQ_151()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 151));
};
INLINER void p152_func_index_SET(uint16_t  src, Pack * dst)//Function index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p152_func_count_SET(uint16_t  src, Pack * dst)//Total count of functions
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p152_data_address_SET(uint16_t  src, Pack * dst)//Address in the flexifunction data, Set to 0xFFFF to use address in target memory
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p152_data_size_SET(uint16_t  src, Pack * dst)//Size of the
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p152_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p152_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p152_data__SET(int8_t*  src, int32_t pos, Pack * dst) //Settings data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 48; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_152()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 152));
};
INLINER void p153_func_index_SET(uint16_t  src, Pack * dst)//Function index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p153_result_SET(uint16_t  src, Pack * dst)//result of acknowledge, 0=fail, 1=good
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p153_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p153_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 153));
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
INLINER void p155_directory_type_SET(uint8_t  src, Pack * dst)//0=inputs, 1=outputs
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p155_start_index_SET(uint8_t  src, Pack * dst)//index of first directory entry to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p155_count_SET(uint8_t  src, Pack * dst)//count of directory entries to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p155_directory_data_SET(int8_t*  src, int32_t pos, Pack * dst) //Settings data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 48; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_155()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 155));
};
INLINER void p156_result_SET(uint16_t  src, Pack * dst)//result of acknowledge, 0=fail, 1=good
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p156_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p156_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p156_directory_type_SET(uint8_t  src, Pack * dst)//0=inputs, 1=outputs
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p156_start_index_SET(uint8_t  src, Pack * dst)//index of first directory entry to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p156_count_SET(uint8_t  src, Pack * dst)//count of directory entries to write
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_ACK_156()
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
INLINER void p157_command_type_SET(uint8_t  src, Pack * dst)//Flexifunction command type
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_157()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 157));
};
INLINER void p158_command_type_SET(uint16_t  src, Pack * dst)//Command acknowledged
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p158_result_SET(uint16_t  src, Pack * dst)//result of acknowledge
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
Pack * c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_ACK_158()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 158));
};
INLINER void p170_sue_waypoint_index_SET(uint16_t  src, Pack * dst)//Serial UDB Extra Waypoint Index
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p170_sue_cog_SET(uint16_t  src, Pack * dst)//Serial UDB Extra GPS Course Over Ground
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p170_sue_cpu_load_SET(uint16_t  src, Pack * dst)//Serial UDB Extra CPU Load
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p170_sue_air_speed_3DIMU_SET(uint16_t  src, Pack * dst)//Serial UDB Extra 3D IMU Air Speed
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p170_sue_time_SET(uint32_t  src, Pack * dst)//Serial UDB Extra Time
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER void p170_sue_status_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER void p170_sue_latitude_SET(int32_t  src, Pack * dst)//Serial UDB Extra Latitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  13);
}
INLINER void p170_sue_longitude_SET(int32_t  src, Pack * dst)//Serial UDB Extra Longitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER void p170_sue_altitude_SET(int32_t  src, Pack * dst)//Serial UDB Extra Altitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER void p170_sue_rmat0_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 0
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  25);
}
INLINER void p170_sue_rmat1_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  27);
}
INLINER void p170_sue_rmat2_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  29);
}
INLINER void p170_sue_rmat3_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 3
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  31);
}
INLINER void p170_sue_rmat4_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 4
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  33);
}
INLINER void p170_sue_rmat5_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 5
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  35);
}
INLINER void p170_sue_rmat6_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 6
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  37);
}
INLINER void p170_sue_rmat7_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 7
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  39);
}
INLINER void p170_sue_rmat8_SET(int16_t  src, Pack * dst)//Serial UDB Extra Rmat 8
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  41);
}
INLINER void p170_sue_sog_SET(int16_t  src, Pack * dst)//Serial UDB Extra Speed Over Ground
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  43);
}
INLINER void p170_sue_estimated_wind_0_SET(int16_t  src, Pack * dst)//Serial UDB Extra Estimated Wind 0
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  45);
}
INLINER void p170_sue_estimated_wind_1_SET(int16_t  src, Pack * dst)//Serial UDB Extra Estimated Wind 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  47);
}
INLINER void p170_sue_estimated_wind_2_SET(int16_t  src, Pack * dst)//Serial UDB Extra Estimated Wind 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  49);
}
INLINER void p170_sue_magFieldEarth0_SET(int16_t  src, Pack * dst)//Serial UDB Extra Magnetic Field Earth 0
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  51);
}
INLINER void p170_sue_magFieldEarth1_SET(int16_t  src, Pack * dst)//Serial UDB Extra Magnetic Field Earth 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  53);
}
INLINER void p170_sue_magFieldEarth2_SET(int16_t  src, Pack * dst)//Serial UDB Extra Magnetic Field Earth 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  55);
}
INLINER void p170_sue_svs_SET(int16_t  src, Pack * dst)//Serial UDB Extra Number of Sattelites in View
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  57);
}
INLINER void p170_sue_hdop_SET(int16_t  src, Pack * dst)//Serial UDB Extra GPS Horizontal Dilution of Precision
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  59);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_A_170()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 170));
};
INLINER void p171_sue_time_SET(uint32_t  src, Pack * dst)//Serial UDB Extra Time
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p171_sue_flags_SET(uint32_t  src, Pack * dst)//Serial UDB Extra Status Flags
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p171_sue_pwm_input_1_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p171_sue_pwm_input_2_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p171_sue_pwm_input_3_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 3
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p171_sue_pwm_input_4_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 4
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p171_sue_pwm_input_5_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 5
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
INLINER void p171_sue_pwm_input_6_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 6
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  18);
}
INLINER void p171_sue_pwm_input_7_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 7
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  20);
}
INLINER void p171_sue_pwm_input_8_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 8
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER void p171_sue_pwm_input_9_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 9
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER void p171_sue_pwm_input_10_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  26);
}
INLINER void p171_sue_pwm_input_11_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 11
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  28);
}
INLINER void p171_sue_pwm_input_12_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Input Channel 12
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  30);
}
INLINER void p171_sue_pwm_output_1_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  32);
}
INLINER void p171_sue_pwm_output_2_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  34);
}
INLINER void p171_sue_pwm_output_3_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 3
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  36);
}
INLINER void p171_sue_pwm_output_4_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 4
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  38);
}
INLINER void p171_sue_pwm_output_5_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 5
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  40);
}
INLINER void p171_sue_pwm_output_6_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 6
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  42);
}
INLINER void p171_sue_pwm_output_7_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 7
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  44);
}
INLINER void p171_sue_pwm_output_8_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 8
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  46);
}
INLINER void p171_sue_pwm_output_9_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 9
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  48);
}
INLINER void p171_sue_pwm_output_10_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  50);
}
INLINER void p171_sue_pwm_output_11_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 11
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  52);
}
INLINER void p171_sue_pwm_output_12_SET(int16_t  src, Pack * dst)//Serial UDB Extra PWM Output Channel 12
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  54);
}
INLINER void p171_sue_imu_location_x_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Location X
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  56);
}
INLINER void p171_sue_imu_location_y_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Location Y
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  58);
}
INLINER void p171_sue_imu_location_z_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Location Z
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  60);
}
INLINER void p171_sue_location_error_earth_x_SET(int16_t  src, Pack * dst)//Serial UDB Location Error Earth X
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  62);
}
INLINER void p171_sue_location_error_earth_y_SET(int16_t  src, Pack * dst)//Serial UDB Location Error Earth Y
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  64);
}
INLINER void p171_sue_location_error_earth_z_SET(int16_t  src, Pack * dst)//Serial UDB Location Error Earth Z
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  66);
}
INLINER void p171_sue_osc_fails_SET(int16_t  src, Pack * dst)//Serial UDB Extra Oscillator Failure Count
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  68);
}
INLINER void p171_sue_imu_velocity_x_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Velocity X
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  70);
}
INLINER void p171_sue_imu_velocity_y_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Velocity Y
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  72);
}
INLINER void p171_sue_imu_velocity_z_SET(int16_t  src, Pack * dst)//Serial UDB Extra IMU Velocity Z
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  74);
}
INLINER void p171_sue_waypoint_goal_x_SET(int16_t  src, Pack * dst)//Serial UDB Extra Current Waypoint Goal X
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  76);
}
INLINER void p171_sue_waypoint_goal_y_SET(int16_t  src, Pack * dst)//Serial UDB Extra Current Waypoint Goal Y
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  78);
}
INLINER void p171_sue_waypoint_goal_z_SET(int16_t  src, Pack * dst)//Serial UDB Extra Current Waypoint Goal Z
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  80);
}
INLINER void p171_sue_aero_x_SET(int16_t  src, Pack * dst)//Aeroforce in UDB X Axis
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  82);
}
INLINER void p171_sue_aero_y_SET(int16_t  src, Pack * dst)//Aeroforce in UDB Y Axis
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  84);
}
INLINER void p171_sue_aero_z_SET(int16_t  src, Pack * dst)//Aeroforce in UDB Z axis
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  86);
}
INLINER void p171_sue_barom_temp_SET(int16_t  src, Pack * dst)//SUE barometer temperature
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  88);
}
INLINER void p171_sue_barom_press_SET(int32_t  src, Pack * dst)//SUE barometer pressure
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  90);
}
INLINER void p171_sue_barom_alt_SET(int32_t  src, Pack * dst)//SUE barometer altitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  94);
}
INLINER void p171_sue_bat_volt_SET(int16_t  src, Pack * dst)//SUE battery voltage
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  98);
}
INLINER void p171_sue_bat_amp_SET(int16_t  src, Pack * dst)//SUE battery current
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  100);
}
INLINER void p171_sue_bat_amp_hours_SET(int16_t  src, Pack * dst)//SUE battery milli amp hours used
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  102);
}
INLINER void p171_sue_desired_height_SET(int16_t  src, Pack * dst)//Sue autopilot desired height
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  104);
}
INLINER void p171_sue_memory_stack_free_SET(int16_t  src, Pack * dst)//Serial UDB Extra Stack Memory Free
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  106);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_B_171()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 171));
};
INLINER void p172_sue_ROLL_STABILIZATION_AILERONS_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Roll Stabilization with Ailerons Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p172_sue_ROLL_STABILIZATION_RUDDER_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Roll Stabilization with Rudder Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p172_sue_PITCH_STABILIZATION_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Pitch Stabilization Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p172_sue_YAW_STABILIZATION_RUDDER_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Yaw Stabilization using Rudder Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p172_sue_YAW_STABILIZATION_AILERON_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Yaw Stabilization using Ailerons Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p172_sue_AILERON_NAVIGATION_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Navigation with Ailerons Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p172_sue_RUDDER_NAVIGATION_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Navigation with Rudder Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p172_sue_ALTITUDEHOLD_STABILIZED_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p172_sue_ALTITUDEHOLD_WAYPOINT_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p172_sue_RACING_MODE_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Firmware racing mode enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F4_172()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 172));
};
INLINER void p173_sue_YAWKP_AILERON_SET(float  src, Pack * dst)//Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p173_sue_YAWKD_AILERON_SET(float  src, Pack * dst)//Serial UDB YAWKD_AILERON Gain for Rate control of navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p173_sue_ROLLKP_SET(float  src, Pack * dst)//Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p173_sue_ROLLKD_SET(float  src, Pack * dst)//Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F5_173()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 173));
};
INLINER void p174_sue_PITCHGAIN_SET(float  src, Pack * dst)//Serial UDB Extra PITCHGAIN Proportional Control
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p174_sue_PITCHKD_SET(float  src, Pack * dst)//Serial UDB Extra Pitch Rate Control
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p174_sue_RUDDER_ELEV_MIX_SET(float  src, Pack * dst)//Serial UDB Extra Rudder to Elevator Mix
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p174_sue_ROLL_ELEV_MIX_SET(float  src, Pack * dst)//Serial UDB Extra Roll to Elevator Mix
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p174_sue_ELEVATOR_BOOST_SET(float  src, Pack * dst)//Gain For Boosting Manual Elevator control When Plane Stabilized
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F6_174()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 174));
};
INLINER void p175_sue_YAWKP_RUDDER_SET(float  src, Pack * dst)//Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p175_sue_YAWKD_RUDDER_SET(float  src, Pack * dst)//Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p175_sue_ROLLKP_RUDDER_SET(float  src, Pack * dst)//Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p175_sue_ROLLKD_RUDDER_SET(float  src, Pack * dst)//Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p175_sue_RUDDER_BOOST_SET(float  src, Pack * dst)//SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p175_sue_RTL_PITCH_DOWN_SET(float  src, Pack * dst)//Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F7_175()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 175));
};
INLINER void p176_sue_HEIGHT_TARGET_MAX_SET(float  src, Pack * dst)//Serial UDB Extra HEIGHT_TARGET_MAX
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p176_sue_HEIGHT_TARGET_MIN_SET(float  src, Pack * dst)//Serial UDB Extra HEIGHT_TARGET_MIN
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p176_sue_ALT_HOLD_THROTTLE_MIN_SET(float  src, Pack * dst)//Serial UDB Extra ALT_HOLD_THROTTLE_MIN
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p176_sue_ALT_HOLD_THROTTLE_MAX_SET(float  src, Pack * dst)//Serial UDB Extra ALT_HOLD_THROTTLE_MAX
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p176_sue_ALT_HOLD_PITCH_MIN_SET(float  src, Pack * dst)//Serial UDB Extra ALT_HOLD_PITCH_MIN
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p176_sue_ALT_HOLD_PITCH_MAX_SET(float  src, Pack * dst)//Serial UDB Extra ALT_HOLD_PITCH_MAX
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p176_sue_ALT_HOLD_PITCH_HIGH_SET(float  src, Pack * dst)//Serial UDB Extra ALT_HOLD_PITCH_HIGH
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F8_176()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 176));
};
INLINER void p177_sue_week_no_SET(int16_t  src, Pack * dst)//Serial UDB Extra GPS Week Number
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  0);
}
INLINER void p177_sue_lat_origin_SET(int32_t  src, Pack * dst)//Serial UDB Extra MP Origin Latitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
INLINER void p177_sue_lon_origin_SET(int32_t  src, Pack * dst)//Serial UDB Extra MP Origin Longitude
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  6);
}
INLINER void p177_sue_alt_origin_SET(int32_t  src, Pack * dst)//Serial UDB Extra MP Origin Altitude Above Sea Level
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F13_177()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 177));
};
INLINER void p178_sue_TRAP_SOURCE_SET(uint32_t  src, Pack * dst)//Serial UDB Extra Type Program Address of Last Trap
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p178_sue_WIND_ESTIMATION_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Wind Estimation Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p178_sue_GPS_TYPE_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of GPS Unit
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p178_sue_DR_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Dead Reckoning Enabled
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p178_sue_BOARD_TYPE_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of UDB Hardware
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p178_sue_AIRFRAME_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of Airframe
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p178_sue_RCON_SET(int16_t  src, Pack * dst)//Serial UDB Extra Reboot Register of DSPIC
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER void p178_sue_TRAP_FLAGS_SET(int16_t  src, Pack * dst)//Serial UDB Extra  Last dspic Trap Flags
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER void p178_sue_osc_fail_count_SET(int16_t  src, Pack * dst)//Serial UDB Extra Number of Ocillator Failures
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER void p178_sue_CLOCK_CONFIG_SET(uint8_t  src, Pack * dst)//Serial UDB Extra UDB Internal Clock Configuration
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER void p178_sue_FLIGHT_PLAN_TYPE_SET(uint8_t  src, Pack * dst)//Serial UDB Extra Type of Flight Plan
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F14_178()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 178));
};
INLINER void p179_sue_ID_VEHICLE_MODEL_NAME_SET(uint8_t*  src, int32_t pos, Pack * dst) //Serial UDB Extra Model Name Of Vehicle
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 40; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p179_sue_ID_VEHICLE_REGISTRATION_SET(uint8_t*  src, int32_t pos, Pack * dst) //Serial UDB Extra Registraton Number of Vehicle
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  40, src_max = pos + 20; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F15_179()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 179));
};
INLINER void p180_sue_ID_LEAD_PILOT_SET(uint8_t*  src, int32_t pos, Pack * dst) //Serial UDB Extra Name of Expected Lead Pilot
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 40; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p180_sue_ID_DIY_DRONES_URL_SET(uint8_t*  src, int32_t pos, Pack * dst) //Serial UDB Extra URL of Lead Pilot or Team
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  40, src_max = pos + 70; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F16_180()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 180));
};
INLINER void p181_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p181_alt_gps_SET(int32_t  src, Pack * dst)//GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER void p181_alt_imu_SET(int32_t  src, Pack * dst)//IMU altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER void p181_alt_barometric_SET(int32_t  src, Pack * dst)//barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  12);
}
INLINER void p181_alt_optical_flow_SET(int32_t  src, Pack * dst)//Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p181_alt_range_finder_SET(int32_t  src, Pack * dst)//Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p181_alt_extra_SET(int32_t  src, Pack * dst)//Extra altitude above ground in meters, expressed as * 1000 (millimeters)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  24);
}
Pack * c_TEST_Channel_new_ALTITUDES_181()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 181));
};
INLINER void p182_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p182_airspeed_imu_SET(int16_t  src, Pack * dst)//Airspeed estimate from IMU, cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p182_airspeed_pitot_SET(int16_t  src, Pack * dst)//Pitot measured forward airpseed, cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER void p182_airspeed_hot_wire_SET(int16_t  src, Pack * dst)//Hot wire anenometer measured airspeed, cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p182_airspeed_ultrasonic_SET(int16_t  src, Pack * dst)//Ultrasonic measured airspeed, cm/s
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p182_aoa_SET(int16_t  src, Pack * dst)//Angle of attack sensor, degrees * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p182_aoy_SET(int16_t  src, Pack * dst)//Yaw angle sensor, degrees * 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
Pack * c_TEST_Channel_new_AIRSPEEDS_182()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 182));
};
INLINER void p183_sue_feed_forward_SET(float  src, Pack * dst)//SUE Feed Forward Gain
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p183_sue_turn_rate_nav_SET(float  src, Pack * dst)//SUE Max Turn Rate when Navigating
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p183_sue_turn_rate_fbw_SET(float  src, Pack * dst)//SUE Max Turn Rate in Fly By Wire Mode
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F17_183()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 183));
};
INLINER void p184_angle_of_attack_normal_SET(float  src, Pack * dst)//SUE Angle of Attack Normal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p184_angle_of_attack_inverted_SET(float  src, Pack * dst)//SUE Angle of Attack Inverted
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p184_elevator_trim_normal_SET(float  src, Pack * dst)//SUE Elevator Trim Normal
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p184_elevator_trim_inverted_SET(float  src, Pack * dst)//SUE Elevator Trim Inverted
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p184_reference_speed_SET(float  src, Pack * dst)//SUE reference_speed
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F18_184()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 184));
};
INLINER void p185_sue_aileron_output_channel_SET(uint8_t  src, Pack * dst)//SUE aileron output channel
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p185_sue_aileron_reversed_SET(uint8_t  src, Pack * dst)//SUE aileron reversed
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p185_sue_elevator_output_channel_SET(uint8_t  src, Pack * dst)//SUE elevator output channel
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p185_sue_elevator_reversed_SET(uint8_t  src, Pack * dst)//SUE elevator reversed
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p185_sue_throttle_output_channel_SET(uint8_t  src, Pack * dst)//SUE throttle output channel
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p185_sue_throttle_reversed_SET(uint8_t  src, Pack * dst)//SUE throttle reversed
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p185_sue_rudder_output_channel_SET(uint8_t  src, Pack * dst)//SUE rudder output channel
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p185_sue_rudder_reversed_SET(uint8_t  src, Pack * dst)//SUE rudder reversed
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F19_185()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 185));
};
INLINER void p186_sue_number_of_inputs_SET(uint8_t  src, Pack * dst)//SUE Number of Input Channels
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p186_sue_trim_value_input_1_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  1);
}
INLINER void p186_sue_trim_value_input_2_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  3);
}
INLINER void p186_sue_trim_value_input_3_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 3
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  5);
}
INLINER void p186_sue_trim_value_input_4_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 4
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  7);
}
INLINER void p186_sue_trim_value_input_5_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 5
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  9);
}
INLINER void p186_sue_trim_value_input_6_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 6
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  11);
}
INLINER void p186_sue_trim_value_input_7_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 7
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER void p186_sue_trim_value_input_8_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 8
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  15);
}
INLINER void p186_sue_trim_value_input_9_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 9
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  17);
}
INLINER void p186_sue_trim_value_input_10_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 10
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  19);
}
INLINER void p186_sue_trim_value_input_11_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 11
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  21);
}
INLINER void p186_sue_trim_value_input_12_SET(int16_t  src, Pack * dst)//SUE UDB PWM Trim Value on Input 12
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F20_186()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 186));
};
INLINER void p187_sue_accel_x_offset_SET(int16_t  src, Pack * dst)//SUE X accelerometer offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  0);
}
INLINER void p187_sue_accel_y_offset_SET(int16_t  src, Pack * dst)//SUE Y accelerometer offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p187_sue_accel_z_offset_SET(int16_t  src, Pack * dst)//SUE Z accelerometer offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p187_sue_gyro_x_offset_SET(int16_t  src, Pack * dst)//SUE X gyro offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER void p187_sue_gyro_y_offset_SET(int16_t  src, Pack * dst)//SUE Y gyro offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p187_sue_gyro_z_offset_SET(int16_t  src, Pack * dst)//SUE Z gyro offset
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F21_187()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 187));
};
INLINER void p188_sue_accel_x_at_calibration_SET(int16_t  src, Pack * dst)//SUE X accelerometer at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  0);
}
INLINER void p188_sue_accel_y_at_calibration_SET(int16_t  src, Pack * dst)//SUE Y accelerometer at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  2);
}
INLINER void p188_sue_accel_z_at_calibration_SET(int16_t  src, Pack * dst)//SUE Z accelerometer at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  4);
}
INLINER void p188_sue_gyro_x_at_calibration_SET(int16_t  src, Pack * dst)//SUE X gyro at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  6);
}
INLINER void p188_sue_gyro_y_at_calibration_SET(int16_t  src, Pack * dst)//SUE Y gyro at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p188_sue_gyro_z_at_calibration_SET(int16_t  src, Pack * dst)//SUE Z gyro at calibration time
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
Pack * c_TEST_Channel_new_SERIAL_UDB_EXTRA_F22_188()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 188));
};
INLINER void p244_message_id_SET(uint16_t  src, Pack * dst)//The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p244_interval_us_SET(int32_t  src, Pack * dst)//0 indicates the interval at which it is sent.
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  2);
}
Pack * c_TEST_Channel_new_MESSAGE_INTERVAL_244()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 244));
};
INLINER void p245_vtol_state_SET(e_MAV_VTOL_STATE  src, Pack * dst)//The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 0);
}
INLINER void p245_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 3);
}
Pack * c_TEST_Channel_new_EXTENDED_SYS_STATE_245()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 245));
};
INLINER void p246_heading_SET(uint16_t  src, Pack * dst)//Course over ground in centidegrees
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p246_hor_velocity_SET(uint16_t  src, Pack * dst)//The horizontal velocity in centimeters/second
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p246_squawk_SET(uint16_t  src, Pack * dst)//Squawk code
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p246_ICAO_address_SET(uint32_t  src, Pack * dst)//ICAO address
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p246_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  10);
}
INLINER void p246_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  14);
}
INLINER void p246_altitude_SET(int32_t  src, Pack * dst)//Altitude(ASL) in millimeters
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  18);
}
INLINER void p246_ver_velocity_SET(int16_t  src, Pack * dst)//The vertical velocity in centimeters/second, positive is up
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  22);
}
INLINER void p246_tslc_SET(uint8_t  src, Pack * dst)//Time since last communication in seconds
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  24);
}
INLINER void p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE  src, Pack * dst)//Type from ADSB_ALTITUDE_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 200);
}
INLINER void p246_emitter_type_SET(e_ADSB_EMITTER_TYPE  src, Pack * dst)//Type from ADSB_EMITTER_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 201);
}
INLINER void p246_flags_SET(e_ADSB_FLAGS  src, Pack * dst)//Flags to indicate various statuses including valid data fields
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 7, data, 206);
}
INLINER void p246_callsign_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //The callsign, 8+null
{
    if(dst->base.field_bit != 213 && insert_field(dst, 213, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p246_callsign_SET_(char16_t*  src, Bounds_Inside * dst) {p246_callsign_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_ADSB_VEHICLE_246()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 246));
};
INLINER void p247_id_SET(uint32_t  src, Pack * dst)//Unique identifier, domain based on src field
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p247_time_to_minimum_delta_SET(float  src, Pack * dst)//Estimated time until collision occurs (seconds)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p247_altitude_minimum_delta_SET(float  src, Pack * dst)//Closest vertical distance in meters between vehicle and object
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p247_horizontal_minimum_delta_SET(float  src, Pack * dst)//Closest horizontal distance in meteres between vehicle and object
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p247_src__SET(e_MAV_COLLISION_SRC  src, Pack * dst)//Collision data source
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 1, data, 128);
}
INLINER void p247_action_SET(e_MAV_COLLISION_ACTION  src, Pack * dst)//Action that is being taken to avoid this collision
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 129);
}
INLINER void p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL  src, Pack * dst)//How concerned the aircraft is about this collision
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 132);
}
Pack * c_TEST_Channel_new_COLLISION_247()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 247));
};/**
*A code that identifies the software component that understands this message (analogous to usb device classes
*	or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
*	and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
*	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
*	Message_types greater than 32767 are considered local experiments and should not be checked in to any
*	widely distributed codebase*/
INLINER void p248_message_type_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p248_target_network_SET(uint8_t  src, Pack * dst)//Network ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p248_target_system_SET(uint8_t  src, Pack * dst)//System ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p248_target_component_SET(uint8_t  src, Pack * dst)//Component ID (0 for broadcast)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
/**
*Variable length payload. The length is defined by the remaining message length when subtracting the header
*	and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	message_type.  The particular encoding used can be extension specific and might not always be documented
*	as part of the mavlink specification*/
INLINER void p248_payload_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  5, src_max = pos + 249; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_V2_EXTENSION_248()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 248));
};
INLINER void p249_address_SET(uint16_t  src, Pack * dst)//Starting address of the debug variables
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p249_ver_SET(uint8_t  src, Pack * dst)//Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p249_type_SET(uint8_t  src, Pack * dst)//Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p249_value_SET(int8_t*  src, int32_t pos, Pack * dst) //Memory contents at specified address
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_MEMORY_VECT_249()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 249));
};
INLINER void p250_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p250_x_SET(float  src, Pack * dst)//x
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p250_y_SET(float  src, Pack * dst)//y
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p250_z_SET(float  src, Pack * dst)//z
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p250_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name
{
    if(dst->base.field_bit != 160 && insert_field(dst, 160, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p250_name_SET_(char16_t*  src, Bounds_Inside * dst) {p250_name_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_DEBUG_VECT_250()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 250));
};
INLINER void p251_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p251_value_SET(float  src, Pack * dst)//Floating point value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p251_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of the debug variable
{
    if(dst->base.field_bit != 64 && insert_field(dst, 64, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p251_name_SET_(char16_t*  src, Bounds_Inside * dst) {p251_name_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_NAMED_VALUE_FLOAT_251()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 251));
};
INLINER void p252_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p252_value_SET(int32_t  src, Pack * dst)//Signed integer value
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER void p252_name_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Name of the debug variable
{
    if(dst->base.field_bit != 64 && insert_field(dst, 64, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p252_name_SET_(char16_t*  src, Bounds_Inside * dst) {p252_name_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_NAMED_VALUE_INT_252()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 252));
};
INLINER void p253_severity_SET(e_MAV_SEVERITY  src, Pack * dst)//Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 0);
}
INLINER void p253_text_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Status text message, without null termination character
{
    if(dst->base.field_bit != 3 && insert_field(dst, 3, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p253_text_SET_(char16_t*  src, Bounds_Inside * dst) {p253_text_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_STATUSTEXT_253()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 253));
};
INLINER void p254_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p254_ind_SET(uint8_t  src, Pack * dst)//index of debug variable
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p254_value_SET(float  src, Pack * dst)//DEBUG value
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
Pack * c_TEST_Channel_new_DEBUG_254()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 254));
};
INLINER void p256_initial_timestamp_SET(uint64_t  src, Pack * dst)//initial timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p256_target_system_SET(uint8_t  src, Pack * dst)//system id of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p256_target_component_SET(uint8_t  src, Pack * dst)//component ID of the target
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p256_secret_key_SET(uint8_t*  src, int32_t pos, Pack * dst) //signing key
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_SETUP_SIGNING_256()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 256));
};
INLINER void p257_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p257_last_change_ms_SET(uint32_t  src, Pack * dst)//Time of last change of button state
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p257_state_SET(uint8_t  src, Pack * dst)//Bitmap state of buttons
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
Pack * c_TEST_Channel_new_BUTTON_CHANGE_257()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 257));
};
INLINER void p258_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p258_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p258_tune_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //tune in board specific format
{
    if(dst->base.field_bit != 16 && insert_field(dst, 16, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p258_tune_SET_(char16_t*  src, Bounds_Inside * dst) {p258_tune_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_PLAY_TUNE_258()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 258));
};
INLINER void p259_resolution_h_SET(uint16_t  src, Pack * dst)//Image resolution in pixels horizontal
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p259_resolution_v_SET(uint16_t  src, Pack * dst)//Image resolution in pixels vertical
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p259_cam_definition_version_SET(uint16_t  src, Pack * dst)//Camera definition version (iteration)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p259_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p259_firmware_version_SET(uint32_t  src, Pack * dst)//0xff = Major)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  10);
}
INLINER void p259_vendor_name_SET(uint8_t*  src, int32_t pos, Pack * dst) //Name of the camera vendor
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  14, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p259_model_name_SET(uint8_t*  src, int32_t pos, Pack * dst) //Name of the camera model
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  46, src_max = pos + 32; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p259_focal_length_SET(float  src, Pack * dst)//Focal length in mm
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  78);
}
INLINER void p259_sensor_size_h_SET(float  src, Pack * dst)//Image sensor size horizontal in mm
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  82);
}
INLINER void p259_sensor_size_v_SET(float  src, Pack * dst)//Image sensor size vertical in mm
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  86);
}
INLINER void p259_lens_id_SET(uint8_t  src, Pack * dst)//Reserved for a lens ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  90);
}
INLINER void p259_flags_SET(e_CAMERA_CAP_FLAGS  src, Pack * dst)//CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 6, data, 728);
}
INLINER void p259_cam_definition_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Camera definition URI (if any, otherwise only basic functions will be available).
{
    if(dst->base.field_bit != 734 && insert_field(dst, 734, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p259_cam_definition_uri_SET_(char16_t*  src, Bounds_Inside * dst) {p259_cam_definition_uri_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_CAMERA_INFORMATION_259()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 259));
};
INLINER void p260_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p260_mode_id_SET(e_CAMERA_MODE  src, Pack * dst)//Camera mode (CAMERA_MODE)
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 32);
}
Pack * c_TEST_Channel_new_CAMERA_SETTINGS_260()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 260));
};
INLINER void p261_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p261_storage_id_SET(uint8_t  src, Pack * dst)//Storage ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p261_storage_count_SET(uint8_t  src, Pack * dst)//Number of storage devices
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p261_status_SET(uint8_t  src, Pack * dst)//Status of storage (0 not available, 1 unformatted, 2 formatted)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p261_total_capacity_SET(float  src, Pack * dst)//Total capacity in MiB
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  7);
}
INLINER void p261_used_capacity_SET(float  src, Pack * dst)//Used capacity in MiB
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  11);
}
INLINER void p261_available_capacity_SET(float  src, Pack * dst)//Available capacity in MiB
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  15);
}
INLINER void p261_read_speed_SET(float  src, Pack * dst)//Read speed in MiB/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  19);
}
INLINER void p261_write_speed_SET(float  src, Pack * dst)//Write speed in MiB/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
Pack * c_TEST_Channel_new_STORAGE_INFORMATION_261()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 261));
};
INLINER void p262_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p262_recording_time_ms_SET(uint32_t  src, Pack * dst)//Time in milliseconds since recording started
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
/**
*Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval
*	set and capture in progress*/
INLINER void p262_image_status_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p262_video_status_SET(uint8_t  src, Pack * dst)//Current status of video capturing (0: idle, 1: capture in progress)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p262_image_interval_SET(float  src, Pack * dst)//Image capture interval in seconds
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p262_available_capacity_SET(float  src, Pack * dst)//Available storage capacity in MiB
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
Pack * c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 262));
};
INLINER void p263_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p263_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER void p263_camera_id_SET(uint8_t  src, Pack * dst)//Camera ID (1 for first, 2 for second, etc.)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER void p263_lat_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7 where image was taken
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  13);
}
INLINER void p263_lon_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7 where capture was taken
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER void p263_alt_SET(int32_t  src, Pack * dst)//Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER void p263_relative_alt_SET(int32_t  src, Pack * dst)//Altitude above ground in meters, expressed as * 1E3 where image was taken
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
INLINER void p263_q_SET(float*  src, int32_t pos, Pack * dst) //Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p263_image_index_SET(int32_t  src, Pack * dst)//Zero based index of this image (image count since armed -1)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  45);
}
INLINER void p263_capture_result_SET(int8_t  src, Pack * dst)//Boolean indicating success (1) or failure (0) while capturing this image.
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  49);
}
INLINER void p263_file_url_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //URL of image taken. Either local storage or http:foo.jpg if camera provides an HTTP interface.
{
    if(dst->base.field_bit != 402 && insert_field(dst, 402, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p263_file_url_SET_(char16_t*  src, Bounds_Inside * dst) {p263_file_url_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 263));
};
INLINER void p264_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p264_arming_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  4);
}
INLINER void p264_takeoff_time_utc_SET(uint64_t  src, Pack * dst)//Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER void p264_flight_uuid_SET(uint64_t  src, Pack * dst)//Universally unique identifier (UUID) of flight, should correspond to name of logfiles
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  20);
}
Pack * c_TEST_Channel_new_FLIGHT_INFORMATION_264()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 264));
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


void c_TEST_Channel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_TRICOPTER);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2350547745L);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED));
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY));
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)62518);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)43885);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)45873);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)98);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)31156);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)58769);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)40546);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)39250);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)34208);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING));
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -23869);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2359963211507791355L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1656917868L);
};


void c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_x_GET(pack) == (float)1.9363945E38F);
    assert(p3_vz_GET(pack) == (float) -1.3236602E38F);
    assert(p3_afx_GET(pack) == (float)2.686841E38F);
    assert(p3_vy_GET(pack) == (float)3.3420322E38F);
    assert(p3_yaw_rate_GET(pack) == (float)6.682461E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p3_afz_GET(pack) == (float)2.9500363E38F);
    assert(p3_z_GET(pack) == (float)1.8426516E38F);
    assert(p3_y_GET(pack) == (float) -3.2231994E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)10971);
    assert(p3_afy_GET(pack) == (float)1.7358694E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)552633439L);
    assert(p3_vx_GET(pack) == (float) -1.1590105E38F);
    assert(p3_yaw_GET(pack) == (float)1.3479233E37F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)7875498662349690672L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p4_seq_GET(pack) == (uint32_t)3529787616L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)16);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p5_passkey_LEN(ph) == 25);
    {
        char16_t * exemplary = u"lbogabvBlzsoemfppbnsqvpec";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)27);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)190);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 9);
    {
        char16_t * exemplary = u"lfNkcUcij";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)3818623829L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"zlcrnbgiV";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -14503);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)66);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float)6.4976175E37F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)28161);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)23384);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"T";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"bkebuFda";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
    assert(p23_param_value_GET(pack) == (float)1.455713E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)193);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_lat_GET(pack) == (int32_t)1814399168);
    assert(p24_time_usec_GET(pack) == (uint64_t)6101908425062521068L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3481926648L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2534368895L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)3799599399L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)17212);
    assert(p24_alt_GET(pack) == (int32_t)1659203894);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1228914901L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)24083);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -689226831);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)42294);
    assert(p24_lon_GET(pack) == (int32_t) -546563754);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)35914);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)90, (uint8_t)204, (uint8_t)114, (uint8_t)94, (uint8_t)125, (uint8_t)95, (uint8_t)61, (uint8_t)91, (uint8_t)200, (uint8_t)202, (uint8_t)101, (uint8_t)50, (uint8_t)86, (uint8_t)182, (uint8_t)183, (uint8_t)143, (uint8_t)158, (uint8_t)133, (uint8_t)17, (uint8_t)184} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)163);
    {
        uint8_t exemplary[] =  {(uint8_t)246, (uint8_t)225, (uint8_t)213, (uint8_t)59, (uint8_t)243, (uint8_t)92, (uint8_t)179, (uint8_t)205, (uint8_t)115, (uint8_t)31, (uint8_t)74, (uint8_t)27, (uint8_t)58, (uint8_t)108, (uint8_t)20, (uint8_t)142, (uint8_t)145, (uint8_t)2, (uint8_t)209, (uint8_t)236} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)198, (uint8_t)111, (uint8_t)178, (uint8_t)210, (uint8_t)103, (uint8_t)65, (uint8_t)178, (uint8_t)34, (uint8_t)44, (uint8_t)54, (uint8_t)242, (uint8_t)148, (uint8_t)144, (uint8_t)201, (uint8_t)0, (uint8_t)17, (uint8_t)171, (uint8_t)43, (uint8_t)36} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)114, (uint8_t)152, (uint8_t)145, (uint8_t)125, (uint8_t)128, (uint8_t)113, (uint8_t)22, (uint8_t)40, (uint8_t)143, (uint8_t)233, (uint8_t)252, (uint8_t)53, (uint8_t)255, (uint8_t)40, (uint8_t)22, (uint8_t)123, (uint8_t)23, (uint8_t)44, (uint8_t)2, (uint8_t)4} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)244, (uint8_t)102, (uint8_t)221, (uint8_t)186, (uint8_t)223, (uint8_t)144, (uint8_t)98, (uint8_t)111, (uint8_t)38, (uint8_t)229, (uint8_t)132, (uint8_t)210, (uint8_t)20, (uint8_t)63, (uint8_t)110, (uint8_t)86, (uint8_t)214, (uint8_t)40, (uint8_t)114, (uint8_t)210} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)20437);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)21757);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)13215);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -6837);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)6072);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)10277);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -20939);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)1456278183L);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -15875);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -3198);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -23054);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)28061);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)2943);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)13313);
    assert(p27_time_usec_GET(pack) == (uint64_t)8264481687497404204L);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)12784);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -24636);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -28604);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -3677);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -1902);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)20794);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)25944);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)11170);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)31847);
    assert(p28_time_usec_GET(pack) == (uint64_t)7029033341584447061L);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)19618);
    assert(p29_press_abs_GET(pack) == (float) -4.4104404E37F);
    assert(p29_press_diff_GET(pack) == (float) -9.304874E37F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1134866958L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_roll_GET(pack) == (float) -1.3352148E38F);
    assert(p30_yawspeed_GET(pack) == (float)3.0062093E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)1672970808L);
    assert(p30_rollspeed_GET(pack) == (float)9.180455E37F);
    assert(p30_yaw_GET(pack) == (float)2.20606E38F);
    assert(p30_pitch_GET(pack) == (float) -1.1532459E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -4.5974325E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float) -2.8398773E38F);
    assert(p31_rollspeed_GET(pack) == (float)1.2233485E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.7625937E38F);
    assert(p31_yawspeed_GET(pack) == (float) -1.1819234E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)4082374940L);
    assert(p31_q2_GET(pack) == (float) -3.2696157E38F);
    assert(p31_q4_GET(pack) == (float)4.1276995E37F);
    assert(p31_q1_GET(pack) == (float) -3.3821741E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vz_GET(pack) == (float) -1.6738869E38F);
    assert(p32_y_GET(pack) == (float)1.7605083E38F);
    assert(p32_x_GET(pack) == (float) -1.305126E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)4120268304L);
    assert(p32_vx_GET(pack) == (float)7.0624806E37F);
    assert(p32_vy_GET(pack) == (float) -3.7785625E37F);
    assert(p32_z_GET(pack) == (float) -3.2863773E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_alt_GET(pack) == (int32_t)933052202);
    assert(p33_relative_alt_GET(pack) == (int32_t) -13593537);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -490);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)925935010L);
    assert(p33_lon_GET(pack) == (int32_t)1147802622);
    assert(p33_lat_GET(pack) == (int32_t) -418012940);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)24907);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)63165);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -19792);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -1572);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)25106);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)12421);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -2070);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -12902);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)17404);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -15854);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -32555);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2895359963L);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)226);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1608544495L);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)19885);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)13893);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)42778);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)40618);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)58033);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)61165);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)51535);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)15588);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)60063);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)33849);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)39028);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)19987);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)29387);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)19742);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)19082);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)36393);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)47955);
    assert(p36_time_usec_GET(pack) == (uint32_t)3201523141L);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)37650);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)5804);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)39633);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)60061);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)31306);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)3599);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)49349);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)10884);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -24906);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -9411);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)13999);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO);
    assert(p39_z_GET(pack) == (float) -2.773029E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)47523);
    assert(p39_x_GET(pack) == (float)2.614986E38F);
    assert(p39_param2_GET(pack) == (float)2.4287265E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p39_y_GET(pack) == (float)9.085843E37F);
    assert(p39_param1_GET(pack) == (float)1.8411E38F);
    assert(p39_param4_GET(pack) == (float) -1.2997819E37F);
    assert(p39_param3_GET(pack) == (float) -3.0351297E37F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)25571);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)41345);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)152);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)34309);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)17863);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)38338);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -1365198296);
    assert(p48_time_usec_TRY(ph) == (uint64_t)7695424595866253829L);
    assert(p48_altitude_GET(pack) == (int32_t)500870691);
    assert(p48_longitude_GET(pack) == (int32_t) -2042471914);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)1179382182938005125L);
    assert(p49_altitude_GET(pack) == (int32_t) -1788813891);
    assert(p49_longitude_GET(pack) == (int32_t) -1428138135);
    assert(p49_latitude_GET(pack) == (int32_t) -96624527);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value_max_GET(pack) == (float) -8.649492E37F);
    assert(p50_scale_GET(pack) == (float) -3.2418468E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -13089);
    assert(p50_param_value_min_GET(pack) == (float) -3.0401182E37F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p50_param_value0_GET(pack) == (float)1.6154314E38F);
    assert(p50_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"na";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)41762);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2z_GET(pack) == (float) -6.842973E37F);
    assert(p54_p1x_GET(pack) == (float) -4.9856216E36F);
    assert(p54_p2y_GET(pack) == (float)1.0095332E38F);
    assert(p54_p1y_GET(pack) == (float)2.1807687E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p54_p1z_GET(pack) == (float) -2.4403045E37F);
    assert(p54_p2x_GET(pack) == (float)2.8931904E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p55_p1y_GET(pack) == (float)1.292125E37F);
    assert(p55_p2z_GET(pack) == (float)5.108572E37F);
    assert(p55_p2y_GET(pack) == (float) -1.0422561E38F);
    assert(p55_p1x_GET(pack) == (float) -7.582295E37F);
    assert(p55_p1z_GET(pack) == (float)5.7370986E37F);
    assert(p55_p2x_GET(pack) == (float)3.9700913E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_pitchspeed_GET(pack) == (float)2.8510058E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)2144852225318049785L);
    assert(p61_yawspeed_GET(pack) == (float) -5.469619E37F);
    {
        float exemplary[] =  {2.682231E38F, -1.9974085E38F, -2.537388E38F, -3.297437E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)1.1451788E38F);
    {
        float exemplary[] =  {-2.6407774E38F, -1.8310943E38F, 1.9474598E38F, -2.0609565E38F, 1.0978255E38F, -1.257742E38F, 1.9424228E38F, 2.275525E38F, 1.816117E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_alt_error_GET(pack) == (float) -2.2488284E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -13585);
    assert(p62_nav_pitch_GET(pack) == (float)6.3079856E37F);
    assert(p62_aspd_error_GET(pack) == (float)1.3634263E37F);
    assert(p62_nav_roll_GET(pack) == (float) -5.451845E36F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -5021);
    assert(p62_xtrack_error_GET(pack) == (float) -2.6921246E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)52476);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t)211531339);
    assert(p63_vx_GET(pack) == (float) -3.2168135E38F);
    assert(p63_lat_GET(pack) == (int32_t) -2068750565);
    assert(p63_alt_GET(pack) == (int32_t)925805048);
    {
        float exemplary[] =  {2.364486E37F, 1.5890142E38F, -2.2390208E38F, 1.2336601E38F, -2.1852899E38F, -1.7989986E38F, -2.1826428E38F, -1.8497639E38F, 1.8525257E38F, -8.2000783E37F, -1.3762437E38F, -2.1663774E37F, -6.510152E37F, -2.6536732E38F, 1.1237025E38F, 1.691337E38F, -1.4601223E38F, 3.1783221E38F, -1.354376E38F, -3.2550322E38F, -2.6504398E38F, 2.5306953E37F, 2.732532E38F, -3.0081E38F, -3.4287315E37F, 2.7891875E38F, -1.5597256E38F, 2.3827627E38F, -3.1440893E38F, 3.0836501E38F, 1.6348122E38F, -2.5673208E37F, -2.0447715E38F, 4.7001183E37F, 6.0002755E37F, -9.789147E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_relative_alt_GET(pack) == (int32_t)315462959);
    assert(p63_vz_GET(pack) == (float)8.4272494E37F);
    assert(p63_vy_GET(pack) == (float) -1.3460124E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)2150240240045842683L);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_y_GET(pack) == (float)6.753368E37F);
    assert(p64_z_GET(pack) == (float)2.5174768E38F);
    assert(p64_vz_GET(pack) == (float) -2.7680546E38F);
    assert(p64_vx_GET(pack) == (float) -9.978187E37F);
    {
        float exemplary[] =  {-1.631249E38F, -1.433325E38F, -1.3930306E38F, 3.0396436E38F, 7.2394507E37F, 1.5265495E38F, 3.2204734E38F, 2.8676881E38F, 5.103595E37F, -2.8219797E38F, -4.369845E37F, -3.8006264E37F, -2.0904554E38F, 1.4742114E38F, 3.4643704E37F, -3.1797827E38F, -2.3908915E38F, -1.8366158E38F, 2.4212907E38F, -2.8341966E38F, -1.2210072E38F, 1.6037575E38F, 3.1155805E38F, -1.6267239E38F, 3.200471E38F, 1.2257835E38F, 1.6731687E38F, 2.6999564E38F, -6.6387917E37F, -2.9808897E38F, -2.1626186E38F, -7.245029E37F, 1.091978E38F, -3.0511561E37F, 1.4739452E38F, 2.3621335E37F, 3.2520217E38F, -2.9314469E38F, -2.5881643E38F, -2.2739753E38F, -1.5969954E37F, 2.2660782E36F, 3.0036478E38F, 9.482579E37F, 4.353717E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_ay_GET(pack) == (float) -2.7730897E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)1654117827905533451L);
    assert(p64_ax_GET(pack) == (float) -1.850989E38F);
    assert(p64_vy_GET(pack) == (float)3.1287184E37F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_x_GET(pack) == (float)2.4602624E38F);
    assert(p64_az_GET(pack) == (float) -6.35316E37F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)64808);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)41586);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)4036342742L);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)8646);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)41528);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)40630);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)33065);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)65276);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)44760);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)48346);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)56993);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)56954);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)41839);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)64231);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)44045);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)45227);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)47491);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)43066);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)53162);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)9148);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)65413);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -18188);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)27089);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)28765);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -32103);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)27912);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)66);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)12551);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)17465);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)64446);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)39844);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)42962);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)2417);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)42362);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)37375);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_CONDITION_YAW);
    assert(p73_param4_GET(pack) == (float) -2.8929695E38F);
    assert(p73_param2_GET(pack) == (float) -2.3989184E38F);
    assert(p73_x_GET(pack) == (int32_t)1934610622);
    assert(p73_param3_GET(pack) == (float)6.2004786E37F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)48756);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p73_z_GET(pack) == (float)1.0606379E38F);
    assert(p73_param1_GET(pack) == (float) -2.2804558E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p73_y_GET(pack) == (int32_t) -576801491);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -26454);
    assert(p74_alt_GET(pack) == (float)2.1527004E37F);
    assert(p74_climb_GET(pack) == (float) -1.4089901E38F);
    assert(p74_groundspeed_GET(pack) == (float)1.8647128E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)35986);
    assert(p74_airspeed_GET(pack) == (float)3.2619944E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_z_GET(pack) == (float)1.0552128E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_PANORAMA_CREATE);
    assert(p75_y_GET(pack) == (int32_t)66100510);
    assert(p75_param2_GET(pack) == (float)2.6478592E38F);
    assert(p75_x_GET(pack) == (int32_t) -1245959934);
    assert(p75_param1_GET(pack) == (float) -1.5583509E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p75_param4_GET(pack) == (float) -3.2558277E38F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p75_param3_GET(pack) == (float) -3.1335424E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)24);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param7_GET(pack) == (float) -2.7515892E38F);
    assert(p76_param6_GET(pack) == (float) -2.652674E38F);
    assert(p76_param3_GET(pack) == (float) -2.8262564E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_USER_3);
    assert(p76_param5_GET(pack) == (float) -6.427449E37F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p76_param2_GET(pack) == (float) -3.009936E37F);
    assert(p76_param1_GET(pack) == (float)1.8260963E38F);
    assert(p76_param4_GET(pack) == (float)4.680206E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)12);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)66);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)235);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_CONDITION_LAST);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)57);
    assert(p77_result_param2_TRY(ph) == (int32_t)1331592986);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_pitch_GET(pack) == (float)3.259313E36F);
    assert(p81_roll_GET(pack) == (float) -2.7265791E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)965649189L);
    assert(p81_yaw_GET(pack) == (float)7.1277585E37F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p81_thrust_GET(pack) == (float) -2.8936236E38F);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.663323E38F);
    {
        float exemplary[] =  {-2.7567377E37F, -4.1606904E37F, -2.5716185E38F, -2.0604985E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_roll_rate_GET(pack) == (float) -6.4952596E37F);
    assert(p82_thrust_GET(pack) == (float) -2.1038188E37F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.7465768E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2928638263L);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)207);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float) -8.714316E37F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)2639285890L);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p83_body_yaw_rate_GET(pack) == (float) -3.2264484E38F);
    assert(p83_thrust_GET(pack) == (float)3.1406887E38F);
    {
        float exemplary[] =  {9.979019E37F, 7.7050085E37F, 1.3057248E38F, 1.8285688E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_roll_rate_GET(pack) == (float) -1.1005224E38F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_rate_GET(pack) == (float) -2.05703E38F);
    assert(p84_afz_GET(pack) == (float)5.435867E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p84_y_GET(pack) == (float) -8.473599E37F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p84_x_GET(pack) == (float) -2.4298211E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p84_vx_GET(pack) == (float) -1.9087123E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)46380);
    assert(p84_vy_GET(pack) == (float) -1.019203E38F);
    assert(p84_afy_GET(pack) == (float) -1.1937035E38F);
    assert(p84_vz_GET(pack) == (float) -6.172216E37F);
    assert(p84_afx_GET(pack) == (float)2.9040255E38F);
    assert(p84_yaw_GET(pack) == (float) -6.534373E36F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)4175635791L);
    assert(p84_z_GET(pack) == (float) -2.9479002E38F);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p86_alt_GET(pack) == (float) -9.30516E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)45261);
    assert(p86_yaw_GET(pack) == (float) -2.7295694E37F);
    assert(p86_afx_GET(pack) == (float)9.124705E37F);
    assert(p86_yaw_rate_GET(pack) == (float)7.7330626E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)4225103150L);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p86_afy_GET(pack) == (float) -2.756889E38F);
    assert(p86_vz_GET(pack) == (float)1.1468529E38F);
    assert(p86_afz_GET(pack) == (float) -3.0465412E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)1519567279);
    assert(p86_vx_GET(pack) == (float) -1.200121E38F);
    assert(p86_vy_GET(pack) == (float)2.6874738E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)1014553364);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_alt_GET(pack) == (float) -1.7616987E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p87_afz_GET(pack) == (float)1.6059814E38F);
    assert(p87_vy_GET(pack) == (float)2.6259656E38F);
    assert(p87_vx_GET(pack) == (float)8.2430917E37F);
    assert(p87_lon_int_GET(pack) == (int32_t) -2029287617);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)55289);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)89403356L);
    assert(p87_afx_GET(pack) == (float)2.661108E38F);
    assert(p87_vz_GET(pack) == (float) -4.206204E37F);
    assert(p87_afy_GET(pack) == (float) -6.420202E37F);
    assert(p87_yaw_GET(pack) == (float) -1.5781232E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.9268498E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -1321325930);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3197860420L);
    assert(p89_x_GET(pack) == (float) -2.4457022E38F);
    assert(p89_yaw_GET(pack) == (float)4.69269E37F);
    assert(p89_y_GET(pack) == (float) -2.9060673E38F);
    assert(p89_pitch_GET(pack) == (float)3.3404557E38F);
    assert(p89_z_GET(pack) == (float)7.551368E37F);
    assert(p89_roll_GET(pack) == (float) -3.0683067E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)171);
    assert(p90_pitch_GET(pack) == (float)7.4840575E37F);
    assert(p90_rollspeed_GET(pack) == (float)3.0688927E38F);
    assert(p90_lat_GET(pack) == (int32_t)1983905422);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -5681);
    assert(p90_pitchspeed_GET(pack) == (float) -1.3221695E38F);
    assert(p90_lon_GET(pack) == (int32_t)876226594);
    assert(p90_yaw_GET(pack) == (float)1.2493468E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -25685);
    assert(p90_roll_GET(pack) == (float) -2.3509759E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -10946);
    assert(p90_alt_GET(pack) == (int32_t)1093540743);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)10385);
    assert(p90_time_usec_GET(pack) == (uint64_t)124457987584244553L);
    assert(p90_yawspeed_GET(pack) == (float) -1.4500534E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)17712);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    assert(p91_throttle_GET(pack) == (float)6.3778407E37F);
    assert(p91_aux2_GET(pack) == (float) -9.3942924E36F);
    assert(p91_roll_ailerons_GET(pack) == (float)2.5348205E38F);
    assert(p91_aux3_GET(pack) == (float) -2.9307922E37F);
    assert(p91_aux4_GET(pack) == (float)9.131819E36F);
    assert(p91_aux1_GET(pack) == (float) -2.5184782E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)1.0668024E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -4.1973395E37F);
    assert(p91_time_usec_GET(pack) == (uint64_t)5319421263644823112L);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)46825);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)2700);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)34094);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)24820);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)59035);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)27796);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)61795);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)35699);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)58772);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)62196);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)13468);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)10636);
    assert(p92_time_usec_GET(pack) == (uint64_t)2578351291580659787L);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)87672674486249522L);
    {
        float exemplary[] =  {7.553762E37F, 1.5830987E38F, 2.3934902E38F, 3.060731E38F, -3.2026963E38F, 3.1636857E38F, 1.1598541E38F, -1.6381032E38F, 8.846658E37F, 2.8108501E38F, -1.1019242E38F, 1.7672964E38F, 4.4532905E37F, -2.3768278E38F, 8.378375E37F, -1.6903183E36F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)3650995472913210785L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)29684);
    assert(p100_ground_distance_GET(pack) == (float) -3.2858003E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.773371E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -25800);
    assert(p100_time_usec_GET(pack) == (uint64_t)8369373042498735927L);
    assert(p100_flow_rate_x_TRY(ph) == (float) -3.2568262E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)1.4826549E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float) -1.134021E38F);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_usec_GET(pack) == (uint64_t)8399314785562518344L);
    assert(p101_roll_GET(pack) == (float) -3.9043578E37F);
    assert(p101_yaw_GET(pack) == (float)5.509464E36F);
    assert(p101_pitch_GET(pack) == (float) -3.2479876E38F);
    assert(p101_x_GET(pack) == (float) -5.8924933E37F);
    assert(p101_y_GET(pack) == (float) -1.9296504E38F);
    assert(p101_z_GET(pack) == (float)2.738303E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float) -8.2824036E37F);
    assert(p102_pitch_GET(pack) == (float) -1.668868E38F);
    assert(p102_yaw_GET(pack) == (float)1.5270555E38F);
    assert(p102_y_GET(pack) == (float) -3.1870166E38F);
    assert(p102_roll_GET(pack) == (float) -2.0582498E38F);
    assert(p102_usec_GET(pack) == (uint64_t)2603630122814068226L);
    assert(p102_x_GET(pack) == (float) -2.2787364E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float)2.2553145E38F);
    assert(p103_usec_GET(pack) == (uint64_t)3072414355446520441L);
    assert(p103_z_GET(pack) == (float) -2.9248105E38F);
    assert(p103_y_GET(pack) == (float)4.401698E37F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_y_GET(pack) == (float) -4.0174834E37F);
    assert(p104_pitch_GET(pack) == (float) -1.3233016E37F);
    assert(p104_usec_GET(pack) == (uint64_t)8515427990926803105L);
    assert(p104_z_GET(pack) == (float) -2.1742376E38F);
    assert(p104_yaw_GET(pack) == (float) -2.6711775E38F);
    assert(p104_roll_GET(pack) == (float)5.7385873E37F);
    assert(p104_x_GET(pack) == (float) -2.3404006E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)57636);
    assert(p105_diff_pressure_GET(pack) == (float)1.5966447E37F);
    assert(p105_zgyro_GET(pack) == (float) -2.5879164E38F);
    assert(p105_xmag_GET(pack) == (float)2.9246022E38F);
    assert(p105_xgyro_GET(pack) == (float)2.5883959E38F);
    assert(p105_ymag_GET(pack) == (float) -1.9801309E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -5.8666946E37F);
    assert(p105_temperature_GET(pack) == (float) -1.5752714E38F);
    assert(p105_zmag_GET(pack) == (float)1.4931904E38F);
    assert(p105_yacc_GET(pack) == (float) -1.3439534E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -1.5385457E38F);
    assert(p105_xacc_GET(pack) == (float)1.1782019E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4101463850505522576L);
    assert(p105_zacc_GET(pack) == (float)1.5409481E38F);
    assert(p105_ygyro_GET(pack) == (float) -5.9616173E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)300786032L);
    assert(p106_integrated_x_GET(pack) == (float) -2.1543438E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float)5.020694E37F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -9.893342E37F);
    assert(p106_integrated_y_GET(pack) == (float) -1.6237024E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.6163753E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)5742230153494084550L);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p106_distance_GET(pack) == (float) -4.7138363E37F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)26929);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)387601100L);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float)5.401285E36F);
    assert(p107_xmag_GET(pack) == (float)5.667068E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)4237697076721673836L);
    assert(p107_zacc_GET(pack) == (float) -2.0610773E37F);
    assert(p107_ygyro_GET(pack) == (float) -1.8395069E38F);
    assert(p107_zgyro_GET(pack) == (float)3.3551736E38F);
    assert(p107_yacc_GET(pack) == (float) -5.0136935E37F);
    assert(p107_diff_pressure_GET(pack) == (float)2.743232E38F);
    assert(p107_ymag_GET(pack) == (float)1.1284062E38F);
    assert(p107_abs_pressure_GET(pack) == (float)2.8820095E38F);
    assert(p107_xacc_GET(pack) == (float) -1.001909E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -3.1703372E38F);
    assert(p107_xgyro_GET(pack) == (float) -3.2418647E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1217442050L);
    assert(p107_temperature_GET(pack) == (float) -2.5988087E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_vd_GET(pack) == (float)3.124242E38F);
    assert(p108_alt_GET(pack) == (float)1.4719493E38F);
    assert(p108_q4_GET(pack) == (float)9.328354E37F);
    assert(p108_yacc_GET(pack) == (float) -2.3961576E38F);
    assert(p108_roll_GET(pack) == (float) -1.3328574E38F);
    assert(p108_q3_GET(pack) == (float)2.7046301E38F);
    assert(p108_q1_GET(pack) == (float) -1.7509023E38F);
    assert(p108_ygyro_GET(pack) == (float) -2.5701012E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -2.5652269E38F);
    assert(p108_q2_GET(pack) == (float)1.715343E38F);
    assert(p108_zgyro_GET(pack) == (float) -1.6139322E37F);
    assert(p108_vn_GET(pack) == (float)1.4135707E38F);
    assert(p108_zacc_GET(pack) == (float) -2.7327665E38F);
    assert(p108_lat_GET(pack) == (float)2.6722213E38F);
    assert(p108_yaw_GET(pack) == (float) -2.0378771E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)8.985911E37F);
    assert(p108_lon_GET(pack) == (float)2.9779957E37F);
    assert(p108_xacc_GET(pack) == (float) -8.772037E37F);
    assert(p108_ve_GET(pack) == (float)3.2188797E38F);
    assert(p108_pitch_GET(pack) == (float) -2.2379998E38F);
    assert(p108_xgyro_GET(pack) == (float) -8.799285E37F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)56893);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)51048);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)122, (uint8_t)221, (uint8_t)215, (uint8_t)157, (uint8_t)24, (uint8_t)204, (uint8_t)158, (uint8_t)23, (uint8_t)244, (uint8_t)10, (uint8_t)33, (uint8_t)233, (uint8_t)80, (uint8_t)233, (uint8_t)106, (uint8_t)249, (uint8_t)121, (uint8_t)16, (uint8_t)52, (uint8_t)209, (uint8_t)154, (uint8_t)177, (uint8_t)72, (uint8_t)38, (uint8_t)79, (uint8_t)9, (uint8_t)150, (uint8_t)251, (uint8_t)119, (uint8_t)248, (uint8_t)192, (uint8_t)60, (uint8_t)108, (uint8_t)136, (uint8_t)202, (uint8_t)29, (uint8_t)197, (uint8_t)221, (uint8_t)37, (uint8_t)190, (uint8_t)109, (uint8_t)114, (uint8_t)36, (uint8_t)251, (uint8_t)243, (uint8_t)173, (uint8_t)106, (uint8_t)242, (uint8_t)219, (uint8_t)96, (uint8_t)207, (uint8_t)111, (uint8_t)243, (uint8_t)254, (uint8_t)35, (uint8_t)127, (uint8_t)69, (uint8_t)94, (uint8_t)168, (uint8_t)26, (uint8_t)133, (uint8_t)168, (uint8_t)42, (uint8_t)255, (uint8_t)122, (uint8_t)91, (uint8_t)92, (uint8_t)215, (uint8_t)245, (uint8_t)166, (uint8_t)49, (uint8_t)101, (uint8_t)56, (uint8_t)231, (uint8_t)91, (uint8_t)125, (uint8_t)234, (uint8_t)10, (uint8_t)136, (uint8_t)141, (uint8_t)96, (uint8_t)220, (uint8_t)10, (uint8_t)191, (uint8_t)47, (uint8_t)68, (uint8_t)182, (uint8_t)2, (uint8_t)187, (uint8_t)175, (uint8_t)154, (uint8_t)213, (uint8_t)73, (uint8_t)237, (uint8_t)92, (uint8_t)194, (uint8_t)161, (uint8_t)199, (uint8_t)138, (uint8_t)196, (uint8_t)43, (uint8_t)140, (uint8_t)115, (uint8_t)48, (uint8_t)60, (uint8_t)208, (uint8_t)124, (uint8_t)40, (uint8_t)64, (uint8_t)43, (uint8_t)129, (uint8_t)93, (uint8_t)0, (uint8_t)1, (uint8_t)5, (uint8_t)227, (uint8_t)169, (uint8_t)26, (uint8_t)194, (uint8_t)204, (uint8_t)240, (uint8_t)243, (uint8_t)121, (uint8_t)146, (uint8_t)153, (uint8_t)172, (uint8_t)230, (uint8_t)38, (uint8_t)217, (uint8_t)200, (uint8_t)99, (uint8_t)178, (uint8_t)65, (uint8_t)247, (uint8_t)85, (uint8_t)179, (uint8_t)237, (uint8_t)2, (uint8_t)185, (uint8_t)100, (uint8_t)54, (uint8_t)237, (uint8_t)99, (uint8_t)65, (uint8_t)30, (uint8_t)15, (uint8_t)57, (uint8_t)92, (uint8_t)160, (uint8_t)248, (uint8_t)123, (uint8_t)141, (uint8_t)196, (uint8_t)6, (uint8_t)129, (uint8_t)144, (uint8_t)209, (uint8_t)17, (uint8_t)185, (uint8_t)51, (uint8_t)213, (uint8_t)123, (uint8_t)42, (uint8_t)62, (uint8_t)181, (uint8_t)123, (uint8_t)171, (uint8_t)137, (uint8_t)60, (uint8_t)193, (uint8_t)61, (uint8_t)185, (uint8_t)66, (uint8_t)184, (uint8_t)142, (uint8_t)146, (uint8_t)91, (uint8_t)209, (uint8_t)79, (uint8_t)168, (uint8_t)137, (uint8_t)101, (uint8_t)121, (uint8_t)19, (uint8_t)245, (uint8_t)235, (uint8_t)5, (uint8_t)175, (uint8_t)135, (uint8_t)55, (uint8_t)243, (uint8_t)224, (uint8_t)201, (uint8_t)251, (uint8_t)56, (uint8_t)245, (uint8_t)218, (uint8_t)84, (uint8_t)247, (uint8_t)132, (uint8_t)6, (uint8_t)7, (uint8_t)190, (uint8_t)92, (uint8_t)223, (uint8_t)92, (uint8_t)191, (uint8_t)89, (uint8_t)72, (uint8_t)152, (uint8_t)32, (uint8_t)153, (uint8_t)35, (uint8_t)62, (uint8_t)129, (uint8_t)206, (uint8_t)163, (uint8_t)45, (uint8_t)131, (uint8_t)113, (uint8_t)60, (uint8_t)16, (uint8_t)163, (uint8_t)253, (uint8_t)55, (uint8_t)30, (uint8_t)114, (uint8_t)106, (uint8_t)33, (uint8_t)19, (uint8_t)253, (uint8_t)216, (uint8_t)84, (uint8_t)221, (uint8_t)121, (uint8_t)20, (uint8_t)158, (uint8_t)249, (uint8_t)195, (uint8_t)76, (uint8_t)111, (uint8_t)184, (uint8_t)88, (uint8_t)27, (uint8_t)72, (uint8_t)147, (uint8_t)232, (uint8_t)117, (uint8_t)66, (uint8_t)95, (uint8_t)58} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)121);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)8659119403300594770L);
    assert(p111_ts1_GET(pack) == (int64_t)2918086017587043568L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)2910297332L);
    assert(p112_time_usec_GET(pack) == (uint64_t)297739231722986962L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lon_GET(pack) == (int32_t)2138381134);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -6537);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)26282);
    assert(p113_lat_GET(pack) == (int32_t)1355557602);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)61652);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)22051);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)56288);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -32059);
    assert(p113_time_usec_GET(pack) == (uint64_t)7361583187616750593L);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)30260);
    assert(p113_alt_GET(pack) == (int32_t)1017717923);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.3479937E38F);
    assert(p114_distance_GET(pack) == (float) -1.3816953E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)18390);
    assert(p114_integrated_x_GET(pack) == (float)2.5170178E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float)1.6721967E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4099773066L);
    assert(p114_integrated_y_GET(pack) == (float) -6.071221E37F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p114_time_usec_GET(pack) == (uint64_t)5490094233126551350L);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)1305184856L);
    assert(p114_integrated_ygyro_GET(pack) == (float) -6.7340585E36F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_rollspeed_GET(pack) == (float) -9.5004336E35F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -22569);
    assert(p115_lat_GET(pack) == (int32_t)558597971);
    assert(p115_pitchspeed_GET(pack) == (float)2.9662779E38F);
    {
        float exemplary[] =  {2.150577E38F, -2.4061532E37F, 5.940041E37F, -6.9653374E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_time_usec_GET(pack) == (uint64_t)8360879872211272766L);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)50829);
    assert(p115_lon_GET(pack) == (int32_t) -1528245611);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)6563);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -14816);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -23099);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)8137);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)8777);
    assert(p115_alt_GET(pack) == (int32_t)470967320);
    assert(p115_yawspeed_GET(pack) == (float) -1.1938568E38F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -25542);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)31025);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)21145);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -26041);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)15645);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -20016);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)588);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)3804);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)22741);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)12066);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3835916301L);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)12184);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)16325);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)15937);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)63871);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)43113);
    assert(p118_time_utc_GET(pack) == (uint32_t)623653489L);
    assert(p118_size_GET(pack) == (uint32_t)3792280005L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)33445);
    assert(p119_ofs_GET(pack) == (uint32_t)3768331389L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p119_count_GET(pack) == (uint32_t)1313965482L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)76, (uint8_t)172, (uint8_t)179, (uint8_t)61, (uint8_t)29, (uint8_t)2, (uint8_t)149, (uint8_t)128, (uint8_t)200, (uint8_t)13, (uint8_t)205, (uint8_t)32, (uint8_t)223, (uint8_t)14, (uint8_t)56, (uint8_t)7, (uint8_t)47, (uint8_t)93, (uint8_t)128, (uint8_t)177, (uint8_t)182, (uint8_t)179, (uint8_t)241, (uint8_t)236, (uint8_t)253, (uint8_t)144, (uint8_t)220, (uint8_t)136, (uint8_t)158, (uint8_t)136, (uint8_t)53, (uint8_t)213, (uint8_t)130, (uint8_t)137, (uint8_t)36, (uint8_t)46, (uint8_t)152, (uint8_t)62, (uint8_t)104, (uint8_t)252, (uint8_t)252, (uint8_t)171, (uint8_t)117, (uint8_t)206, (uint8_t)55, (uint8_t)31, (uint8_t)16, (uint8_t)118, (uint8_t)223, (uint8_t)77, (uint8_t)231, (uint8_t)79, (uint8_t)61, (uint8_t)202, (uint8_t)38, (uint8_t)115, (uint8_t)234, (uint8_t)62, (uint8_t)247, (uint8_t)105, (uint8_t)240, (uint8_t)133, (uint8_t)21, (uint8_t)225, (uint8_t)125, (uint8_t)31, (uint8_t)50, (uint8_t)229, (uint8_t)184, (uint8_t)222, (uint8_t)214, (uint8_t)15, (uint8_t)106, (uint8_t)177, (uint8_t)56, (uint8_t)65, (uint8_t)93, (uint8_t)91, (uint8_t)115, (uint8_t)193, (uint8_t)17, (uint8_t)249, (uint8_t)49, (uint8_t)129, (uint8_t)145, (uint8_t)46, (uint8_t)71, (uint8_t)75, (uint8_t)111, (uint8_t)201} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)3512122345L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)1487);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)4);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)16);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)160);
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)145, (uint8_t)167, (uint8_t)236, (uint8_t)183, (uint8_t)223, (uint8_t)1, (uint8_t)6, (uint8_t)50, (uint8_t)137, (uint8_t)179, (uint8_t)113, (uint8_t)176, (uint8_t)17, (uint8_t)10, (uint8_t)28, (uint8_t)133, (uint8_t)108, (uint8_t)32, (uint8_t)102, (uint8_t)243, (uint8_t)162, (uint8_t)66, (uint8_t)39, (uint8_t)180, (uint8_t)92, (uint8_t)190, (uint8_t)222, (uint8_t)229, (uint8_t)9, (uint8_t)146, (uint8_t)106, (uint8_t)229, (uint8_t)12, (uint8_t)255, (uint8_t)154, (uint8_t)231, (uint8_t)236, (uint8_t)249, (uint8_t)104, (uint8_t)97, (uint8_t)67, (uint8_t)7, (uint8_t)96, (uint8_t)92, (uint8_t)173, (uint8_t)199, (uint8_t)183, (uint8_t)140, (uint8_t)202, (uint8_t)109, (uint8_t)23, (uint8_t)16, (uint8_t)6, (uint8_t)45, (uint8_t)78, (uint8_t)58, (uint8_t)194, (uint8_t)112, (uint8_t)217, (uint8_t)93, (uint8_t)16, (uint8_t)58, (uint8_t)11, (uint8_t)99, (uint8_t)155, (uint8_t)47, (uint8_t)165, (uint8_t)109, (uint8_t)87, (uint8_t)57, (uint8_t)111, (uint8_t)36, (uint8_t)212, (uint8_t)41, (uint8_t)155, (uint8_t)92, (uint8_t)58, (uint8_t)198, (uint8_t)162, (uint8_t)68, (uint8_t)239, (uint8_t)185, (uint8_t)100, (uint8_t)184, (uint8_t)61, (uint8_t)224, (uint8_t)226, (uint8_t)74, (uint8_t)83, (uint8_t)213, (uint8_t)95, (uint8_t)89, (uint8_t)231, (uint8_t)227, (uint8_t)46, (uint8_t)48, (uint8_t)24, (uint8_t)171, (uint8_t)235, (uint8_t)48, (uint8_t)131, (uint8_t)165, (uint8_t)163, (uint8_t)217, (uint8_t)57, (uint8_t)150, (uint8_t)99, (uint8_t)54, (uint8_t)73} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_alt_GET(pack) == (int32_t)1140683915);
    assert(p124_lat_GET(pack) == (int32_t) -1257696601);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)22651);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)35319);
    assert(p124_dgps_age_GET(pack) == (uint32_t)2361107573L);
    assert(p124_lon_GET(pack) == (int32_t) -969902343);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)28509);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)51340);
    assert(p124_time_usec_GET(pack) == (uint64_t)1479048031372739085L);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)25504);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID));
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)26630);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)1016690666L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI));
    {
        uint8_t exemplary[] =  {(uint8_t)173, (uint8_t)162, (uint8_t)182, (uint8_t)51, (uint8_t)195, (uint8_t)139, (uint8_t)78, (uint8_t)91, (uint8_t)128, (uint8_t)29, (uint8_t)135, (uint8_t)208, (uint8_t)184, (uint8_t)127, (uint8_t)141, (uint8_t)11, (uint8_t)45, (uint8_t)130, (uint8_t)30, (uint8_t)165, (uint8_t)123, (uint8_t)232, (uint8_t)254, (uint8_t)153, (uint8_t)53, (uint8_t)108, (uint8_t)252, (uint8_t)247, (uint8_t)77, (uint8_t)221, (uint8_t)207, (uint8_t)71, (uint8_t)220, (uint8_t)84, (uint8_t)126, (uint8_t)219, (uint8_t)79, (uint8_t)165, (uint8_t)10, (uint8_t)2, (uint8_t)124, (uint8_t)87, (uint8_t)109, (uint8_t)179, (uint8_t)91, (uint8_t)116, (uint8_t)54, (uint8_t)91, (uint8_t)111, (uint8_t)161, (uint8_t)129, (uint8_t)50, (uint8_t)220, (uint8_t)11, (uint8_t)143, (uint8_t)125, (uint8_t)85, (uint8_t)119, (uint8_t)205, (uint8_t)190, (uint8_t)69, (uint8_t)28, (uint8_t)120, (uint8_t)198, (uint8_t)250, (uint8_t)169, (uint8_t)159, (uint8_t)85, (uint8_t)239, (uint8_t)3} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)1182);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1586554802);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)703433813);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)487519151);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)738851248);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)752014926L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p127_accuracy_GET(pack) == (uint32_t)249153682L);
    assert(p127_tow_GET(pack) == (uint32_t)1203394775L);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)61201);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)169);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)6060);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4151804849L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)50839164);
    assert(p128_accuracy_GET(pack) == (uint32_t)2621767620L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)594893478);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p128_tow_GET(pack) == (uint32_t)3441757651L);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)1985071772);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)1617431880);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -26699);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -28506);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)2264218689L);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)546);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -12821);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)14127);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -511);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -22534);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)16666);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)20249);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)58451);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p130_size_GET(pack) == (uint32_t)4237625042L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)50584);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)27299);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)249, (uint8_t)163, (uint8_t)40, (uint8_t)98, (uint8_t)213, (uint8_t)39, (uint8_t)217, (uint8_t)191, (uint8_t)137, (uint8_t)176, (uint8_t)134, (uint8_t)6, (uint8_t)209, (uint8_t)174, (uint8_t)220, (uint8_t)248, (uint8_t)142, (uint8_t)45, (uint8_t)14, (uint8_t)143, (uint8_t)74, (uint8_t)22, (uint8_t)121, (uint8_t)63, (uint8_t)254, (uint8_t)233, (uint8_t)249, (uint8_t)125, (uint8_t)168, (uint8_t)8, (uint8_t)138, (uint8_t)110, (uint8_t)199, (uint8_t)20, (uint8_t)141, (uint8_t)31, (uint8_t)94, (uint8_t)186, (uint8_t)60, (uint8_t)99, (uint8_t)175, (uint8_t)149, (uint8_t)179, (uint8_t)162, (uint8_t)165, (uint8_t)217, (uint8_t)248, (uint8_t)55, (uint8_t)138, (uint8_t)13, (uint8_t)75, (uint8_t)56, (uint8_t)98, (uint8_t)40, (uint8_t)8, (uint8_t)138, (uint8_t)233, (uint8_t)108, (uint8_t)34, (uint8_t)135, (uint8_t)24, (uint8_t)67, (uint8_t)101, (uint8_t)201, (uint8_t)11, (uint8_t)171, (uint8_t)143, (uint8_t)99, (uint8_t)99, (uint8_t)236, (uint8_t)140, (uint8_t)3, (uint8_t)177, (uint8_t)15, (uint8_t)30, (uint8_t)35, (uint8_t)149, (uint8_t)162, (uint8_t)69, (uint8_t)156, (uint8_t)137, (uint8_t)158, (uint8_t)91, (uint8_t)113, (uint8_t)137, (uint8_t)68, (uint8_t)192, (uint8_t)44, (uint8_t)60, (uint8_t)8, (uint8_t)99, (uint8_t)78, (uint8_t)235, (uint8_t)106, (uint8_t)25, (uint8_t)31, (uint8_t)46, (uint8_t)185, (uint8_t)129, (uint8_t)241, (uint8_t)58, (uint8_t)137, (uint8_t)75, (uint8_t)186, (uint8_t)23, (uint8_t)12, (uint8_t)39, (uint8_t)231, (uint8_t)175, (uint8_t)141, (uint8_t)131, (uint8_t)78, (uint8_t)118, (uint8_t)156, (uint8_t)141, (uint8_t)148, (uint8_t)69, (uint8_t)118, (uint8_t)5, (uint8_t)1, (uint8_t)195, (uint8_t)219, (uint8_t)154, (uint8_t)27, (uint8_t)207, (uint8_t)251, (uint8_t)159, (uint8_t)50, (uint8_t)225, (uint8_t)227, (uint8_t)199, (uint8_t)49, (uint8_t)147, (uint8_t)28, (uint8_t)40, (uint8_t)109, (uint8_t)225, (uint8_t)82, (uint8_t)176, (uint8_t)44, (uint8_t)172, (uint8_t)187, (uint8_t)255, (uint8_t)153, (uint8_t)248, (uint8_t)38, (uint8_t)41, (uint8_t)222, (uint8_t)149, (uint8_t)216, (uint8_t)143, (uint8_t)230, (uint8_t)214, (uint8_t)25, (uint8_t)68, (uint8_t)233, (uint8_t)198, (uint8_t)3, (uint8_t)0, (uint8_t)82, (uint8_t)207, (uint8_t)14, (uint8_t)232, (uint8_t)164, (uint8_t)204, (uint8_t)233, (uint8_t)94, (uint8_t)168, (uint8_t)59, (uint8_t)157, (uint8_t)33, (uint8_t)95, (uint8_t)198, (uint8_t)243, (uint8_t)141, (uint8_t)122, (uint8_t)223, (uint8_t)163, (uint8_t)6, (uint8_t)135, (uint8_t)137, (uint8_t)215, (uint8_t)180, (uint8_t)246, (uint8_t)208, (uint8_t)140, (uint8_t)213, (uint8_t)175, (uint8_t)78, (uint8_t)44, (uint8_t)106, (uint8_t)40, (uint8_t)242, (uint8_t)244, (uint8_t)65, (uint8_t)37, (uint8_t)157, (uint8_t)155, (uint8_t)133, (uint8_t)214, (uint8_t)142, (uint8_t)172, (uint8_t)119, (uint8_t)174, (uint8_t)129, (uint8_t)205, (uint8_t)30, (uint8_t)13, (uint8_t)64, (uint8_t)43, (uint8_t)238, (uint8_t)57, (uint8_t)1, (uint8_t)226, (uint8_t)194, (uint8_t)205, (uint8_t)248, (uint8_t)173, (uint8_t)86, (uint8_t)173, (uint8_t)213, (uint8_t)195, (uint8_t)73, (uint8_t)132, (uint8_t)77, (uint8_t)21, (uint8_t)250, (uint8_t)164, (uint8_t)28, (uint8_t)40, (uint8_t)201, (uint8_t)200, (uint8_t)228, (uint8_t)234, (uint8_t)58, (uint8_t)177, (uint8_t)252, (uint8_t)33, (uint8_t)45, (uint8_t)216, (uint8_t)113, (uint8_t)142, (uint8_t)42, (uint8_t)20, (uint8_t)80, (uint8_t)173, (uint8_t)12, (uint8_t)212, (uint8_t)13, (uint8_t)130, (uint8_t)10, (uint8_t)3, (uint8_t)209} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)38364);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)39361);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)47144);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1757479355L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)30766);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)117);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lon_GET(pack) == (int32_t)1151851511);
    assert(p133_lat_GET(pack) == (int32_t) -1086956974);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)6139);
    assert(p133_mask_GET(pack) == (uint64_t)1341659818993430521L);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)20808);
    assert(p134_lon_GET(pack) == (int32_t) -601745621);
    {
        int16_t exemplary[] =  {(int16_t)24426, (int16_t) -18106, (int16_t)8001, (int16_t) -9786, (int16_t)10975, (int16_t)11624, (int16_t)26350, (int16_t)32643, (int16_t) -7936, (int16_t) -18762, (int16_t)25100, (int16_t) -29868, (int16_t)17833, (int16_t)16462, (int16_t) -17988, (int16_t) -31281} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lat_GET(pack) == (int32_t)2108905097);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)120);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)2032225830);
    assert(p135_lat_GET(pack) == (int32_t) -1594904275);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)60893);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)5623);
    assert(p136_current_height_GET(pack) == (float) -3.10456E38F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)35256);
    assert(p136_terrain_height_GET(pack) == (float)2.2009664E38F);
    assert(p136_lat_GET(pack) == (int32_t) -942876852);
    assert(p136_lon_GET(pack) == (int32_t) -292312307);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)4022888425L);
    assert(p137_press_diff_GET(pack) == (float) -1.9966732E38F);
    assert(p137_press_abs_GET(pack) == (float) -3.0553065E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -17819);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)839929048415957882L);
    assert(p138_y_GET(pack) == (float) -3.358048E37F);
    assert(p138_z_GET(pack) == (float)3.8530212E37F);
    {
        float exemplary[] =  {2.6538578E38F, 1.458672E36F, 2.8352578E38F, -1.4093457E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float) -3.1706653E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p139_time_usec_GET(pack) == (uint64_t)7007766848845000456L);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)47);
    {
        float exemplary[] =  {2.1671676E37F, -2.3999457E38F, -3.185691E38F, 1.7666171E38F, 9.979029E36F, -9.340243E37F, -1.9494302E38F, -1.646043E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.1928646E38F, -2.3718047E38F, 6.5785874E37F, -3.0729761E38F, 1.3454329E38F, 2.1389937E38F, -1.8533855E38F, -8.77791E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)52000896839502531L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)184);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_monotonic_GET(pack) == (float) -1.4594012E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)1.6294973E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)1659242072199543845L);
    assert(p141_bottom_clearance_GET(pack) == (float) -8.408816E37F);
    assert(p141_altitude_local_GET(pack) == (float) -3.3333597E38F);
    assert(p141_altitude_relative_GET(pack) == (float)1.428131E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -6.4143906E36F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)23);
    {
        uint8_t exemplary[] =  {(uint8_t)227, (uint8_t)198, (uint8_t)22, (uint8_t)151, (uint8_t)109, (uint8_t)105, (uint8_t)115, (uint8_t)233, (uint8_t)29, (uint8_t)76, (uint8_t)194, (uint8_t)29, (uint8_t)33, (uint8_t)191, (uint8_t)98, (uint8_t)74, (uint8_t)135, (uint8_t)188, (uint8_t)91, (uint8_t)95, (uint8_t)243, (uint8_t)172, (uint8_t)41, (uint8_t)51, (uint8_t)139, (uint8_t)146, (uint8_t)46, (uint8_t)6, (uint8_t)15, (uint8_t)56, (uint8_t)236, (uint8_t)46, (uint8_t)123, (uint8_t)180, (uint8_t)97, (uint8_t)231, (uint8_t)250, (uint8_t)106, (uint8_t)95, (uint8_t)186, (uint8_t)120, (uint8_t)177, (uint8_t)106, (uint8_t)233, (uint8_t)119, (uint8_t)88, (uint8_t)23, (uint8_t)219, (uint8_t)13, (uint8_t)32, (uint8_t)146, (uint8_t)45, (uint8_t)160, (uint8_t)67, (uint8_t)252, (uint8_t)97, (uint8_t)185, (uint8_t)199, (uint8_t)150, (uint8_t)68, (uint8_t)107, (uint8_t)186, (uint8_t)200, (uint8_t)144, (uint8_t)235, (uint8_t)188, (uint8_t)25, (uint8_t)157, (uint8_t)42, (uint8_t)223, (uint8_t)86, (uint8_t)92, (uint8_t)181, (uint8_t)244, (uint8_t)39, (uint8_t)68, (uint8_t)169, (uint8_t)82, (uint8_t)22, (uint8_t)12, (uint8_t)28, (uint8_t)157, (uint8_t)6, (uint8_t)82, (uint8_t)244, (uint8_t)5, (uint8_t)151, (uint8_t)131, (uint8_t)175, (uint8_t)216, (uint8_t)67, (uint8_t)136, (uint8_t)50, (uint8_t)108, (uint8_t)90, (uint8_t)22, (uint8_t)184, (uint8_t)6, (uint8_t)136, (uint8_t)233, (uint8_t)130, (uint8_t)39, (uint8_t)118, (uint8_t)12, (uint8_t)70, (uint8_t)178, (uint8_t)86, (uint8_t)251, (uint8_t)121, (uint8_t)146, (uint8_t)42, (uint8_t)72, (uint8_t)143, (uint8_t)159, (uint8_t)22, (uint8_t)2, (uint8_t)208, (uint8_t)140, (uint8_t)161, (uint8_t)111} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)39, (uint8_t)25, (uint8_t)22, (uint8_t)43, (uint8_t)238, (uint8_t)249, (uint8_t)27, (uint8_t)15, (uint8_t)148, (uint8_t)54, (uint8_t)127, (uint8_t)156, (uint8_t)157, (uint8_t)135, (uint8_t)160, (uint8_t)158, (uint8_t)140, (uint8_t)185, (uint8_t)71, (uint8_t)121, (uint8_t)247, (uint8_t)7, (uint8_t)185, (uint8_t)207, (uint8_t)102, (uint8_t)201, (uint8_t)15, (uint8_t)200, (uint8_t)226, (uint8_t)238, (uint8_t)233, (uint8_t)185, (uint8_t)20, (uint8_t)24, (uint8_t)218, (uint8_t)71, (uint8_t)97, (uint8_t)78, (uint8_t)206, (uint8_t)100, (uint8_t)223, (uint8_t)149, (uint8_t)63, (uint8_t)20, (uint8_t)174, (uint8_t)133, (uint8_t)116, (uint8_t)83, (uint8_t)31, (uint8_t)195, (uint8_t)136, (uint8_t)224, (uint8_t)110, (uint8_t)206, (uint8_t)98, (uint8_t)141, (uint8_t)187, (uint8_t)192, (uint8_t)194, (uint8_t)114, (uint8_t)109, (uint8_t)103, (uint8_t)65, (uint8_t)254, (uint8_t)81, (uint8_t)110, (uint8_t)54, (uint8_t)165, (uint8_t)36, (uint8_t)123, (uint8_t)28, (uint8_t)38, (uint8_t)73, (uint8_t)184, (uint8_t)26, (uint8_t)48, (uint8_t)58, (uint8_t)204, (uint8_t)242, (uint8_t)22, (uint8_t)114, (uint8_t)167, (uint8_t)133, (uint8_t)7, (uint8_t)119, (uint8_t)140, (uint8_t)122, (uint8_t)116, (uint8_t)46, (uint8_t)74, (uint8_t)121, (uint8_t)91, (uint8_t)111, (uint8_t)6, (uint8_t)96, (uint8_t)242, (uint8_t)58, (uint8_t)4, (uint8_t)99, (uint8_t)141, (uint8_t)197, (uint8_t)58, (uint8_t)176, (uint8_t)27, (uint8_t)170, (uint8_t)64, (uint8_t)227, (uint8_t)23, (uint8_t)80, (uint8_t)106, (uint8_t)167, (uint8_t)242, (uint8_t)121, (uint8_t)46, (uint8_t)212, (uint8_t)191, (uint8_t)86, (uint8_t)48, (uint8_t)175, (uint8_t)79} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)1.7927021E38F);
    assert(p143_press_diff_GET(pack) == (float)2.9850507E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)2145);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)4044814564L);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.5706717E38F, -3.0066502E38F, -2.6188266E38F, -3.2808438E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)2229766762357037528L);
    {
        float exemplary[] =  {-1.2611791E38F, -2.2535998E38F, -3.3308633E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)3.1193137E38F);
    {
        float exemplary[] =  {2.7129729E38F, -3.1803202E38F, -5.605886E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)2532045347439496764L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)92);
    {
        float exemplary[] =  {-7.6297096E37F, -1.0794055E37F, 1.6980832E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.7172172E38F, 9.805553E37F, 1.5239787E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -367228611);
    assert(p144_lon_GET(pack) == (int32_t)1870450080);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_z_vel_GET(pack) == (float)1.5645908E38F);
    assert(p146_x_vel_GET(pack) == (float) -3.398022E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.4582325E38F);
    {
        float exemplary[] =  {-1.616339E38F, -3.1587916E38F, -1.4472298E38F, 2.3064278E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float)2.1895904E38F);
    assert(p146_roll_rate_GET(pack) == (float) -3.029926E38F);
    assert(p146_airspeed_GET(pack) == (float) -1.936391E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)5943053024702942339L);
    assert(p146_z_acc_GET(pack) == (float) -3.1456214E38F);
    {
        float exemplary[] =  {-3.344565E36F, -4.755511E37F, -6.698451E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float)9.0160565E36F);
    assert(p146_y_acc_GET(pack) == (float) -7.145255E36F);
    assert(p146_x_pos_GET(pack) == (float) -2.4112706E38F);
    assert(p146_y_pos_GET(pack) == (float)1.3048024E38F);
    assert(p146_z_pos_GET(pack) == (float)1.7984183E38F);
    {
        float exemplary[] =  {4.964177E37F, -2.9974884E38F, -1.885777E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_acc_GET(pack) == (float) -5.1510845E37F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -22096);
    assert(p147_energy_consumed_GET(pack) == (int32_t)389849567);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_current_consumed_GET(pack) == (int32_t)1240343418);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -5860);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)104);
    {
        uint16_t exemplary[] =  {(uint16_t)47199, (uint16_t)17683, (uint16_t)11456, (uint16_t)20671, (uint16_t)18215, (uint16_t)10922, (uint16_t)28316, (uint16_t)53461, (uint16_t)29626, (uint16_t)15810} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)695412048L);
    {
        uint8_t exemplary[] =  {(uint8_t)207, (uint8_t)109, (uint8_t)225, (uint8_t)234, (uint8_t)231, (uint8_t)136, (uint8_t)126, (uint8_t)175} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)41596);
    {
        uint8_t exemplary[] =  {(uint8_t)126, (uint8_t)80, (uint8_t)104, (uint8_t)23, (uint8_t)60, (uint8_t)249, (uint8_t)147, (uint8_t)41} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT));
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)33348);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2387807743L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)2449564781L);
    assert(p148_board_version_GET(pack) == (uint32_t)2397672179L);
    assert(p148_uid_GET(pack) == (uint64_t)5638697991911850844L);
    {
        uint8_t exemplary[] =  {(uint8_t)246, (uint8_t)224, (uint8_t)65, (uint8_t)84, (uint8_t)38, (uint8_t)141, (uint8_t)8, (uint8_t)11} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)213, (uint8_t)193, (uint8_t)57, (uint8_t)61, (uint8_t)138, (uint8_t)209, (uint8_t)17, (uint8_t)184, (uint8_t)75, (uint8_t)5, (uint8_t)10, (uint8_t)7, (uint8_t)154, (uint8_t)234, (uint8_t)40, (uint8_t)241, (uint8_t)241, (uint8_t)104} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_time_usec_GET(pack) == (uint64_t)7768412383148311239L);
    assert(p149_angle_x_GET(pack) == (float)3.116952E38F);
    {
        float exemplary[] =  {-1.2529333E38F, -9.347312E37F, -5.9791377E37F, 2.9348933E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_distance_GET(pack) == (float) -2.204709E38F);
    assert(p149_x_TRY(ph) == (float) -1.354389E38F);
    assert(p149_angle_y_GET(pack) == (float) -2.4744043E38F);
    assert(p149_y_TRY(ph) == (float)3.1520678E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p149_size_x_GET(pack) == (float)3.3777145E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)203);
    assert(p149_z_TRY(ph) == (float)9.824239E37F);
    assert(p149_size_y_GET(pack) == (float) -3.3769675E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_target_component_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p150_target_system_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_data_index_GET(pack) == (int16_t)(int16_t) -29336);
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p151_read_req_type_GET(pack) == (int16_t)(int16_t)14039);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)137);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_func_count_GET(pack) == (uint16_t)(uint16_t)6592);
    assert(p152_data_size_GET(pack) == (uint16_t)(uint16_t)65258);
    assert(p152_data_address_GET(pack) == (uint16_t)(uint16_t)956);
    assert(p152_target_component_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p152_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p152_func_index_GET(pack) == (uint16_t)(uint16_t)58208);
    {
        int8_t exemplary[] =  {(int8_t) -13, (int8_t)111, (int8_t) -67, (int8_t)3, (int8_t) -77, (int8_t)6, (int8_t) -111, (int8_t) -5, (int8_t)88, (int8_t) -13, (int8_t) -125, (int8_t) -71, (int8_t) -43, (int8_t) -89, (int8_t)97, (int8_t)72, (int8_t)43, (int8_t) -49, (int8_t)43, (int8_t)6, (int8_t) -31, (int8_t)109, (int8_t)99, (int8_t)73, (int8_t) -60, (int8_t) -94, (int8_t)20, (int8_t) -47, (int8_t)71, (int8_t) -52, (int8_t) -26, (int8_t)9, (int8_t)16, (int8_t) -3, (int8_t)88, (int8_t)20, (int8_t)87, (int8_t)112, (int8_t)24, (int8_t) -87, (int8_t) -13, (int8_t) -27, (int8_t)42, (int8_t) -126, (int8_t) -113, (int8_t)94, (int8_t) -126, (int8_t) -101} ;
        int8_t*  sample = p152_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p153_target_component_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p153_func_index_GET(pack) == (uint16_t)(uint16_t)23154);
    assert(p153_result_GET(pack) == (uint16_t)(uint16_t)36893);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_count_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p155_directory_type_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p155_start_index_GET(pack) == (uint8_t)(uint8_t)71);
    {
        int8_t exemplary[] =  {(int8_t) -127, (int8_t)85, (int8_t)94, (int8_t)1, (int8_t)4, (int8_t) -6, (int8_t)68, (int8_t)67, (int8_t) -11, (int8_t) -32, (int8_t)99, (int8_t) -48, (int8_t)7, (int8_t) -94, (int8_t)52, (int8_t)70, (int8_t)37, (int8_t)102, (int8_t) -57, (int8_t)98, (int8_t)78, (int8_t) -122, (int8_t)45, (int8_t)64, (int8_t) -118, (int8_t) -17, (int8_t) -113, (int8_t) -10, (int8_t)56, (int8_t) -111, (int8_t) -36, (int8_t) -57, (int8_t)125, (int8_t) -65, (int8_t)6, (int8_t) -88, (int8_t) -102, (int8_t) -84, (int8_t)60, (int8_t) -44, (int8_t)20, (int8_t)58, (int8_t)125, (int8_t) -58, (int8_t)29, (int8_t) -125, (int8_t)50, (int8_t) -8} ;
        int8_t*  sample = p155_directory_data_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)118);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_result_GET(pack) == (uint16_t)(uint16_t)19646);
    assert(p156_count_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p156_start_index_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p156_directory_type_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_command_type_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)21);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_result_GET(pack) == (uint16_t)(uint16_t)56774);
    assert(p158_command_type_GET(pack) == (uint16_t)(uint16_t)11328);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_sue_rmat5_GET(pack) == (int16_t)(int16_t)3727);
    assert(p170_sue_magFieldEarth1_GET(pack) == (int16_t)(int16_t)23175);
    assert(p170_sue_waypoint_index_GET(pack) == (uint16_t)(uint16_t)12823);
    assert(p170_sue_air_speed_3DIMU_GET(pack) == (uint16_t)(uint16_t)65095);
    assert(p170_sue_estimated_wind_2_GET(pack) == (int16_t)(int16_t) -25725);
    assert(p170_sue_longitude_GET(pack) == (int32_t)645671859);
    assert(p170_sue_rmat1_GET(pack) == (int16_t)(int16_t) -1432);
    assert(p170_sue_rmat6_GET(pack) == (int16_t)(int16_t)4717);
    assert(p170_sue_status_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p170_sue_rmat3_GET(pack) == (int16_t)(int16_t)3111);
    assert(p170_sue_cpu_load_GET(pack) == (uint16_t)(uint16_t)35421);
    assert(p170_sue_rmat8_GET(pack) == (int16_t)(int16_t) -9003);
    assert(p170_sue_estimated_wind_1_GET(pack) == (int16_t)(int16_t) -20861);
    assert(p170_sue_rmat0_GET(pack) == (int16_t)(int16_t) -28396);
    assert(p170_sue_svs_GET(pack) == (int16_t)(int16_t) -4963);
    assert(p170_sue_hdop_GET(pack) == (int16_t)(int16_t) -5693);
    assert(p170_sue_magFieldEarth0_GET(pack) == (int16_t)(int16_t)10695);
    assert(p170_sue_rmat7_GET(pack) == (int16_t)(int16_t)6225);
    assert(p170_sue_time_GET(pack) == (uint32_t)2670418949L);
    assert(p170_sue_altitude_GET(pack) == (int32_t)411183183);
    assert(p170_sue_rmat4_GET(pack) == (int16_t)(int16_t) -3328);
    assert(p170_sue_latitude_GET(pack) == (int32_t)1760906038);
    assert(p170_sue_rmat2_GET(pack) == (int16_t)(int16_t) -19467);
    assert(p170_sue_cog_GET(pack) == (uint16_t)(uint16_t)22745);
    assert(p170_sue_magFieldEarth2_GET(pack) == (int16_t)(int16_t) -4606);
    assert(p170_sue_sog_GET(pack) == (int16_t)(int16_t) -10243);
    assert(p170_sue_estimated_wind_0_GET(pack) == (int16_t)(int16_t)1282);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_sue_pwm_input_9_GET(pack) == (int16_t)(int16_t)15402);
    assert(p171_sue_pwm_output_10_GET(pack) == (int16_t)(int16_t)14673);
    assert(p171_sue_waypoint_goal_z_GET(pack) == (int16_t)(int16_t)16762);
    assert(p171_sue_aero_z_GET(pack) == (int16_t)(int16_t) -10995);
    assert(p171_sue_aero_y_GET(pack) == (int16_t)(int16_t) -2);
    assert(p171_sue_flags_GET(pack) == (uint32_t)1118076292L);
    assert(p171_sue_pwm_input_4_GET(pack) == (int16_t)(int16_t)27318);
    assert(p171_sue_imu_location_z_GET(pack) == (int16_t)(int16_t) -23824);
    assert(p171_sue_pwm_input_3_GET(pack) == (int16_t)(int16_t)28841);
    assert(p171_sue_pwm_input_11_GET(pack) == (int16_t)(int16_t)3778);
    assert(p171_sue_pwm_output_11_GET(pack) == (int16_t)(int16_t)4206);
    assert(p171_sue_waypoint_goal_x_GET(pack) == (int16_t)(int16_t) -30927);
    assert(p171_sue_barom_press_GET(pack) == (int32_t) -1852177511);
    assert(p171_sue_imu_location_x_GET(pack) == (int16_t)(int16_t)20688);
    assert(p171_sue_waypoint_goal_y_GET(pack) == (int16_t)(int16_t)8294);
    assert(p171_sue_pwm_output_12_GET(pack) == (int16_t)(int16_t) -11958);
    assert(p171_sue_pwm_input_7_GET(pack) == (int16_t)(int16_t) -26931);
    assert(p171_sue_imu_location_y_GET(pack) == (int16_t)(int16_t) -29509);
    assert(p171_sue_location_error_earth_z_GET(pack) == (int16_t)(int16_t)5281);
    assert(p171_sue_pwm_output_5_GET(pack) == (int16_t)(int16_t)21671);
    assert(p171_sue_barom_alt_GET(pack) == (int32_t) -2094213193);
    assert(p171_sue_barom_temp_GET(pack) == (int16_t)(int16_t) -31116);
    assert(p171_sue_location_error_earth_x_GET(pack) == (int16_t)(int16_t)21210);
    assert(p171_sue_imu_velocity_y_GET(pack) == (int16_t)(int16_t)32071);
    assert(p171_sue_bat_amp_GET(pack) == (int16_t)(int16_t) -2457);
    assert(p171_sue_pwm_output_4_GET(pack) == (int16_t)(int16_t) -7348);
    assert(p171_sue_location_error_earth_y_GET(pack) == (int16_t)(int16_t)27158);
    assert(p171_sue_pwm_input_6_GET(pack) == (int16_t)(int16_t)6153);
    assert(p171_sue_imu_velocity_x_GET(pack) == (int16_t)(int16_t)25954);
    assert(p171_sue_pwm_output_2_GET(pack) == (int16_t)(int16_t) -23976);
    assert(p171_sue_pwm_input_2_GET(pack) == (int16_t)(int16_t) -10383);
    assert(p171_sue_pwm_input_5_GET(pack) == (int16_t)(int16_t) -16282);
    assert(p171_sue_pwm_output_6_GET(pack) == (int16_t)(int16_t) -31976);
    assert(p171_sue_pwm_output_8_GET(pack) == (int16_t)(int16_t)14949);
    assert(p171_sue_pwm_input_1_GET(pack) == (int16_t)(int16_t) -22547);
    assert(p171_sue_pwm_output_7_GET(pack) == (int16_t)(int16_t)31266);
    assert(p171_sue_time_GET(pack) == (uint32_t)3265850006L);
    assert(p171_sue_memory_stack_free_GET(pack) == (int16_t)(int16_t) -28170);
    assert(p171_sue_pwm_output_1_GET(pack) == (int16_t)(int16_t) -3734);
    assert(p171_sue_pwm_output_9_GET(pack) == (int16_t)(int16_t)30550);
    assert(p171_sue_bat_volt_GET(pack) == (int16_t)(int16_t)24091);
    assert(p171_sue_imu_velocity_z_GET(pack) == (int16_t)(int16_t) -12036);
    assert(p171_sue_bat_amp_hours_GET(pack) == (int16_t)(int16_t)24133);
    assert(p171_sue_pwm_output_3_GET(pack) == (int16_t)(int16_t)2344);
    assert(p171_sue_aero_x_GET(pack) == (int16_t)(int16_t) -30526);
    assert(p171_sue_desired_height_GET(pack) == (int16_t)(int16_t) -20291);
    assert(p171_sue_pwm_input_10_GET(pack) == (int16_t)(int16_t)783);
    assert(p171_sue_pwm_input_8_GET(pack) == (int16_t)(int16_t) -1628);
    assert(p171_sue_osc_fails_GET(pack) == (int16_t)(int16_t)7325);
    assert(p171_sue_pwm_input_12_GET(pack) == (int16_t)(int16_t)29850);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_sue_ROLL_STABILIZATION_AILERONS_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p172_sue_YAW_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p172_sue_RACING_MODE_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p172_sue_PITCH_STABILIZATION_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p172_sue_AILERON_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p172_sue_ALTITUDEHOLD_WAYPOINT_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p172_sue_YAW_STABILIZATION_AILERON_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p172_sue_ALTITUDEHOLD_STABILIZED_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p172_sue_ROLL_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p172_sue_RUDDER_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)35);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_sue_ROLLKD_GET(pack) == (float)1.0593349E37F);
    assert(p173_sue_ROLLKP_GET(pack) == (float) -1.1622743E38F);
    assert(p173_sue_YAWKD_AILERON_GET(pack) == (float) -8.68622E37F);
    assert(p173_sue_YAWKP_AILERON_GET(pack) == (float)4.579536E37F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_sue_ELEVATOR_BOOST_GET(pack) == (float) -3.2297537E38F);
    assert(p174_sue_PITCHGAIN_GET(pack) == (float) -3.3416633E38F);
    assert(p174_sue_RUDDER_ELEV_MIX_GET(pack) == (float)9.226391E37F);
    assert(p174_sue_ROLL_ELEV_MIX_GET(pack) == (float)1.3124364E38F);
    assert(p174_sue_PITCHKD_GET(pack) == (float)2.9710836E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_sue_YAWKP_RUDDER_GET(pack) == (float)8.816493E37F);
    assert(p175_sue_RUDDER_BOOST_GET(pack) == (float) -3.18434E37F);
    assert(p175_sue_ROLLKP_RUDDER_GET(pack) == (float) -2.9256206E38F);
    assert(p175_sue_YAWKD_RUDDER_GET(pack) == (float)3.0743646E38F);
    assert(p175_sue_ROLLKD_RUDDER_GET(pack) == (float)1.874043E38F);
    assert(p175_sue_RTL_PITCH_DOWN_GET(pack) == (float)1.5029126E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack) == (float) -1.4148535E37F);
    assert(p176_sue_ALT_HOLD_PITCH_MIN_GET(pack) == (float) -1.7382579E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack) == (float)2.400362E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack) == (float)3.101237E38F);
    assert(p176_sue_HEIGHT_TARGET_MIN_GET(pack) == (float)1.2345561E38F);
    assert(p176_sue_HEIGHT_TARGET_MAX_GET(pack) == (float) -4.2112867E37F);
    assert(p176_sue_ALT_HOLD_PITCH_MAX_GET(pack) == (float) -2.094649E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_sue_alt_origin_GET(pack) == (int32_t) -556089570);
    assert(p177_sue_lat_origin_GET(pack) == (int32_t) -274288077);
    assert(p177_sue_lon_origin_GET(pack) == (int32_t)905789817);
    assert(p177_sue_week_no_GET(pack) == (int16_t)(int16_t)20260);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_sue_GPS_TYPE_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p178_sue_DR_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p178_sue_TRAP_SOURCE_GET(pack) == (uint32_t)1362836943L);
    assert(p178_sue_RCON_GET(pack) == (int16_t)(int16_t)5121);
    assert(p178_sue_TRAP_FLAGS_GET(pack) == (int16_t)(int16_t) -31365);
    assert(p178_sue_CLOCK_CONFIG_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p178_sue_FLIGHT_PLAN_TYPE_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p178_sue_AIRFRAME_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p178_sue_WIND_ESTIMATION_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p178_sue_BOARD_TYPE_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p178_sue_osc_fail_count_GET(pack) == (int16_t)(int16_t)3825);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)242, (uint8_t)235, (uint8_t)72, (uint8_t)56, (uint8_t)140, (uint8_t)108, (uint8_t)225, (uint8_t)192, (uint8_t)163, (uint8_t)113, (uint8_t)234, (uint8_t)9, (uint8_t)23, (uint8_t)167, (uint8_t)129, (uint8_t)89, (uint8_t)126, (uint8_t)216, (uint8_t)125, (uint8_t)175} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)194, (uint8_t)251, (uint8_t)23, (uint8_t)168, (uint8_t)246, (uint8_t)89, (uint8_t)33, (uint8_t)9, (uint8_t)195, (uint8_t)61, (uint8_t)242, (uint8_t)129, (uint8_t)19, (uint8_t)177, (uint8_t)64, (uint8_t)86, (uint8_t)232, (uint8_t)212, (uint8_t)105, (uint8_t)174, (uint8_t)195, (uint8_t)251, (uint8_t)12, (uint8_t)188, (uint8_t)154, (uint8_t)158, (uint8_t)19, (uint8_t)100, (uint8_t)38, (uint8_t)100, (uint8_t)127, (uint8_t)66, (uint8_t)180, (uint8_t)113, (uint8_t)112, (uint8_t)252, (uint8_t)57, (uint8_t)239, (uint8_t)28, (uint8_t)91} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)107, (uint8_t)41, (uint8_t)16, (uint8_t)70, (uint8_t)123, (uint8_t)189, (uint8_t)63, (uint8_t)181, (uint8_t)44, (uint8_t)248, (uint8_t)150, (uint8_t)247, (uint8_t)187, (uint8_t)182, (uint8_t)61, (uint8_t)244, (uint8_t)198, (uint8_t)2, (uint8_t)48, (uint8_t)236, (uint8_t)134, (uint8_t)164, (uint8_t)67, (uint8_t)122, (uint8_t)190, (uint8_t)57, (uint8_t)77, (uint8_t)47, (uint8_t)88, (uint8_t)57, (uint8_t)160, (uint8_t)169, (uint8_t)51, (uint8_t)169, (uint8_t)216, (uint8_t)229, (uint8_t)80, (uint8_t)109, (uint8_t)12, (uint8_t)49} ;
        uint8_t*  sample = p180_sue_ID_LEAD_PILOT_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)135, (uint8_t)164, (uint8_t)39, (uint8_t)9, (uint8_t)11, (uint8_t)75, (uint8_t)223, (uint8_t)163, (uint8_t)126, (uint8_t)177, (uint8_t)59, (uint8_t)73, (uint8_t)206, (uint8_t)183, (uint8_t)207, (uint8_t)226, (uint8_t)109, (uint8_t)201, (uint8_t)139, (uint8_t)196, (uint8_t)135, (uint8_t)218, (uint8_t)223, (uint8_t)12, (uint8_t)81, (uint8_t)75, (uint8_t)50, (uint8_t)180, (uint8_t)205, (uint8_t)137, (uint8_t)197, (uint8_t)129, (uint8_t)200, (uint8_t)106, (uint8_t)181, (uint8_t)24, (uint8_t)131, (uint8_t)209, (uint8_t)179, (uint8_t)47, (uint8_t)173, (uint8_t)159, (uint8_t)69, (uint8_t)156, (uint8_t)138, (uint8_t)155, (uint8_t)227, (uint8_t)119, (uint8_t)49, (uint8_t)119, (uint8_t)50, (uint8_t)214, (uint8_t)152, (uint8_t)16, (uint8_t)195, (uint8_t)245, (uint8_t)123, (uint8_t)155, (uint8_t)160, (uint8_t)11, (uint8_t)184, (uint8_t)8, (uint8_t)170, (uint8_t)75, (uint8_t)91, (uint8_t)169, (uint8_t)147, (uint8_t)20, (uint8_t)165} ;
        uint8_t*  sample = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_alt_imu_GET(pack) == (int32_t) -1205856627);
    assert(p181_time_boot_ms_GET(pack) == (uint32_t)4047212822L);
    assert(p181_alt_optical_flow_GET(pack) == (int32_t)736994397);
    assert(p181_alt_range_finder_GET(pack) == (int32_t)14704953);
    assert(p181_alt_barometric_GET(pack) == (int32_t) -28947551);
    assert(p181_alt_extra_GET(pack) == (int32_t) -2119122694);
    assert(p181_alt_gps_GET(pack) == (int32_t)1854661769);
};


void c_CommunicationChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_time_boot_ms_GET(pack) == (uint32_t)2000606745L);
    assert(p182_airspeed_imu_GET(pack) == (int16_t)(int16_t) -21637);
    assert(p182_airspeed_pitot_GET(pack) == (int16_t)(int16_t) -26665);
    assert(p182_aoy_GET(pack) == (int16_t)(int16_t)17904);
    assert(p182_airspeed_hot_wire_GET(pack) == (int16_t)(int16_t) -13934);
    assert(p182_airspeed_ultrasonic_GET(pack) == (int16_t)(int16_t)2076);
    assert(p182_aoa_GET(pack) == (int16_t)(int16_t)16535);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_sue_turn_rate_fbw_GET(pack) == (float) -8.3958223E37F);
    assert(p183_sue_turn_rate_nav_GET(pack) == (float) -1.8538587E38F);
    assert(p183_sue_feed_forward_GET(pack) == (float)3.2580728E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_angle_of_attack_inverted_GET(pack) == (float) -6.5883574E37F);
    assert(p184_elevator_trim_inverted_GET(pack) == (float)3.2226429E38F);
    assert(p184_angle_of_attack_normal_GET(pack) == (float) -4.413074E37F);
    assert(p184_elevator_trim_normal_GET(pack) == (float) -1.5195805E38F);
    assert(p184_reference_speed_GET(pack) == (float) -1.9352962E37F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_sue_rudder_output_channel_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p185_sue_throttle_reversed_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p185_sue_elevator_reversed_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p185_sue_aileron_output_channel_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p185_sue_aileron_reversed_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p185_sue_elevator_output_channel_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p185_sue_throttle_output_channel_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p185_sue_rudder_reversed_GET(pack) == (uint8_t)(uint8_t)91);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_sue_trim_value_input_2_GET(pack) == (int16_t)(int16_t) -10824);
    assert(p186_sue_trim_value_input_11_GET(pack) == (int16_t)(int16_t)3020);
    assert(p186_sue_trim_value_input_10_GET(pack) == (int16_t)(int16_t) -7419);
    assert(p186_sue_trim_value_input_9_GET(pack) == (int16_t)(int16_t)30536);
    assert(p186_sue_number_of_inputs_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p186_sue_trim_value_input_5_GET(pack) == (int16_t)(int16_t)19323);
    assert(p186_sue_trim_value_input_6_GET(pack) == (int16_t)(int16_t) -30991);
    assert(p186_sue_trim_value_input_4_GET(pack) == (int16_t)(int16_t) -9655);
    assert(p186_sue_trim_value_input_12_GET(pack) == (int16_t)(int16_t) -11015);
    assert(p186_sue_trim_value_input_7_GET(pack) == (int16_t)(int16_t) -17933);
    assert(p186_sue_trim_value_input_1_GET(pack) == (int16_t)(int16_t)14448);
    assert(p186_sue_trim_value_input_8_GET(pack) == (int16_t)(int16_t) -14659);
    assert(p186_sue_trim_value_input_3_GET(pack) == (int16_t)(int16_t)13140);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    assert(p187_sue_gyro_y_offset_GET(pack) == (int16_t)(int16_t)29160);
    assert(p187_sue_accel_x_offset_GET(pack) == (int16_t)(int16_t)7693);
    assert(p187_sue_gyro_z_offset_GET(pack) == (int16_t)(int16_t) -12587);
    assert(p187_sue_gyro_x_offset_GET(pack) == (int16_t)(int16_t) -4058);
    assert(p187_sue_accel_z_offset_GET(pack) == (int16_t)(int16_t) -25422);
    assert(p187_sue_accel_y_offset_GET(pack) == (int16_t)(int16_t) -796);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_sue_accel_x_at_calibration_GET(pack) == (int16_t)(int16_t) -9211);
    assert(p188_sue_gyro_z_at_calibration_GET(pack) == (int16_t)(int16_t)5698);
    assert(p188_sue_gyro_y_at_calibration_GET(pack) == (int16_t)(int16_t) -20026);
    assert(p188_sue_accel_z_at_calibration_GET(pack) == (int16_t)(int16_t) -15398);
    assert(p188_sue_accel_y_at_calibration_GET(pack) == (int16_t)(int16_t) -14418);
    assert(p188_sue_gyro_x_at_calibration_GET(pack) == (int16_t)(int16_t)2202);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_time_usec_GET(pack) == (uint64_t)6248845200717246395L);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.8286976E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL));
    assert(p230_hagl_ratio_GET(pack) == (float) -6.653195E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.6870002E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)3.3661884E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -2.9117432E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -7.037302E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.9698682E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.6199287E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float) -2.4341015E38F);
    assert(p231_wind_x_GET(pack) == (float) -2.6074413E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)9147695598010197187L);
    assert(p231_horiz_accuracy_GET(pack) == (float)2.2053025E38F);
    assert(p231_var_horiz_GET(pack) == (float)3.2857788E38F);
    assert(p231_wind_z_GET(pack) == (float)1.752714E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -5.8000973E37F);
    assert(p231_wind_y_GET(pack) == (float)1.3419783E38F);
    assert(p231_var_vert_GET(pack) == (float) -2.391233E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_vd_GET(pack) == (float) -1.7292868E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p232_vert_accuracy_GET(pack) == (float)8.426687E37F);
    assert(p232_alt_GET(pack) == (float)4.556866E37F);
    assert(p232_ve_GET(pack) == (float) -1.1583307E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)1412210928L);
    assert(p232_vdop_GET(pack) == (float)3.0729035E37F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)29808);
    assert(p232_vn_GET(pack) == (float) -2.8135014E38F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
    assert(p232_lat_GET(pack) == (int32_t)1486587317);
    assert(p232_time_usec_GET(pack) == (uint64_t)8737193965070143742L);
    assert(p232_lon_GET(pack) == (int32_t)1752001633);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p232_horiz_accuracy_GET(pack) == (float)1.1791403E37F);
    assert(p232_speed_accuracy_GET(pack) == (float) -1.5101175E38F);
    assert(p232_hdop_GET(pack) == (float) -1.958829E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)10);
    {
        uint8_t exemplary[] =  {(uint8_t)183, (uint8_t)192, (uint8_t)161, (uint8_t)81, (uint8_t)91, (uint8_t)51, (uint8_t)169, (uint8_t)94, (uint8_t)31, (uint8_t)82, (uint8_t)200, (uint8_t)112, (uint8_t)250, (uint8_t)104, (uint8_t)142, (uint8_t)231, (uint8_t)202, (uint8_t)22, (uint8_t)69, (uint8_t)222, (uint8_t)129, (uint8_t)190, (uint8_t)140, (uint8_t)180, (uint8_t)204, (uint8_t)101, (uint8_t)149, (uint8_t)219, (uint8_t)179, (uint8_t)56, (uint8_t)209, (uint8_t)241, (uint8_t)109, (uint8_t)207, (uint8_t)34, (uint8_t)2, (uint8_t)183, (uint8_t)97, (uint8_t)110, (uint8_t)150, (uint8_t)25, (uint8_t)232, (uint8_t)151, (uint8_t)204, (uint8_t)233, (uint8_t)242, (uint8_t)17, (uint8_t)215, (uint8_t)236, (uint8_t)38, (uint8_t)100, (uint8_t)103, (uint8_t)224, (uint8_t)178, (uint8_t)50, (uint8_t)84, (uint8_t)166, (uint8_t)116, (uint8_t)234, (uint8_t)54, (uint8_t)157, (uint8_t)198, (uint8_t)237, (uint8_t)53, (uint8_t)188, (uint8_t)223, (uint8_t)251, (uint8_t)152, (uint8_t)143, (uint8_t)211, (uint8_t)124, (uint8_t)22, (uint8_t)7, (uint8_t)251, (uint8_t)223, (uint8_t)215, (uint8_t)86, (uint8_t)114, (uint8_t)66, (uint8_t)164, (uint8_t)150, (uint8_t)221, (uint8_t)253, (uint8_t)124, (uint8_t)35, (uint8_t)143, (uint8_t)192, (uint8_t)61, (uint8_t)179, (uint8_t)149, (uint8_t)63, (uint8_t)185, (uint8_t)148, (uint8_t)249, (uint8_t)16, (uint8_t)174, (uint8_t)33, (uint8_t)69, (uint8_t)168, (uint8_t)77, (uint8_t)252, (uint8_t)185, (uint8_t)228, (uint8_t)139, (uint8_t)186, (uint8_t)7, (uint8_t)25, (uint8_t)187, (uint8_t)152, (uint8_t)214, (uint8_t)195, (uint8_t)180, (uint8_t)232, (uint8_t)29, (uint8_t)89, (uint8_t)10, (uint8_t)113, (uint8_t)166, (uint8_t)241, (uint8_t)204, (uint8_t)199, (uint8_t)239, (uint8_t)99, (uint8_t)62, (uint8_t)148, (uint8_t)27, (uint8_t)2, (uint8_t)187, (uint8_t)206, (uint8_t)51, (uint8_t)105, (uint8_t)199, (uint8_t)236, (uint8_t)30, (uint8_t)0, (uint8_t)105, (uint8_t)161, (uint8_t)13, (uint8_t)251, (uint8_t)119, (uint8_t)109, (uint8_t)254, (uint8_t)103, (uint8_t)145, (uint8_t)45, (uint8_t)83, (uint8_t)154, (uint8_t)101, (uint8_t)211, (uint8_t)193, (uint8_t)245, (uint8_t)66, (uint8_t)119, (uint8_t)135, (uint8_t)73, (uint8_t)98, (uint8_t)5, (uint8_t)49, (uint8_t)119, (uint8_t)236, (uint8_t)74, (uint8_t)43, (uint8_t)140, (uint8_t)33, (uint8_t)91, (uint8_t)32, (uint8_t)87, (uint8_t)134, (uint8_t)3, (uint8_t)178, (uint8_t)193, (uint8_t)98, (uint8_t)221, (uint8_t)150, (uint8_t)220, (uint8_t)72, (uint8_t)135, (uint8_t)29, (uint8_t)238, (uint8_t)42} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)22);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)2509);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)55);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)10749);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)32601);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1013337221L);
    assert(p234_latitude_GET(pack) == (int32_t)2065493425);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)44682);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -32515);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)20913);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)7);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -16040);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)70);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p234_longitude_GET(pack) == (int32_t)18936226);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_0_GET(pack) == (uint32_t)2325159602L);
    assert(p241_vibration_y_GET(pack) == (float) -2.9226975E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)6870660482108680559L);
    assert(p241_vibration_x_GET(pack) == (float) -1.8669227E37F);
    assert(p241_vibration_z_GET(pack) == (float)2.585756E37F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)1437993313L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)684067955L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_altitude_GET(pack) == (int32_t) -647985461);
    assert(p242_x_GET(pack) == (float) -6.2000867E37F);
    assert(p242_z_GET(pack) == (float) -1.8634437E38F);
    assert(p242_longitude_GET(pack) == (int32_t)2074321680);
    assert(p242_approach_x_GET(pack) == (float)1.8458406E38F);
    assert(p242_approach_z_GET(pack) == (float) -3.2795052E38F);
    assert(p242_y_GET(pack) == (float)1.0823807E38F);
    assert(p242_approach_y_GET(pack) == (float)1.7058685E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)7393614016295478337L);
    {
        float exemplary[] =  {-3.0066474E38F, 3.256559E38F, -1.0524146E38F, -1.1424947E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_latitude_GET(pack) == (int32_t)1951866910);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_latitude_GET(pack) == (int32_t)1449617055);
    {
        float exemplary[] =  {1.030865E38F, -8.626237E37F, 1.3287596E38F, 1.0957855E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_y_GET(pack) == (float)7.5168877E37F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p243_approach_x_GET(pack) == (float) -5.7618923E37F);
    assert(p243_longitude_GET(pack) == (int32_t) -717784874);
    assert(p243_x_GET(pack) == (float) -6.7831436E37F);
    assert(p243_approach_y_GET(pack) == (float)1.2787696E38F);
    assert(p243_altitude_GET(pack) == (int32_t)354314507);
    assert(p243_time_usec_TRY(ph) == (uint64_t)7386741990352710258L);
    assert(p243_approach_z_GET(pack) == (float)4.861447E37F);
    assert(p243_z_GET(pack) == (float)1.3558568E37F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -363911996);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)61708);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)65151);
    assert(p246_callsign_LEN(ph) == 7);
    {
        char16_t * exemplary = u"nMlFfzh";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p246_lon_GET(pack) == (int32_t)932303807);
    assert(p246_altitude_GET(pack) == (int32_t) -36135835);
    assert(p246_lat_GET(pack) == (int32_t) -1141306723);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3642377243L);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING));
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)32437);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)12247);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -15786);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -2.5402337E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.0233439E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_id_GET(pack) == (uint32_t)190520276L);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)2.9160507E38F);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)1772);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)0);
    {
        uint8_t exemplary[] =  {(uint8_t)78, (uint8_t)193, (uint8_t)77, (uint8_t)111, (uint8_t)190, (uint8_t)46, (uint8_t)164, (uint8_t)254, (uint8_t)129, (uint8_t)69, (uint8_t)157, (uint8_t)172, (uint8_t)51, (uint8_t)176, (uint8_t)39, (uint8_t)219, (uint8_t)57, (uint8_t)6, (uint8_t)215, (uint8_t)141, (uint8_t)210, (uint8_t)158, (uint8_t)161, (uint8_t)83, (uint8_t)210, (uint8_t)49, (uint8_t)126, (uint8_t)93, (uint8_t)194, (uint8_t)66, (uint8_t)163, (uint8_t)156, (uint8_t)19, (uint8_t)18, (uint8_t)41, (uint8_t)54, (uint8_t)202, (uint8_t)103, (uint8_t)218, (uint8_t)167, (uint8_t)242, (uint8_t)4, (uint8_t)44, (uint8_t)161, (uint8_t)166, (uint8_t)93, (uint8_t)247, (uint8_t)254, (uint8_t)188, (uint8_t)224, (uint8_t)61, (uint8_t)149, (uint8_t)79, (uint8_t)70, (uint8_t)16, (uint8_t)130, (uint8_t)117, (uint8_t)39, (uint8_t)177, (uint8_t)18, (uint8_t)194, (uint8_t)104, (uint8_t)211, (uint8_t)127, (uint8_t)122, (uint8_t)83, (uint8_t)145, (uint8_t)218, (uint8_t)238, (uint8_t)11, (uint8_t)220, (uint8_t)24, (uint8_t)232, (uint8_t)150, (uint8_t)24, (uint8_t)108, (uint8_t)29, (uint8_t)100, (uint8_t)234, (uint8_t)85, (uint8_t)230, (uint8_t)181, (uint8_t)112, (uint8_t)2, (uint8_t)201, (uint8_t)28, (uint8_t)31, (uint8_t)70, (uint8_t)169, (uint8_t)186, (uint8_t)197, (uint8_t)233, (uint8_t)24, (uint8_t)251, (uint8_t)147, (uint8_t)23, (uint8_t)218, (uint8_t)200, (uint8_t)118, (uint8_t)246, (uint8_t)66, (uint8_t)27, (uint8_t)78, (uint8_t)44, (uint8_t)200, (uint8_t)136, (uint8_t)80, (uint8_t)164, (uint8_t)215, (uint8_t)180, (uint8_t)237, (uint8_t)224, (uint8_t)39, (uint8_t)23, (uint8_t)56, (uint8_t)220, (uint8_t)134, (uint8_t)252, (uint8_t)57, (uint8_t)55, (uint8_t)55, (uint8_t)189, (uint8_t)22, (uint8_t)55, (uint8_t)90, (uint8_t)108, (uint8_t)148, (uint8_t)232, (uint8_t)244, (uint8_t)199, (uint8_t)172, (uint8_t)102, (uint8_t)23, (uint8_t)20, (uint8_t)233, (uint8_t)213, (uint8_t)120, (uint8_t)123, (uint8_t)188, (uint8_t)70, (uint8_t)181, (uint8_t)87, (uint8_t)186, (uint8_t)104, (uint8_t)57, (uint8_t)20, (uint8_t)39, (uint8_t)129, (uint8_t)87, (uint8_t)96, (uint8_t)158, (uint8_t)168, (uint8_t)82, (uint8_t)233, (uint8_t)176, (uint8_t)21, (uint8_t)73, (uint8_t)14, (uint8_t)212, (uint8_t)63, (uint8_t)148, (uint8_t)59, (uint8_t)110, (uint8_t)209, (uint8_t)239, (uint8_t)150, (uint8_t)241, (uint8_t)21, (uint8_t)3, (uint8_t)29, (uint8_t)150, (uint8_t)218, (uint8_t)198, (uint8_t)105, (uint8_t)163, (uint8_t)51, (uint8_t)220, (uint8_t)59, (uint8_t)76, (uint8_t)107, (uint8_t)193, (uint8_t)185, (uint8_t)149, (uint8_t)25, (uint8_t)196, (uint8_t)235, (uint8_t)88, (uint8_t)158, (uint8_t)190, (uint8_t)217, (uint8_t)68, (uint8_t)92, (uint8_t)179, (uint8_t)65, (uint8_t)116, (uint8_t)166, (uint8_t)252, (uint8_t)79, (uint8_t)236, (uint8_t)213, (uint8_t)0, (uint8_t)58, (uint8_t)245, (uint8_t)21, (uint8_t)78, (uint8_t)189, (uint8_t)98, (uint8_t)36, (uint8_t)241, (uint8_t)248, (uint8_t)77, (uint8_t)70, (uint8_t)121, (uint8_t)27, (uint8_t)210, (uint8_t)77, (uint8_t)158, (uint8_t)63, (uint8_t)15, (uint8_t)174, (uint8_t)86, (uint8_t)68, (uint8_t)34, (uint8_t)112, (uint8_t)164, (uint8_t)80, (uint8_t)11, (uint8_t)194, (uint8_t)44, (uint8_t)92, (uint8_t)228, (uint8_t)96, (uint8_t)107, (uint8_t)137, (uint8_t)152, (uint8_t)23, (uint8_t)80, (uint8_t)221, (uint8_t)137, (uint8_t)36, (uint8_t)137, (uint8_t)240, (uint8_t)8, (uint8_t)104, (uint8_t)16, (uint8_t)226, (uint8_t)58, (uint8_t)94, (uint8_t)118} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)72);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)64);
    {
        int8_t exemplary[] =  {(int8_t) -51, (int8_t) -76, (int8_t)7, (int8_t)116, (int8_t) -69, (int8_t) -114, (int8_t)125, (int8_t) -12, (int8_t) -100, (int8_t)10, (int8_t)20, (int8_t) -48, (int8_t) -45, (int8_t) -89, (int8_t) -116, (int8_t) -59, (int8_t) -30, (int8_t) -15, (int8_t) -126, (int8_t)114, (int8_t)107, (int8_t)42, (int8_t)76, (int8_t) -82, (int8_t) -104, (int8_t)113, (int8_t) -101, (int8_t)3, (int8_t) -78, (int8_t) -106, (int8_t)127, (int8_t) -32} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)14809);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float)1.1586995E38F);
    assert(p250_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"gxzkihczv";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_y_GET(pack) == (float) -3.0378654E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)3352360465841404862L);
    assert(p250_z_GET(pack) == (float) -1.0313853E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3143771925L);
    assert(p251_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"mauuluhlla";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float) -1.4263979E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3431075127L);
    assert(p252_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"wWxzt";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_value_GET(pack) == (int32_t)67229384);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 26);
    {
        char16_t * exemplary = u"cuwcozSesgAkkzXpfkbvdtbwbv";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 52);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)8.188972E37F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)2944420054L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)158);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)6254431061429079679L);
    {
        uint8_t exemplary[] =  {(uint8_t)248, (uint8_t)209, (uint8_t)34, (uint8_t)179, (uint8_t)140, (uint8_t)51, (uint8_t)76, (uint8_t)35, (uint8_t)3, (uint8_t)226, (uint8_t)42, (uint8_t)34, (uint8_t)29, (uint8_t)118, (uint8_t)47, (uint8_t)225, (uint8_t)202, (uint8_t)35, (uint8_t)216, (uint8_t)227, (uint8_t)127, (uint8_t)86, (uint8_t)230, (uint8_t)158, (uint8_t)153, (uint8_t)23, (uint8_t)23, (uint8_t)58, (uint8_t)236, (uint8_t)27, (uint8_t)122, (uint8_t)11} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)3);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3149641927L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)4171507764L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 28);
    {
        char16_t * exemplary = u"yhotauigcybpcdwhwuehjtPylgnu";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)46709);
    assert(p259_firmware_version_GET(pack) == (uint32_t)2551108048L);
    assert(p259_focal_length_GET(pack) == (float) -2.4084051E38F);
    assert(p259_cam_definition_uri_LEN(ph) == 22);
    {
        char16_t * exemplary = u"EhqFmMzEjwspzeJqzdmcEt";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)162, (uint8_t)17, (uint8_t)105, (uint8_t)182, (uint8_t)197, (uint8_t)126, (uint8_t)124, (uint8_t)127, (uint8_t)91, (uint8_t)41, (uint8_t)197, (uint8_t)237, (uint8_t)185, (uint8_t)137, (uint8_t)51, (uint8_t)157, (uint8_t)79, (uint8_t)93, (uint8_t)154, (uint8_t)115, (uint8_t)47, (uint8_t)49, (uint8_t)62, (uint8_t)121, (uint8_t)195, (uint8_t)134, (uint8_t)63, (uint8_t)25, (uint8_t)250, (uint8_t)116, (uint8_t)99, (uint8_t)24} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)33351);
    assert(p259_sensor_size_v_GET(pack) == (float) -3.0767428E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)22, (uint8_t)98, (uint8_t)126, (uint8_t)148, (uint8_t)187, (uint8_t)168, (uint8_t)92, (uint8_t)143, (uint8_t)37, (uint8_t)255, (uint8_t)205, (uint8_t)33, (uint8_t)35, (uint8_t)107, (uint8_t)39, (uint8_t)165, (uint8_t)140, (uint8_t)137, (uint8_t)173, (uint8_t)154, (uint8_t)160, (uint8_t)42, (uint8_t)11, (uint8_t)120, (uint8_t)68, (uint8_t)124, (uint8_t)224, (uint8_t)193, (uint8_t)84, (uint8_t)222, (uint8_t)232, (uint8_t)140} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)26448);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3159776246L);
    assert(p259_sensor_size_h_GET(pack) == (float) -3.6758394E37F);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1937968843L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p261_available_capacity_GET(pack) == (float) -1.5798219E37F);
    assert(p261_total_capacity_GET(pack) == (float) -2.708585E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p261_read_speed_GET(pack) == (float) -2.2926374E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1788017670L);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p261_used_capacity_GET(pack) == (float)1.6512937E38F);
    assert(p261_write_speed_GET(pack) == (float) -2.7819704E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float) -1.9878287E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)1011099220L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p262_available_capacity_GET(pack) == (float)9.13089E37F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)429483433L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_image_index_GET(pack) == (int32_t)719444745);
    assert(p263_lon_GET(pack) == (int32_t)1328366276);
    assert(p263_file_url_LEN(ph) == 3);
    {
        char16_t * exemplary = u"hjy";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_utc_GET(pack) == (uint64_t)4874047863655874151L);
    assert(p263_alt_GET(pack) == (int32_t) -1712015757);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -3);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)86);
    {
        float exemplary[] =  {2.853934E38F, 3.5989876E37F, 2.42668E38F, -1.3845336E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_lat_GET(pack) == (int32_t) -1649261467);
    assert(p263_relative_alt_GET(pack) == (int32_t)603713508);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2904395782L);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)1400505981L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)1155851505789340459L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)9055510902869687740L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)5706306694792320464L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1579683753L);
    assert(p265_yaw_GET(pack) == (float) -2.052244E38F);
    assert(p265_pitch_GET(pack) == (float)2.124808E38F);
    assert(p265_roll_GET(pack) == (float) -2.1851869E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)42432);
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)182, (uint8_t)237, (uint8_t)190, (uint8_t)6, (uint8_t)203, (uint8_t)65, (uint8_t)226, (uint8_t)152, (uint8_t)70, (uint8_t)130, (uint8_t)144, (uint8_t)116, (uint8_t)228, (uint8_t)47, (uint8_t)62, (uint8_t)45, (uint8_t)234, (uint8_t)171, (uint8_t)125, (uint8_t)77, (uint8_t)144, (uint8_t)36, (uint8_t)130, (uint8_t)76, (uint8_t)159, (uint8_t)158, (uint8_t)97, (uint8_t)152, (uint8_t)4, (uint8_t)232, (uint8_t)216, (uint8_t)34, (uint8_t)153, (uint8_t)59, (uint8_t)18, (uint8_t)37, (uint8_t)240, (uint8_t)95, (uint8_t)162, (uint8_t)39, (uint8_t)116, (uint8_t)35, (uint8_t)129, (uint8_t)72, (uint8_t)15, (uint8_t)145, (uint8_t)96, (uint8_t)184, (uint8_t)77, (uint8_t)113, (uint8_t)37, (uint8_t)75, (uint8_t)114, (uint8_t)36, (uint8_t)157, (uint8_t)107, (uint8_t)179, (uint8_t)175, (uint8_t)5, (uint8_t)177, (uint8_t)170, (uint8_t)80, (uint8_t)10, (uint8_t)220, (uint8_t)54, (uint8_t)36, (uint8_t)32, (uint8_t)28, (uint8_t)218, (uint8_t)220, (uint8_t)91, (uint8_t)23, (uint8_t)19, (uint8_t)169, (uint8_t)192, (uint8_t)3, (uint8_t)216, (uint8_t)61, (uint8_t)98, (uint8_t)37, (uint8_t)65, (uint8_t)57, (uint8_t)201, (uint8_t)225, (uint8_t)76, (uint8_t)80, (uint8_t)174, (uint8_t)93, (uint8_t)162, (uint8_t)115, (uint8_t)80, (uint8_t)7, (uint8_t)117, (uint8_t)245, (uint8_t)214, (uint8_t)175, (uint8_t)197, (uint8_t)199, (uint8_t)196, (uint8_t)192, (uint8_t)230, (uint8_t)209, (uint8_t)203, (uint8_t)213, (uint8_t)216, (uint8_t)247, (uint8_t)140, (uint8_t)49, (uint8_t)84, (uint8_t)25, (uint8_t)45, (uint8_t)223, (uint8_t)30, (uint8_t)180, (uint8_t)181, (uint8_t)65, (uint8_t)41, (uint8_t)43, (uint8_t)231, (uint8_t)43, (uint8_t)35, (uint8_t)219, (uint8_t)120, (uint8_t)38, (uint8_t)118, (uint8_t)91, (uint8_t)250, (uint8_t)17, (uint8_t)103, (uint8_t)215, (uint8_t)209, (uint8_t)54, (uint8_t)150, (uint8_t)188, (uint8_t)115, (uint8_t)160, (uint8_t)68, (uint8_t)197, (uint8_t)6, (uint8_t)165, (uint8_t)185, (uint8_t)57, (uint8_t)29, (uint8_t)172, (uint8_t)226, (uint8_t)21, (uint8_t)27, (uint8_t)124, (uint8_t)249, (uint8_t)229, (uint8_t)221, (uint8_t)1, (uint8_t)212, (uint8_t)20, (uint8_t)47, (uint8_t)32, (uint8_t)101, (uint8_t)154, (uint8_t)28, (uint8_t)252, (uint8_t)148, (uint8_t)210, (uint8_t)172, (uint8_t)116, (uint8_t)146, (uint8_t)254, (uint8_t)208, (uint8_t)209, (uint8_t)198, (uint8_t)2, (uint8_t)185, (uint8_t)32, (uint8_t)85, (uint8_t)84, (uint8_t)156, (uint8_t)239, (uint8_t)122, (uint8_t)43, (uint8_t)141, (uint8_t)119, (uint8_t)39, (uint8_t)147, (uint8_t)203, (uint8_t)167, (uint8_t)129, (uint8_t)66, (uint8_t)85, (uint8_t)123, (uint8_t)58, (uint8_t)57, (uint8_t)217, (uint8_t)142, (uint8_t)106, (uint8_t)37, (uint8_t)217, (uint8_t)116, (uint8_t)48, (uint8_t)248, (uint8_t)45, (uint8_t)234, (uint8_t)133, (uint8_t)241, (uint8_t)8, (uint8_t)194, (uint8_t)152, (uint8_t)34, (uint8_t)71, (uint8_t)51, (uint8_t)133, (uint8_t)49, (uint8_t)246, (uint8_t)14, (uint8_t)134, (uint8_t)2, (uint8_t)71, (uint8_t)158, (uint8_t)211, (uint8_t)165, (uint8_t)176, (uint8_t)250, (uint8_t)118, (uint8_t)159, (uint8_t)20, (uint8_t)162, (uint8_t)140, (uint8_t)100, (uint8_t)224, (uint8_t)204, (uint8_t)108, (uint8_t)248, (uint8_t)128, (uint8_t)228, (uint8_t)70, (uint8_t)114, (uint8_t)248, (uint8_t)66, (uint8_t)25, (uint8_t)163, (uint8_t)137, (uint8_t)35, (uint8_t)37, (uint8_t)222, (uint8_t)72, (uint8_t)224, (uint8_t)82, (uint8_t)53, (uint8_t)195, (uint8_t)194} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)255, (uint8_t)221, (uint8_t)14, (uint8_t)152, (uint8_t)210, (uint8_t)167, (uint8_t)218, (uint8_t)103, (uint8_t)218, (uint8_t)245, (uint8_t)27, (uint8_t)183, (uint8_t)84, (uint8_t)245, (uint8_t)136, (uint8_t)40, (uint8_t)66, (uint8_t)241, (uint8_t)98, (uint8_t)234, (uint8_t)206, (uint8_t)13, (uint8_t)102, (uint8_t)56, (uint8_t)85, (uint8_t)239, (uint8_t)148, (uint8_t)166, (uint8_t)194, (uint8_t)170, (uint8_t)168, (uint8_t)222, (uint8_t)163, (uint8_t)107, (uint8_t)213, (uint8_t)136, (uint8_t)232, (uint8_t)87, (uint8_t)113, (uint8_t)118, (uint8_t)193, (uint8_t)90, (uint8_t)21, (uint8_t)54, (uint8_t)75, (uint8_t)126, (uint8_t)145, (uint8_t)215, (uint8_t)247, (uint8_t)28, (uint8_t)240, (uint8_t)24, (uint8_t)12, (uint8_t)28, (uint8_t)21, (uint8_t)60, (uint8_t)231, (uint8_t)46, (uint8_t)161, (uint8_t)198, (uint8_t)110, (uint8_t)220, (uint8_t)44, (uint8_t)156, (uint8_t)109, (uint8_t)139, (uint8_t)232, (uint8_t)101, (uint8_t)126, (uint8_t)153, (uint8_t)124, (uint8_t)215, (uint8_t)194, (uint8_t)84, (uint8_t)236, (uint8_t)112, (uint8_t)159, (uint8_t)70, (uint8_t)191, (uint8_t)101, (uint8_t)95, (uint8_t)77, (uint8_t)59, (uint8_t)47, (uint8_t)141, (uint8_t)2, (uint8_t)75, (uint8_t)241, (uint8_t)249, (uint8_t)228, (uint8_t)22, (uint8_t)41, (uint8_t)133, (uint8_t)41, (uint8_t)231, (uint8_t)254, (uint8_t)240, (uint8_t)197, (uint8_t)217, (uint8_t)184, (uint8_t)220, (uint8_t)239, (uint8_t)201, (uint8_t)217, (uint8_t)87, (uint8_t)36, (uint8_t)190, (uint8_t)138, (uint8_t)99, (uint8_t)226, (uint8_t)83, (uint8_t)106, (uint8_t)251, (uint8_t)170, (uint8_t)41, (uint8_t)203, (uint8_t)188, (uint8_t)0, (uint8_t)216, (uint8_t)69, (uint8_t)34, (uint8_t)200, (uint8_t)246, (uint8_t)199, (uint8_t)0, (uint8_t)59, (uint8_t)26, (uint8_t)136, (uint8_t)190, (uint8_t)137, (uint8_t)82, (uint8_t)242, (uint8_t)2, (uint8_t)33, (uint8_t)52, (uint8_t)184, (uint8_t)254, (uint8_t)60, (uint8_t)178, (uint8_t)200, (uint8_t)108, (uint8_t)173, (uint8_t)46, (uint8_t)153, (uint8_t)24, (uint8_t)16, (uint8_t)208, (uint8_t)124, (uint8_t)104, (uint8_t)134, (uint8_t)122, (uint8_t)14, (uint8_t)233, (uint8_t)180, (uint8_t)34, (uint8_t)7, (uint8_t)100, (uint8_t)20, (uint8_t)111, (uint8_t)61, (uint8_t)86, (uint8_t)69, (uint8_t)147, (uint8_t)117, (uint8_t)224, (uint8_t)255, (uint8_t)107, (uint8_t)65, (uint8_t)123, (uint8_t)191, (uint8_t)20, (uint8_t)210, (uint8_t)77, (uint8_t)206, (uint8_t)180, (uint8_t)198, (uint8_t)171, (uint8_t)202, (uint8_t)144, (uint8_t)170, (uint8_t)41, (uint8_t)158, (uint8_t)147, (uint8_t)111, (uint8_t)56, (uint8_t)60, (uint8_t)6, (uint8_t)171, (uint8_t)143, (uint8_t)125, (uint8_t)23, (uint8_t)172, (uint8_t)91, (uint8_t)69, (uint8_t)220, (uint8_t)129, (uint8_t)129, (uint8_t)208, (uint8_t)209, (uint8_t)92, (uint8_t)193, (uint8_t)150, (uint8_t)146, (uint8_t)109, (uint8_t)6, (uint8_t)235, (uint8_t)194, (uint8_t)248, (uint8_t)40, (uint8_t)152, (uint8_t)224, (uint8_t)109, (uint8_t)60, (uint8_t)49, (uint8_t)255, (uint8_t)155, (uint8_t)113, (uint8_t)193, (uint8_t)201, (uint8_t)192, (uint8_t)223, (uint8_t)214, (uint8_t)198, (uint8_t)199, (uint8_t)67, (uint8_t)68, (uint8_t)169, (uint8_t)125, (uint8_t)218, (uint8_t)87, (uint8_t)150, (uint8_t)55, (uint8_t)212, (uint8_t)197, (uint8_t)5, (uint8_t)133, (uint8_t)23, (uint8_t)201, (uint8_t)26, (uint8_t)174, (uint8_t)102, (uint8_t)166, (uint8_t)171, (uint8_t)218, (uint8_t)5, (uint8_t)61, (uint8_t)246, (uint8_t)236} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)20679);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)101);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)49947);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)31);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float) -1.7971075E38F);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)65470);
    assert(p269_bitrate_GET(pack) == (uint32_t)3314246950L);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)62315);
    assert(p269_uri_LEN(ph) == 197);
    {
        char16_t * exemplary = u"lxifdbqyffjwdfoqgfqhkeribpkrdmhYjtZuvrgefspsovxxwidjtmpjeayqqwgglgqnwocdvllKkldvCjeoLUxpzcfluggyLfiGurcjcwtjfylxshacbjtxqlsFxdxwzyoVvgtfwvyynnntimjDjfldfgclgjgjyRpebifbxyjFnuxsmslkgkgxnhbksjkiraiSt";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 394);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)52681);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)8495);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)22769);
    assert(p270_bitrate_GET(pack) == (uint32_t)3853658581L);
    assert(p270_uri_LEN(ph) == 198);
    {
        char16_t * exemplary = u"xkxlzrmcggxMqOiszzjopnizmjxgghocTtpdhuGAfsmldgwgkbaxXcxAsjzNhaitjGhwqhrheSwannsnzmujdqHslevbqpecuHaipupklPzYgqlndfsoynlwavckkfgqbkWfczSyqjvmclrdrrjmtjdtXzxkozqxmpfpbxevhUpcmfpwzdebIcgmbCClqumLpnzjTY";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 396);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)19158);
    assert(p270_framerate_GET(pack) == (float) -3.223009E38F);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)117);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 19);
    {
        char16_t * exemplary = u"xnthzjwowouzlAhbcQQ";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 25);
    {
        char16_t * exemplary = u"gluDnsfZclnmrlzvygmwlfiey";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)201, (uint8_t)125, (uint8_t)49, (uint8_t)164, (uint8_t)192, (uint8_t)177, (uint8_t)161} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)35946);
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)113, (uint8_t)69, (uint8_t)226, (uint8_t)48, (uint8_t)29, (uint8_t)229, (uint8_t)176} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)4406);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)45523);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1127211702L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)41776);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_time_usec_GET(pack) == (uint64_t)5477902761537677911L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p311_time_usec_GET(pack) == (uint64_t)267766422710197405L);
    assert(p311_name_LEN(ph) == 61);
    {
        char16_t * exemplary = u"dgqypidhizxvyfuQPHOufbqLcYBekanfgadtbzKsmehjrffjdjirjarhkwcor";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 122);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2758417286L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)216760096L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)251);
    {
        uint8_t exemplary[] =  {(uint8_t)53, (uint8_t)222, (uint8_t)49, (uint8_t)154, (uint8_t)9, (uint8_t)136, (uint8_t)68, (uint8_t)50, (uint8_t)13, (uint8_t)26, (uint8_t)0, (uint8_t)233, (uint8_t)233, (uint8_t)23, (uint8_t)110, (uint8_t)145} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p320_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"guspqdjaynjOIBJ";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)12149);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)6);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)11556);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)17263);
    assert(p322_param_value_LEN(ph) == 12);
    {
        char16_t * exemplary = u"zavxjjyejeNe";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"bu";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p323_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"pzlpjc";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p323_param_value_LEN(ph) == 54);
    {
        char16_t * exemplary = u"ixjjhebxederddsdlpldflssgcmtupitikugJyxplbnejdpoMgpGHl";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 108);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)154);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"Owdngewsuahtxqxx";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 70);
    {
        char16_t * exemplary = u"xoesfaKwrqsrkziidrdsqgTmwmddcxfFmrOUouvdyezkcsLvczqfyqxphfhxfseZjksmyG";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 140);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    {
        uint16_t exemplary[] =  {(uint16_t)46799, (uint16_t)48988, (uint16_t)57200, (uint16_t)9517, (uint16_t)43415, (uint16_t)58942, (uint16_t)58091, (uint16_t)12285, (uint16_t)10847, (uint16_t)58247, (uint16_t)34047, (uint16_t)50637, (uint16_t)42595, (uint16_t)15006, (uint16_t)10625, (uint16_t)12483, (uint16_t)35611, (uint16_t)40532, (uint16_t)53142, (uint16_t)64896, (uint16_t)59866, (uint16_t)5036, (uint16_t)18916, (uint16_t)39510, (uint16_t)25125, (uint16_t)46870, (uint16_t)25779, (uint16_t)44303, (uint16_t)57591, (uint16_t)34736, (uint16_t)9684, (uint16_t)33964, (uint16_t)8594, (uint16_t)51822, (uint16_t)6371, (uint16_t)16522, (uint16_t)59753, (uint16_t)44564, (uint16_t)27684, (uint16_t)51687, (uint16_t)2440, (uint16_t)15797, (uint16_t)43600, (uint16_t)31748, (uint16_t)47251, (uint16_t)8985, (uint16_t)35960, (uint16_t)20472, (uint16_t)9980, (uint16_t)21554, (uint16_t)44324, (uint16_t)13878, (uint16_t)61611, (uint16_t)53054, (uint16_t)39596, (uint16_t)10170, (uint16_t)10871, (uint16_t)53111, (uint16_t)22135, (uint16_t)35605, (uint16_t)40774, (uint16_t)52109, (uint16_t)21740, (uint16_t)53319, (uint16_t)8913, (uint16_t)64909, (uint16_t)21890, (uint16_t)51741, (uint16_t)1752, (uint16_t)37041, (uint16_t)53690, (uint16_t)54269} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)8920);
    assert(p330_time_usec_GET(pack) == (uint64_t)8484876117719229488L);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)49068);
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
        p0_custom_mode_SET((uint32_t)2350547745L, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_TRICOPTER, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING), PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)45873, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)62518, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)31156, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY), PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)34208, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)58769, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)39250, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)98, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -23869, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)40546, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)43885, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)1656917868L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)2359963211507791355L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_SET((float)1.3479233E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float)6.682461E37F, PH.base.pack) ;
        p3_y_SET((float) -3.2231994E38F, PH.base.pack) ;
        p3_z_SET((float)1.8426516E38F, PH.base.pack) ;
        p3_vz_SET((float) -1.3236602E38F, PH.base.pack) ;
        p3_vy_SET((float)3.3420322E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)552633439L, PH.base.pack) ;
        p3_afz_SET((float)2.9500363E38F, PH.base.pack) ;
        p3_x_SET((float)1.9363945E38F, PH.base.pack) ;
        p3_vx_SET((float) -1.1590105E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)10971, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p3_afx_SET((float)2.686841E38F, PH.base.pack) ;
        p3_afy_SET((float)1.7358694E38F, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p4_seq_SET((uint32_t)3529787616L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)7875498662349690672L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        {
            char16_t* passkey = u"lbogabvBlzsoemfppbnsqvpec";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"lfNkcUcij";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)3818623829L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -14503, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        {
            char16_t* param_id = u"zlcrnbgiV";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)23384, PH.base.pack) ;
        {
            char16_t* param_id = u"T";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_index_SET((uint16_t)(uint16_t)28161, PH.base.pack) ;
        p22_param_value_SET((float)6.4976175E37F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        {
            char16_t* param_id = u"bkebuFda";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p23_param_value_SET((float)1.455713E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_h_acc_SET((uint32_t)1228914901L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_alt_SET((int32_t)1659203894, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)35914, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)2534368895L, &PH) ;
        p24_vel_acc_SET((uint32_t)3481926648L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)42294, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p24_lat_SET((int32_t)1814399168, PH.base.pack) ;
        p24_lon_SET((int32_t) -546563754, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)3799599399L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -689226831, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)24083, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)17212, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)6101908425062521068L, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)90, (uint8_t)204, (uint8_t)114, (uint8_t)94, (uint8_t)125, (uint8_t)95, (uint8_t)61, (uint8_t)91, (uint8_t)200, (uint8_t)202, (uint8_t)101, (uint8_t)50, (uint8_t)86, (uint8_t)182, (uint8_t)183, (uint8_t)143, (uint8_t)158, (uint8_t)133, (uint8_t)17, (uint8_t)184};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)114, (uint8_t)152, (uint8_t)145, (uint8_t)125, (uint8_t)128, (uint8_t)113, (uint8_t)22, (uint8_t)40, (uint8_t)143, (uint8_t)233, (uint8_t)252, (uint8_t)53, (uint8_t)255, (uint8_t)40, (uint8_t)22, (uint8_t)123, (uint8_t)23, (uint8_t)44, (uint8_t)2, (uint8_t)4};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)57, (uint8_t)198, (uint8_t)111, (uint8_t)178, (uint8_t)210, (uint8_t)103, (uint8_t)65, (uint8_t)178, (uint8_t)34, (uint8_t)44, (uint8_t)54, (uint8_t)242, (uint8_t)148, (uint8_t)144, (uint8_t)201, (uint8_t)0, (uint8_t)17, (uint8_t)171, (uint8_t)43, (uint8_t)36};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)244, (uint8_t)102, (uint8_t)221, (uint8_t)186, (uint8_t)223, (uint8_t)144, (uint8_t)98, (uint8_t)111, (uint8_t)38, (uint8_t)229, (uint8_t)132, (uint8_t)210, (uint8_t)20, (uint8_t)63, (uint8_t)110, (uint8_t)86, (uint8_t)214, (uint8_t)40, (uint8_t)114, (uint8_t)210};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)246, (uint8_t)225, (uint8_t)213, (uint8_t)59, (uint8_t)243, (uint8_t)92, (uint8_t)179, (uint8_t)205, (uint8_t)115, (uint8_t)31, (uint8_t)74, (uint8_t)27, (uint8_t)58, (uint8_t)108, (uint8_t)20, (uint8_t)142, (uint8_t)145, (uint8_t)2, (uint8_t)209, (uint8_t)236};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_zgyro_SET((int16_t)(int16_t)10277, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -3198, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)21757, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)1456278183L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)6072, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)20437, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -20939, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -6837, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -15875, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)13215, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_ygyro_SET((int16_t)(int16_t) -3677, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)13313, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)2943, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)8264481687497404204L, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -23054, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -1902, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -28604, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)28061, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)12784, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -24636, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t)25944, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)31847, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)20794, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)11170, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)7029033341584447061L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float) -9.304874E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1134866958L, PH.base.pack) ;
        p29_press_abs_SET((float) -4.4104404E37F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)19618, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_roll_SET((float) -1.3352148E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)9.180455E37F, PH.base.pack) ;
        p30_pitch_SET((float) -1.1532459E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -4.5974325E37F, PH.base.pack) ;
        p30_yaw_SET((float)2.20606E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)1672970808L, PH.base.pack) ;
        p30_yawspeed_SET((float)3.0062093E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_rollspeed_SET((float)1.2233485E38F, PH.base.pack) ;
        p31_q2_SET((float) -3.2696157E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -1.1819234E38F, PH.base.pack) ;
        p31_q4_SET((float)4.1276995E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)4082374940L, PH.base.pack) ;
        p31_q1_SET((float) -3.3821741E38F, PH.base.pack) ;
        p31_q3_SET((float) -2.8398773E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)1.7625937E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)4120268304L, PH.base.pack) ;
        p32_vz_SET((float) -1.6738869E38F, PH.base.pack) ;
        p32_z_SET((float) -3.2863773E37F, PH.base.pack) ;
        p32_vx_SET((float)7.0624806E37F, PH.base.pack) ;
        p32_vy_SET((float) -3.7785625E37F, PH.base.pack) ;
        p32_y_SET((float)1.7605083E38F, PH.base.pack) ;
        p32_x_SET((float) -1.305126E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lon_SET((int32_t)1147802622, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)925935010L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)63165, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)24907, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -490, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -19792, PH.base.pack) ;
        p33_alt_SET((int32_t)933052202, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -13593537, PH.base.pack) ;
        p33_lat_SET((int32_t) -418012940, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan5_scaled_SET((int16_t)(int16_t)12421, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2895359963L, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -15854, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -2070, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -12902, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -1572, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)17404, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)25106, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -32555, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan4_raw_SET((uint16_t)(uint16_t)61165, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)19885, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1608544495L, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)40618, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)58033, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)15588, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)42778, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)51535, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)13893, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo10_raw_SET((uint16_t)(uint16_t)19987, &PH) ;
        p36_time_usec_SET((uint32_t)3201523141L, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)36393, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)3599, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)37650, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)39028, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)39633, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)33849, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)19082, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)60063, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)5804, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)29387, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)47955, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)60061, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)49349, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)19742, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)31306, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_end_index_SET((int16_t)(int16_t)10884, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -24906, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)13999, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -9411, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_target_component_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_x_SET((float)2.614986E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p39_param1_SET((float)1.8411E38F, PH.base.pack) ;
        p39_y_SET((float)9.085843E37F, PH.base.pack) ;
        p39_param2_SET((float)2.4287265E38F, PH.base.pack) ;
        p39_z_SET((float) -2.773029E38F, PH.base.pack) ;
        p39_param3_SET((float) -3.0351297E37F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)47523, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p39_param4_SET((float) -1.2997819E37F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)25571, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)41345, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)34309, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)17863, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)38338, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t) -2042471914, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1365198296, PH.base.pack) ;
        p48_altitude_SET((int32_t)500870691, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)7695424595866253829L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t) -96624527, PH.base.pack) ;
        p49_longitude_SET((int32_t) -1428138135, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1788813891, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)1179382182938005125L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p50_param_value_min_SET((float) -3.0401182E37F, PH.base.pack) ;
        p50_param_value0_SET((float)1.6154314E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p50_scale_SET((float) -3.2418468E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"na";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -13089, PH.base.pack) ;
        p50_param_value_max_SET((float) -8.649492E37F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)41762, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1y_SET((float)2.1807687E37F, PH.base.pack) ;
        p54_p2x_SET((float)2.8931904E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p54_p2y_SET((float)1.0095332E38F, PH.base.pack) ;
        p54_p1z_SET((float) -2.4403045E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p54_p1x_SET((float) -4.9856216E36F, PH.base.pack) ;
        p54_p2z_SET((float) -6.842973E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2z_SET((float)5.108572E37F, PH.base.pack) ;
        p55_p1x_SET((float) -7.582295E37F, PH.base.pack) ;
        p55_p1z_SET((float)5.7370986E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p55_p2y_SET((float) -1.0422561E38F, PH.base.pack) ;
        p55_p2x_SET((float)3.9700913E37F, PH.base.pack) ;
        p55_p1y_SET((float)1.292125E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_pitchspeed_SET((float)2.8510058E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.6407774E38F, -1.8310943E38F, 1.9474598E38F, -2.0609565E38F, 1.0978255E38F, -1.257742E38F, 1.9424228E38F, 2.275525E38F, 1.816117E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float) -5.469619E37F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)2144852225318049785L, PH.base.pack) ;
        {
            float q[] =  {2.682231E38F, -1.9974085E38F, -2.537388E38F, -3.297437E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)1.1451788E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_pitch_SET((float)6.3079856E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.6921246E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -13585, PH.base.pack) ;
        p62_nav_roll_SET((float) -5.451845E36F, PH.base.pack) ;
        p62_aspd_error_SET((float)1.3634263E37F, PH.base.pack) ;
        p62_alt_error_SET((float) -2.2488284E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)52476, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -5021, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        {
            float covariance[] =  {2.364486E37F, 1.5890142E38F, -2.2390208E38F, 1.2336601E38F, -2.1852899E38F, -1.7989986E38F, -2.1826428E38F, -1.8497639E38F, 1.8525257E38F, -8.2000783E37F, -1.3762437E38F, -2.1663774E37F, -6.510152E37F, -2.6536732E38F, 1.1237025E38F, 1.691337E38F, -1.4601223E38F, 3.1783221E38F, -1.354376E38F, -3.2550322E38F, -2.6504398E38F, 2.5306953E37F, 2.732532E38F, -3.0081E38F, -3.4287315E37F, 2.7891875E38F, -1.5597256E38F, 2.3827627E38F, -3.1440893E38F, 3.0836501E38F, 1.6348122E38F, -2.5673208E37F, -2.0447715E38F, 4.7001183E37F, 6.0002755E37F, -9.789147E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_alt_SET((int32_t)925805048, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)315462959, PH.base.pack) ;
        p63_vy_SET((float) -1.3460124E38F, PH.base.pack) ;
        p63_vz_SET((float)8.4272494E37F, PH.base.pack) ;
        p63_vx_SET((float) -3.2168135E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)2150240240045842683L, PH.base.pack) ;
        p63_lat_SET((int32_t) -2068750565, PH.base.pack) ;
        p63_lon_SET((int32_t)211531339, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ax_SET((float) -1.850989E38F, PH.base.pack) ;
        p64_vy_SET((float)3.1287184E37F, PH.base.pack) ;
        p64_vx_SET((float) -9.978187E37F, PH.base.pack) ;
        {
            float covariance[] =  {-1.631249E38F, -1.433325E38F, -1.3930306E38F, 3.0396436E38F, 7.2394507E37F, 1.5265495E38F, 3.2204734E38F, 2.8676881E38F, 5.103595E37F, -2.8219797E38F, -4.369845E37F, -3.8006264E37F, -2.0904554E38F, 1.4742114E38F, 3.4643704E37F, -3.1797827E38F, -2.3908915E38F, -1.8366158E38F, 2.4212907E38F, -2.8341966E38F, -1.2210072E38F, 1.6037575E38F, 3.1155805E38F, -1.6267239E38F, 3.200471E38F, 1.2257835E38F, 1.6731687E38F, 2.6999564E38F, -6.6387917E37F, -2.9808897E38F, -2.1626186E38F, -7.245029E37F, 1.091978E38F, -3.0511561E37F, 1.4739452E38F, 2.3621335E37F, 3.2520217E38F, -2.9314469E38F, -2.5881643E38F, -2.2739753E38F, -1.5969954E37F, 2.2660782E36F, 3.0036478E38F, 9.482579E37F, 4.353717E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_az_SET((float) -6.35316E37F, PH.base.pack) ;
        p64_z_SET((float)2.5174768E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)1654117827905533451L, PH.base.pack) ;
        p64_y_SET((float)6.753368E37F, PH.base.pack) ;
        p64_vz_SET((float) -2.7680546E38F, PH.base.pack) ;
        p64_ay_SET((float) -2.7730897E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_x_SET((float)2.4602624E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_time_boot_ms_SET((uint32_t)4036342742L, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)44045, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)41586, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)56954, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)33065, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)53162, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)41528, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)44760, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)45227, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)56993, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)41839, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)43066, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)48346, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)64231, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)64808, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)40630, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)47491, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)65276, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)8646, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)9148, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)65413, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t) -32103, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)27912, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)28765, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)27089, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -18188, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan5_raw_SET((uint16_t)(uint16_t)2417, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)37375, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)17465, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)39844, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)64446, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)42362, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)12551, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)42962, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_current_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p73_param3_SET((float)6.2004786E37F, PH.base.pack) ;
        p73_param4_SET((float) -2.8929695E38F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_z_SET((float)1.0606379E38F, PH.base.pack) ;
        p73_param2_SET((float) -2.3989184E38F, PH.base.pack) ;
        p73_param1_SET((float) -2.2804558E38F, PH.base.pack) ;
        p73_x_SET((int32_t)1934610622, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_YAW, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)48756, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p73_y_SET((int32_t) -576801491, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float)3.2619944E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -26454, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)35986, PH.base.pack) ;
        p74_groundspeed_SET((float)1.8647128E38F, PH.base.pack) ;
        p74_climb_SET((float) -1.4089901E38F, PH.base.pack) ;
        p74_alt_SET((float)2.1527004E37F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param4_SET((float) -3.2558277E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p75_y_SET((int32_t)66100510, PH.base.pack) ;
        p75_z_SET((float)1.0552128E38F, PH.base.pack) ;
        p75_x_SET((int32_t) -1245959934, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_PANORAMA_CREATE, PH.base.pack) ;
        p75_param3_SET((float) -3.1335424E38F, PH.base.pack) ;
        p75_param2_SET((float)2.6478592E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p75_param1_SET((float) -1.5583509E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param2_SET((float) -3.009936E37F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_USER_3, PH.base.pack) ;
        p76_param3_SET((float) -2.8262564E38F, PH.base.pack) ;
        p76_param5_SET((float) -6.427449E37F, PH.base.pack) ;
        p76_param4_SET((float)4.680206E37F, PH.base.pack) ;
        p76_param1_SET((float)1.8260963E38F, PH.base.pack) ;
        p76_param6_SET((float) -2.652674E38F, PH.base.pack) ;
        p76_param7_SET((float) -2.7515892E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)235, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)57, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)66, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_LAST, PH.base.pack) ;
        p77_result_param2_SET((int32_t)1331592986, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p81_roll_SET((float) -2.7265791E38F, PH.base.pack) ;
        p81_thrust_SET((float) -2.8936236E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)965649189L, PH.base.pack) ;
        p81_pitch_SET((float)3.259313E36F, PH.base.pack) ;
        p81_yaw_SET((float)7.1277585E37F, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_thrust_SET((float) -2.1038188E37F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -2.7465768E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)2928638263L, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -1.663323E38F, PH.base.pack) ;
        {
            float q[] =  {-2.7567377E37F, -4.1606904E37F, -2.5716185E38F, -2.0604985E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -6.4952596E37F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float) -1.1005224E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -3.2264484E38F, PH.base.pack) ;
        {
            float q[] =  {9.979019E37F, 7.7050085E37F, 1.3057248E38F, 1.8285688E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_pitch_rate_SET((float) -8.714316E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)2639285890L, PH.base.pack) ;
        p83_thrust_SET((float)3.1406887E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afz_SET((float)5.435867E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)46380, PH.base.pack) ;
        p84_vy_SET((float) -1.019203E38F, PH.base.pack) ;
        p84_y_SET((float) -8.473599E37F, PH.base.pack) ;
        p84_vz_SET((float) -6.172216E37F, PH.base.pack) ;
        p84_yaw_SET((float) -6.534373E36F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)4175635791L, PH.base.pack) ;
        p84_z_SET((float) -2.9479002E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.1937035E38F, PH.base.pack) ;
        p84_afx_SET((float)2.9040255E38F, PH.base.pack) ;
        p84_x_SET((float) -2.4298211E38F, PH.base.pack) ;
        p84_vx_SET((float) -1.9087123E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -2.05703E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_lon_int_SET((int32_t)1519567279, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)4225103150L, PH.base.pack) ;
        p86_vz_SET((float)1.1468529E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p86_yaw_rate_SET((float)7.7330626E37F, PH.base.pack) ;
        p86_alt_SET((float) -9.30516E37F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)1014553364, PH.base.pack) ;
        p86_yaw_SET((float) -2.7295694E37F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)45261, PH.base.pack) ;
        p86_vx_SET((float) -1.200121E38F, PH.base.pack) ;
        p86_vy_SET((float)2.6874738E38F, PH.base.pack) ;
        p86_afy_SET((float) -2.756889E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p86_afx_SET((float)9.124705E37F, PH.base.pack) ;
        p86_afz_SET((float) -3.0465412E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_yaw_SET((float) -1.5781232E38F, PH.base.pack) ;
        p87_afz_SET((float)1.6059814E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)55289, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.9268498E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -2029287617, PH.base.pack) ;
        p87_vx_SET((float)8.2430917E37F, PH.base.pack) ;
        p87_alt_SET((float) -1.7616987E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p87_vz_SET((float) -4.206204E37F, PH.base.pack) ;
        p87_vy_SET((float)2.6259656E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1321325930, PH.base.pack) ;
        p87_afx_SET((float)2.661108E38F, PH.base.pack) ;
        p87_afy_SET((float) -6.420202E37F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)89403356L, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_y_SET((float) -2.9060673E38F, PH.base.pack) ;
        p89_pitch_SET((float)3.3404557E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)3197860420L, PH.base.pack) ;
        p89_x_SET((float) -2.4457022E38F, PH.base.pack) ;
        p89_yaw_SET((float)4.69269E37F, PH.base.pack) ;
        p89_roll_SET((float) -3.0683067E38F, PH.base.pack) ;
        p89_z_SET((float)7.551368E37F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_roll_SET((float) -2.3509759E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)124457987584244553L, PH.base.pack) ;
        p90_lat_SET((int32_t)1983905422, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -10946, PH.base.pack) ;
        p90_lon_SET((int32_t)876226594, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)171, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)17712, PH.base.pack) ;
        p90_yaw_SET((float)1.2493468E38F, PH.base.pack) ;
        p90_rollspeed_SET((float)3.0688927E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)10385, PH.base.pack) ;
        p90_pitchspeed_SET((float) -1.3221695E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -5681, PH.base.pack) ;
        p90_alt_SET((int32_t)1093540743, PH.base.pack) ;
        p90_pitch_SET((float)7.4840575E37F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -25685, PH.base.pack) ;
        p90_yawspeed_SET((float) -1.4500534E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_roll_ailerons_SET((float)2.5348205E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)1.0668024E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p91_aux4_SET((float)9.131819E36F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p91_aux2_SET((float) -9.3942924E36F, PH.base.pack) ;
        p91_aux3_SET((float) -2.9307922E37F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -4.1973395E37F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)5319421263644823112L, PH.base.pack) ;
        p91_aux1_SET((float) -2.5184782E38F, PH.base.pack) ;
        p91_throttle_SET((float)6.3778407E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan7_raw_SET((uint16_t)(uint16_t)27796, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)24820, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)59035, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)2700, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)58772, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)62196, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)10636, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)46825, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)35699, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)13468, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)34094, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)2578351291580659787L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)61795, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)3650995472913210785L, PH.base.pack) ;
        {
            float controls[] =  {7.553762E37F, 1.5830987E38F, 2.3934902E38F, 3.060731E38F, -3.2026963E38F, 3.1636857E38F, 1.1598541E38F, -1.6381032E38F, 8.846658E37F, 2.8108501E38F, -1.1019242E38F, 1.7672964E38F, 4.4532905E37F, -2.3768278E38F, 8.378375E37F, -1.6903183E36F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)87672674486249522L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)1.773371E38F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -25800, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)1.4826549E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)8369373042498735927L, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)29684, PH.base.pack) ;
        p100_ground_distance_SET((float) -3.2858003E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -1.134021E38F, &PH) ;
        p100_flow_rate_x_SET((float) -3.2568262E38F, &PH) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)8399314785562518344L, PH.base.pack) ;
        p101_z_SET((float)2.738303E38F, PH.base.pack) ;
        p101_yaw_SET((float)5.509464E36F, PH.base.pack) ;
        p101_y_SET((float) -1.9296504E38F, PH.base.pack) ;
        p101_roll_SET((float) -3.9043578E37F, PH.base.pack) ;
        p101_x_SET((float) -5.8924933E37F, PH.base.pack) ;
        p101_pitch_SET((float) -3.2479876E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_usec_SET((uint64_t)2603630122814068226L, PH.base.pack) ;
        p102_pitch_SET((float) -1.668868E38F, PH.base.pack) ;
        p102_y_SET((float) -3.1870166E38F, PH.base.pack) ;
        p102_x_SET((float) -2.2787364E38F, PH.base.pack) ;
        p102_yaw_SET((float)1.5270555E38F, PH.base.pack) ;
        p102_roll_SET((float) -2.0582498E38F, PH.base.pack) ;
        p102_z_SET((float) -8.2824036E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -2.9248105E38F, PH.base.pack) ;
        p103_x_SET((float)2.2553145E38F, PH.base.pack) ;
        p103_y_SET((float)4.401698E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)3072414355446520441L, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_pitch_SET((float) -1.3233016E37F, PH.base.pack) ;
        p104_z_SET((float) -2.1742376E38F, PH.base.pack) ;
        p104_y_SET((float) -4.0174834E37F, PH.base.pack) ;
        p104_x_SET((float) -2.3404006E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.6711775E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)8515427990926803105L, PH.base.pack) ;
        p104_roll_SET((float)5.7385873E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_xacc_SET((float)1.1782019E38F, PH.base.pack) ;
        p105_xgyro_SET((float)2.5883959E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -1.5385457E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -5.8666946E37F, PH.base.pack) ;
        p105_temperature_SET((float) -1.5752714E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -5.9616173E37F, PH.base.pack) ;
        p105_zmag_SET((float)1.4931904E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.5879164E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4101463850505522576L, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)57636, PH.base.pack) ;
        p105_diff_pressure_SET((float)1.5966447E37F, PH.base.pack) ;
        p105_zacc_SET((float)1.5409481E38F, PH.base.pack) ;
        p105_yacc_SET((float) -1.3439534E38F, PH.base.pack) ;
        p105_ymag_SET((float) -1.9801309E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.9246022E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_quality_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)387601100L, PH.base.pack) ;
        p106_integrated_y_SET((float) -1.6237024E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)5.020694E37F, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.1543438E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)26929, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)5742230153494084550L, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)300786032L, PH.base.pack) ;
        p106_distance_SET((float) -4.7138363E37F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.6163753E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -9.893342E37F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_pressure_alt_SET((float) -3.1703372E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)4237697076721673836L, PH.base.pack) ;
        p107_ymag_SET((float)1.1284062E38F, PH.base.pack) ;
        p107_zmag_SET((float)5.401285E36F, PH.base.pack) ;
        p107_xgyro_SET((float) -3.2418647E38F, PH.base.pack) ;
        p107_ygyro_SET((float) -1.8395069E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)2.743232E38F, PH.base.pack) ;
        p107_temperature_SET((float) -2.5988087E38F, PH.base.pack) ;
        p107_zgyro_SET((float)3.3551736E38F, PH.base.pack) ;
        p107_xacc_SET((float) -1.001909E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)1217442050L, PH.base.pack) ;
        p107_xmag_SET((float)5.667068E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.8820095E38F, PH.base.pack) ;
        p107_zacc_SET((float) -2.0610773E37F, PH.base.pack) ;
        p107_yacc_SET((float) -5.0136935E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_ygyro_SET((float) -2.5701012E38F, PH.base.pack) ;
        p108_vn_SET((float)1.4135707E38F, PH.base.pack) ;
        p108_ve_SET((float)3.2188797E38F, PH.base.pack) ;
        p108_yaw_SET((float) -2.0378771E38F, PH.base.pack) ;
        p108_q1_SET((float) -1.7509023E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -1.6139322E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -2.5652269E38F, PH.base.pack) ;
        p108_xacc_SET((float) -8.772037E37F, PH.base.pack) ;
        p108_roll_SET((float) -1.3328574E38F, PH.base.pack) ;
        p108_vd_SET((float)3.124242E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -8.799285E37F, PH.base.pack) ;
        p108_lon_SET((float)2.9779957E37F, PH.base.pack) ;
        p108_q3_SET((float)2.7046301E38F, PH.base.pack) ;
        p108_zacc_SET((float) -2.7327665E38F, PH.base.pack) ;
        p108_yacc_SET((float) -2.3961576E38F, PH.base.pack) ;
        p108_q2_SET((float)1.715343E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)8.985911E37F, PH.base.pack) ;
        p108_lat_SET((float)2.6722213E38F, PH.base.pack) ;
        p108_q4_SET((float)9.328354E37F, PH.base.pack) ;
        p108_alt_SET((float)1.4719493E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.2379998E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)56893, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)51048, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)122, (uint8_t)221, (uint8_t)215, (uint8_t)157, (uint8_t)24, (uint8_t)204, (uint8_t)158, (uint8_t)23, (uint8_t)244, (uint8_t)10, (uint8_t)33, (uint8_t)233, (uint8_t)80, (uint8_t)233, (uint8_t)106, (uint8_t)249, (uint8_t)121, (uint8_t)16, (uint8_t)52, (uint8_t)209, (uint8_t)154, (uint8_t)177, (uint8_t)72, (uint8_t)38, (uint8_t)79, (uint8_t)9, (uint8_t)150, (uint8_t)251, (uint8_t)119, (uint8_t)248, (uint8_t)192, (uint8_t)60, (uint8_t)108, (uint8_t)136, (uint8_t)202, (uint8_t)29, (uint8_t)197, (uint8_t)221, (uint8_t)37, (uint8_t)190, (uint8_t)109, (uint8_t)114, (uint8_t)36, (uint8_t)251, (uint8_t)243, (uint8_t)173, (uint8_t)106, (uint8_t)242, (uint8_t)219, (uint8_t)96, (uint8_t)207, (uint8_t)111, (uint8_t)243, (uint8_t)254, (uint8_t)35, (uint8_t)127, (uint8_t)69, (uint8_t)94, (uint8_t)168, (uint8_t)26, (uint8_t)133, (uint8_t)168, (uint8_t)42, (uint8_t)255, (uint8_t)122, (uint8_t)91, (uint8_t)92, (uint8_t)215, (uint8_t)245, (uint8_t)166, (uint8_t)49, (uint8_t)101, (uint8_t)56, (uint8_t)231, (uint8_t)91, (uint8_t)125, (uint8_t)234, (uint8_t)10, (uint8_t)136, (uint8_t)141, (uint8_t)96, (uint8_t)220, (uint8_t)10, (uint8_t)191, (uint8_t)47, (uint8_t)68, (uint8_t)182, (uint8_t)2, (uint8_t)187, (uint8_t)175, (uint8_t)154, (uint8_t)213, (uint8_t)73, (uint8_t)237, (uint8_t)92, (uint8_t)194, (uint8_t)161, (uint8_t)199, (uint8_t)138, (uint8_t)196, (uint8_t)43, (uint8_t)140, (uint8_t)115, (uint8_t)48, (uint8_t)60, (uint8_t)208, (uint8_t)124, (uint8_t)40, (uint8_t)64, (uint8_t)43, (uint8_t)129, (uint8_t)93, (uint8_t)0, (uint8_t)1, (uint8_t)5, (uint8_t)227, (uint8_t)169, (uint8_t)26, (uint8_t)194, (uint8_t)204, (uint8_t)240, (uint8_t)243, (uint8_t)121, (uint8_t)146, (uint8_t)153, (uint8_t)172, (uint8_t)230, (uint8_t)38, (uint8_t)217, (uint8_t)200, (uint8_t)99, (uint8_t)178, (uint8_t)65, (uint8_t)247, (uint8_t)85, (uint8_t)179, (uint8_t)237, (uint8_t)2, (uint8_t)185, (uint8_t)100, (uint8_t)54, (uint8_t)237, (uint8_t)99, (uint8_t)65, (uint8_t)30, (uint8_t)15, (uint8_t)57, (uint8_t)92, (uint8_t)160, (uint8_t)248, (uint8_t)123, (uint8_t)141, (uint8_t)196, (uint8_t)6, (uint8_t)129, (uint8_t)144, (uint8_t)209, (uint8_t)17, (uint8_t)185, (uint8_t)51, (uint8_t)213, (uint8_t)123, (uint8_t)42, (uint8_t)62, (uint8_t)181, (uint8_t)123, (uint8_t)171, (uint8_t)137, (uint8_t)60, (uint8_t)193, (uint8_t)61, (uint8_t)185, (uint8_t)66, (uint8_t)184, (uint8_t)142, (uint8_t)146, (uint8_t)91, (uint8_t)209, (uint8_t)79, (uint8_t)168, (uint8_t)137, (uint8_t)101, (uint8_t)121, (uint8_t)19, (uint8_t)245, (uint8_t)235, (uint8_t)5, (uint8_t)175, (uint8_t)135, (uint8_t)55, (uint8_t)243, (uint8_t)224, (uint8_t)201, (uint8_t)251, (uint8_t)56, (uint8_t)245, (uint8_t)218, (uint8_t)84, (uint8_t)247, (uint8_t)132, (uint8_t)6, (uint8_t)7, (uint8_t)190, (uint8_t)92, (uint8_t)223, (uint8_t)92, (uint8_t)191, (uint8_t)89, (uint8_t)72, (uint8_t)152, (uint8_t)32, (uint8_t)153, (uint8_t)35, (uint8_t)62, (uint8_t)129, (uint8_t)206, (uint8_t)163, (uint8_t)45, (uint8_t)131, (uint8_t)113, (uint8_t)60, (uint8_t)16, (uint8_t)163, (uint8_t)253, (uint8_t)55, (uint8_t)30, (uint8_t)114, (uint8_t)106, (uint8_t)33, (uint8_t)19, (uint8_t)253, (uint8_t)216, (uint8_t)84, (uint8_t)221, (uint8_t)121, (uint8_t)20, (uint8_t)158, (uint8_t)249, (uint8_t)195, (uint8_t)76, (uint8_t)111, (uint8_t)184, (uint8_t)88, (uint8_t)27, (uint8_t)72, (uint8_t)147, (uint8_t)232, (uint8_t)117, (uint8_t)66, (uint8_t)95, (uint8_t)58};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)2918086017587043568L, PH.base.pack) ;
        p111_tc1_SET((int64_t)8659119403300594770L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)2910297332L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)297739231722986962L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_alt_SET((int32_t)1017717923, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)56288, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)22051, PH.base.pack) ;
        p113_lat_SET((int32_t)1355557602, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -6537, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)26282, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -32059, PH.base.pack) ;
        p113_lon_SET((int32_t)2138381134, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)7361583187616750593L, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)61652, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)30260, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_quality_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p114_integrated_x_SET((float)2.5170178E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)18390, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)1.6721967E38F, PH.base.pack) ;
        p114_distance_SET((float) -1.3816953E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)1305184856L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4099773066L, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.3479937E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -6.7340585E36F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)5490094233126551350L, PH.base.pack) ;
        p114_integrated_y_SET((float) -6.071221E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_xacc_SET((int16_t)(int16_t) -22569, PH.base.pack) ;
        p115_alt_SET((int32_t)470967320, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)50829, PH.base.pack) ;
        p115_rollspeed_SET((float) -9.5004336E35F, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -25542, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)8360879872211272766L, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.9662779E38F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)8777, PH.base.pack) ;
        p115_lat_SET((int32_t)558597971, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)6563, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -23099, PH.base.pack) ;
        p115_lon_SET((int32_t) -1528245611, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)8137, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -14816, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {2.150577E38F, -2.4061532E37F, 5.940041E37F, -6.9653374E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_yawspeed_SET((float) -1.1938568E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_xmag_SET((int16_t)(int16_t)12066, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)15645, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)22741, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)21145, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)31025, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -26041, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)3804, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)3835916301L, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -20016, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)588, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)16325, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)12184, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_id_SET((uint16_t)(uint16_t)15937, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)63871, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)623653489L, PH.base.pack) ;
        p118_size_SET((uint32_t)3792280005L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)43113, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)1313965482L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p119_ofs_SET((uint32_t)3768331389L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)33445, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_id_SET((uint16_t)(uint16_t)1487, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)76, (uint8_t)172, (uint8_t)179, (uint8_t)61, (uint8_t)29, (uint8_t)2, (uint8_t)149, (uint8_t)128, (uint8_t)200, (uint8_t)13, (uint8_t)205, (uint8_t)32, (uint8_t)223, (uint8_t)14, (uint8_t)56, (uint8_t)7, (uint8_t)47, (uint8_t)93, (uint8_t)128, (uint8_t)177, (uint8_t)182, (uint8_t)179, (uint8_t)241, (uint8_t)236, (uint8_t)253, (uint8_t)144, (uint8_t)220, (uint8_t)136, (uint8_t)158, (uint8_t)136, (uint8_t)53, (uint8_t)213, (uint8_t)130, (uint8_t)137, (uint8_t)36, (uint8_t)46, (uint8_t)152, (uint8_t)62, (uint8_t)104, (uint8_t)252, (uint8_t)252, (uint8_t)171, (uint8_t)117, (uint8_t)206, (uint8_t)55, (uint8_t)31, (uint8_t)16, (uint8_t)118, (uint8_t)223, (uint8_t)77, (uint8_t)231, (uint8_t)79, (uint8_t)61, (uint8_t)202, (uint8_t)38, (uint8_t)115, (uint8_t)234, (uint8_t)62, (uint8_t)247, (uint8_t)105, (uint8_t)240, (uint8_t)133, (uint8_t)21, (uint8_t)225, (uint8_t)125, (uint8_t)31, (uint8_t)50, (uint8_t)229, (uint8_t)184, (uint8_t)222, (uint8_t)214, (uint8_t)15, (uint8_t)106, (uint8_t)177, (uint8_t)56, (uint8_t)65, (uint8_t)93, (uint8_t)91, (uint8_t)115, (uint8_t)193, (uint8_t)17, (uint8_t)249, (uint8_t)49, (uint8_t)129, (uint8_t)145, (uint8_t)46, (uint8_t)71, (uint8_t)75, (uint8_t)111, (uint8_t)201};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)3512122345L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)195, (uint8_t)145, (uint8_t)167, (uint8_t)236, (uint8_t)183, (uint8_t)223, (uint8_t)1, (uint8_t)6, (uint8_t)50, (uint8_t)137, (uint8_t)179, (uint8_t)113, (uint8_t)176, (uint8_t)17, (uint8_t)10, (uint8_t)28, (uint8_t)133, (uint8_t)108, (uint8_t)32, (uint8_t)102, (uint8_t)243, (uint8_t)162, (uint8_t)66, (uint8_t)39, (uint8_t)180, (uint8_t)92, (uint8_t)190, (uint8_t)222, (uint8_t)229, (uint8_t)9, (uint8_t)146, (uint8_t)106, (uint8_t)229, (uint8_t)12, (uint8_t)255, (uint8_t)154, (uint8_t)231, (uint8_t)236, (uint8_t)249, (uint8_t)104, (uint8_t)97, (uint8_t)67, (uint8_t)7, (uint8_t)96, (uint8_t)92, (uint8_t)173, (uint8_t)199, (uint8_t)183, (uint8_t)140, (uint8_t)202, (uint8_t)109, (uint8_t)23, (uint8_t)16, (uint8_t)6, (uint8_t)45, (uint8_t)78, (uint8_t)58, (uint8_t)194, (uint8_t)112, (uint8_t)217, (uint8_t)93, (uint8_t)16, (uint8_t)58, (uint8_t)11, (uint8_t)99, (uint8_t)155, (uint8_t)47, (uint8_t)165, (uint8_t)109, (uint8_t)87, (uint8_t)57, (uint8_t)111, (uint8_t)36, (uint8_t)212, (uint8_t)41, (uint8_t)155, (uint8_t)92, (uint8_t)58, (uint8_t)198, (uint8_t)162, (uint8_t)68, (uint8_t)239, (uint8_t)185, (uint8_t)100, (uint8_t)184, (uint8_t)61, (uint8_t)224, (uint8_t)226, (uint8_t)74, (uint8_t)83, (uint8_t)213, (uint8_t)95, (uint8_t)89, (uint8_t)231, (uint8_t)227, (uint8_t)46, (uint8_t)48, (uint8_t)24, (uint8_t)171, (uint8_t)235, (uint8_t)48, (uint8_t)131, (uint8_t)165, (uint8_t)163, (uint8_t)217, (uint8_t)57, (uint8_t)150, (uint8_t)99, (uint8_t)54, (uint8_t)73};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)2361107573L, PH.base.pack) ;
        p124_lon_SET((int32_t) -969902343, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)22651, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)35319, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)51340, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p124_alt_SET((int32_t)1140683915, PH.base.pack) ;
        p124_lat_SET((int32_t) -1257696601, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)28509, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)1479048031372739085L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)25504, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID), PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)26630, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)173, (uint8_t)162, (uint8_t)182, (uint8_t)51, (uint8_t)195, (uint8_t)139, (uint8_t)78, (uint8_t)91, (uint8_t)128, (uint8_t)29, (uint8_t)135, (uint8_t)208, (uint8_t)184, (uint8_t)127, (uint8_t)141, (uint8_t)11, (uint8_t)45, (uint8_t)130, (uint8_t)30, (uint8_t)165, (uint8_t)123, (uint8_t)232, (uint8_t)254, (uint8_t)153, (uint8_t)53, (uint8_t)108, (uint8_t)252, (uint8_t)247, (uint8_t)77, (uint8_t)221, (uint8_t)207, (uint8_t)71, (uint8_t)220, (uint8_t)84, (uint8_t)126, (uint8_t)219, (uint8_t)79, (uint8_t)165, (uint8_t)10, (uint8_t)2, (uint8_t)124, (uint8_t)87, (uint8_t)109, (uint8_t)179, (uint8_t)91, (uint8_t)116, (uint8_t)54, (uint8_t)91, (uint8_t)111, (uint8_t)161, (uint8_t)129, (uint8_t)50, (uint8_t)220, (uint8_t)11, (uint8_t)143, (uint8_t)125, (uint8_t)85, (uint8_t)119, (uint8_t)205, (uint8_t)190, (uint8_t)69, (uint8_t)28, (uint8_t)120, (uint8_t)198, (uint8_t)250, (uint8_t)169, (uint8_t)159, (uint8_t)85, (uint8_t)239, (uint8_t)3};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI), PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)1182, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1016690666L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)1203394775L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)703433813, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)487519151, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1586554802, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)249153682L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)752014926L, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)738851248, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)61201, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_nsats_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)6060, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p128_tow_SET((uint32_t)3441757651L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2621767620L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4151804849L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)1985071772, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)1617431880, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)594893478, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)50839164, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_time_boot_ms_SET((uint32_t)2264218689L, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)546, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)14127, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -28506, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -12821, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -511, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)20249, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -26699, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -22534, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)16666, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_payload_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)50584, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)58451, PH.base.pack) ;
        p130_size_SET((uint32_t)4237625042L, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)27299, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)249, (uint8_t)163, (uint8_t)40, (uint8_t)98, (uint8_t)213, (uint8_t)39, (uint8_t)217, (uint8_t)191, (uint8_t)137, (uint8_t)176, (uint8_t)134, (uint8_t)6, (uint8_t)209, (uint8_t)174, (uint8_t)220, (uint8_t)248, (uint8_t)142, (uint8_t)45, (uint8_t)14, (uint8_t)143, (uint8_t)74, (uint8_t)22, (uint8_t)121, (uint8_t)63, (uint8_t)254, (uint8_t)233, (uint8_t)249, (uint8_t)125, (uint8_t)168, (uint8_t)8, (uint8_t)138, (uint8_t)110, (uint8_t)199, (uint8_t)20, (uint8_t)141, (uint8_t)31, (uint8_t)94, (uint8_t)186, (uint8_t)60, (uint8_t)99, (uint8_t)175, (uint8_t)149, (uint8_t)179, (uint8_t)162, (uint8_t)165, (uint8_t)217, (uint8_t)248, (uint8_t)55, (uint8_t)138, (uint8_t)13, (uint8_t)75, (uint8_t)56, (uint8_t)98, (uint8_t)40, (uint8_t)8, (uint8_t)138, (uint8_t)233, (uint8_t)108, (uint8_t)34, (uint8_t)135, (uint8_t)24, (uint8_t)67, (uint8_t)101, (uint8_t)201, (uint8_t)11, (uint8_t)171, (uint8_t)143, (uint8_t)99, (uint8_t)99, (uint8_t)236, (uint8_t)140, (uint8_t)3, (uint8_t)177, (uint8_t)15, (uint8_t)30, (uint8_t)35, (uint8_t)149, (uint8_t)162, (uint8_t)69, (uint8_t)156, (uint8_t)137, (uint8_t)158, (uint8_t)91, (uint8_t)113, (uint8_t)137, (uint8_t)68, (uint8_t)192, (uint8_t)44, (uint8_t)60, (uint8_t)8, (uint8_t)99, (uint8_t)78, (uint8_t)235, (uint8_t)106, (uint8_t)25, (uint8_t)31, (uint8_t)46, (uint8_t)185, (uint8_t)129, (uint8_t)241, (uint8_t)58, (uint8_t)137, (uint8_t)75, (uint8_t)186, (uint8_t)23, (uint8_t)12, (uint8_t)39, (uint8_t)231, (uint8_t)175, (uint8_t)141, (uint8_t)131, (uint8_t)78, (uint8_t)118, (uint8_t)156, (uint8_t)141, (uint8_t)148, (uint8_t)69, (uint8_t)118, (uint8_t)5, (uint8_t)1, (uint8_t)195, (uint8_t)219, (uint8_t)154, (uint8_t)27, (uint8_t)207, (uint8_t)251, (uint8_t)159, (uint8_t)50, (uint8_t)225, (uint8_t)227, (uint8_t)199, (uint8_t)49, (uint8_t)147, (uint8_t)28, (uint8_t)40, (uint8_t)109, (uint8_t)225, (uint8_t)82, (uint8_t)176, (uint8_t)44, (uint8_t)172, (uint8_t)187, (uint8_t)255, (uint8_t)153, (uint8_t)248, (uint8_t)38, (uint8_t)41, (uint8_t)222, (uint8_t)149, (uint8_t)216, (uint8_t)143, (uint8_t)230, (uint8_t)214, (uint8_t)25, (uint8_t)68, (uint8_t)233, (uint8_t)198, (uint8_t)3, (uint8_t)0, (uint8_t)82, (uint8_t)207, (uint8_t)14, (uint8_t)232, (uint8_t)164, (uint8_t)204, (uint8_t)233, (uint8_t)94, (uint8_t)168, (uint8_t)59, (uint8_t)157, (uint8_t)33, (uint8_t)95, (uint8_t)198, (uint8_t)243, (uint8_t)141, (uint8_t)122, (uint8_t)223, (uint8_t)163, (uint8_t)6, (uint8_t)135, (uint8_t)137, (uint8_t)215, (uint8_t)180, (uint8_t)246, (uint8_t)208, (uint8_t)140, (uint8_t)213, (uint8_t)175, (uint8_t)78, (uint8_t)44, (uint8_t)106, (uint8_t)40, (uint8_t)242, (uint8_t)244, (uint8_t)65, (uint8_t)37, (uint8_t)157, (uint8_t)155, (uint8_t)133, (uint8_t)214, (uint8_t)142, (uint8_t)172, (uint8_t)119, (uint8_t)174, (uint8_t)129, (uint8_t)205, (uint8_t)30, (uint8_t)13, (uint8_t)64, (uint8_t)43, (uint8_t)238, (uint8_t)57, (uint8_t)1, (uint8_t)226, (uint8_t)194, (uint8_t)205, (uint8_t)248, (uint8_t)173, (uint8_t)86, (uint8_t)173, (uint8_t)213, (uint8_t)195, (uint8_t)73, (uint8_t)132, (uint8_t)77, (uint8_t)21, (uint8_t)250, (uint8_t)164, (uint8_t)28, (uint8_t)40, (uint8_t)201, (uint8_t)200, (uint8_t)228, (uint8_t)234, (uint8_t)58, (uint8_t)177, (uint8_t)252, (uint8_t)33, (uint8_t)45, (uint8_t)216, (uint8_t)113, (uint8_t)142, (uint8_t)42, (uint8_t)20, (uint8_t)80, (uint8_t)173, (uint8_t)12, (uint8_t)212, (uint8_t)13, (uint8_t)130, (uint8_t)10, (uint8_t)3, (uint8_t)209};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)38364, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_time_boot_ms_SET((uint32_t)1757479355L, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)47144, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)39361, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)30766, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)6139, PH.base.pack) ;
        p133_lat_SET((int32_t) -1086956974, PH.base.pack) ;
        p133_mask_SET((uint64_t)1341659818993430521L, PH.base.pack) ;
        p133_lon_SET((int32_t)1151851511, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t)2108905097, PH.base.pack) ;
        p134_lon_SET((int32_t) -601745621, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)20808, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)24426, (int16_t) -18106, (int16_t)8001, (int16_t) -9786, (int16_t)10975, (int16_t)11624, (int16_t)26350, (int16_t)32643, (int16_t) -7936, (int16_t) -18762, (int16_t)25100, (int16_t) -29868, (int16_t)17833, (int16_t)16462, (int16_t) -17988, (int16_t) -31281};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -1594904275, PH.base.pack) ;
        p135_lon_SET((int32_t)2032225830, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float) -3.10456E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)5623, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)35256, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)60893, PH.base.pack) ;
        p136_lat_SET((int32_t) -942876852, PH.base.pack) ;
        p136_terrain_height_SET((float)2.2009664E38F, PH.base.pack) ;
        p136_lon_SET((int32_t) -292312307, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float) -3.0553065E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)4022888425L, PH.base.pack) ;
        p137_press_diff_SET((float) -1.9966732E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -17819, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {2.6538578E38F, 1.458672E36F, 2.8352578E38F, -1.4093457E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_y_SET((float) -3.358048E37F, PH.base.pack) ;
        p138_x_SET((float) -3.1706653E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)839929048415957882L, PH.base.pack) ;
        p138_z_SET((float)3.8530212E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)7007766848845000456L, PH.base.pack) ;
        {
            float controls[] =  {2.1671676E37F, -2.3999457E38F, -3.185691E38F, 1.7666171E38F, 9.979029E36F, -9.340243E37F, -1.9494302E38F, -1.646043E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {3.1928646E38F, -2.3718047E38F, 6.5785874E37F, -3.0729761E38F, 1.3454329E38F, 2.1389937E38F, -1.8533855E38F, -8.77791E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)52000896839502531L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float)1.428131E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -6.4143906E36F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)1.6294973E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -1.4594012E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)1659242072199543845L, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -8.408816E37F, PH.base.pack) ;
        p141_altitude_local_SET((float) -3.3333597E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)39, (uint8_t)25, (uint8_t)22, (uint8_t)43, (uint8_t)238, (uint8_t)249, (uint8_t)27, (uint8_t)15, (uint8_t)148, (uint8_t)54, (uint8_t)127, (uint8_t)156, (uint8_t)157, (uint8_t)135, (uint8_t)160, (uint8_t)158, (uint8_t)140, (uint8_t)185, (uint8_t)71, (uint8_t)121, (uint8_t)247, (uint8_t)7, (uint8_t)185, (uint8_t)207, (uint8_t)102, (uint8_t)201, (uint8_t)15, (uint8_t)200, (uint8_t)226, (uint8_t)238, (uint8_t)233, (uint8_t)185, (uint8_t)20, (uint8_t)24, (uint8_t)218, (uint8_t)71, (uint8_t)97, (uint8_t)78, (uint8_t)206, (uint8_t)100, (uint8_t)223, (uint8_t)149, (uint8_t)63, (uint8_t)20, (uint8_t)174, (uint8_t)133, (uint8_t)116, (uint8_t)83, (uint8_t)31, (uint8_t)195, (uint8_t)136, (uint8_t)224, (uint8_t)110, (uint8_t)206, (uint8_t)98, (uint8_t)141, (uint8_t)187, (uint8_t)192, (uint8_t)194, (uint8_t)114, (uint8_t)109, (uint8_t)103, (uint8_t)65, (uint8_t)254, (uint8_t)81, (uint8_t)110, (uint8_t)54, (uint8_t)165, (uint8_t)36, (uint8_t)123, (uint8_t)28, (uint8_t)38, (uint8_t)73, (uint8_t)184, (uint8_t)26, (uint8_t)48, (uint8_t)58, (uint8_t)204, (uint8_t)242, (uint8_t)22, (uint8_t)114, (uint8_t)167, (uint8_t)133, (uint8_t)7, (uint8_t)119, (uint8_t)140, (uint8_t)122, (uint8_t)116, (uint8_t)46, (uint8_t)74, (uint8_t)121, (uint8_t)91, (uint8_t)111, (uint8_t)6, (uint8_t)96, (uint8_t)242, (uint8_t)58, (uint8_t)4, (uint8_t)99, (uint8_t)141, (uint8_t)197, (uint8_t)58, (uint8_t)176, (uint8_t)27, (uint8_t)170, (uint8_t)64, (uint8_t)227, (uint8_t)23, (uint8_t)80, (uint8_t)106, (uint8_t)167, (uint8_t)242, (uint8_t)121, (uint8_t)46, (uint8_t)212, (uint8_t)191, (uint8_t)86, (uint8_t)48, (uint8_t)175, (uint8_t)79};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        {
            uint8_t uri[] =  {(uint8_t)227, (uint8_t)198, (uint8_t)22, (uint8_t)151, (uint8_t)109, (uint8_t)105, (uint8_t)115, (uint8_t)233, (uint8_t)29, (uint8_t)76, (uint8_t)194, (uint8_t)29, (uint8_t)33, (uint8_t)191, (uint8_t)98, (uint8_t)74, (uint8_t)135, (uint8_t)188, (uint8_t)91, (uint8_t)95, (uint8_t)243, (uint8_t)172, (uint8_t)41, (uint8_t)51, (uint8_t)139, (uint8_t)146, (uint8_t)46, (uint8_t)6, (uint8_t)15, (uint8_t)56, (uint8_t)236, (uint8_t)46, (uint8_t)123, (uint8_t)180, (uint8_t)97, (uint8_t)231, (uint8_t)250, (uint8_t)106, (uint8_t)95, (uint8_t)186, (uint8_t)120, (uint8_t)177, (uint8_t)106, (uint8_t)233, (uint8_t)119, (uint8_t)88, (uint8_t)23, (uint8_t)219, (uint8_t)13, (uint8_t)32, (uint8_t)146, (uint8_t)45, (uint8_t)160, (uint8_t)67, (uint8_t)252, (uint8_t)97, (uint8_t)185, (uint8_t)199, (uint8_t)150, (uint8_t)68, (uint8_t)107, (uint8_t)186, (uint8_t)200, (uint8_t)144, (uint8_t)235, (uint8_t)188, (uint8_t)25, (uint8_t)157, (uint8_t)42, (uint8_t)223, (uint8_t)86, (uint8_t)92, (uint8_t)181, (uint8_t)244, (uint8_t)39, (uint8_t)68, (uint8_t)169, (uint8_t)82, (uint8_t)22, (uint8_t)12, (uint8_t)28, (uint8_t)157, (uint8_t)6, (uint8_t)82, (uint8_t)244, (uint8_t)5, (uint8_t)151, (uint8_t)131, (uint8_t)175, (uint8_t)216, (uint8_t)67, (uint8_t)136, (uint8_t)50, (uint8_t)108, (uint8_t)90, (uint8_t)22, (uint8_t)184, (uint8_t)6, (uint8_t)136, (uint8_t)233, (uint8_t)130, (uint8_t)39, (uint8_t)118, (uint8_t)12, (uint8_t)70, (uint8_t)178, (uint8_t)86, (uint8_t)251, (uint8_t)121, (uint8_t)146, (uint8_t)42, (uint8_t)72, (uint8_t)143, (uint8_t)159, (uint8_t)22, (uint8_t)2, (uint8_t)208, (uint8_t)140, (uint8_t)161, (uint8_t)111};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)1.7927021E38F, PH.base.pack) ;
        p143_press_diff_SET((float)2.9850507E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)2145, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)4044814564L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float rates[] =  {2.7172172E38F, 9.805553E37F, 1.5239787E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)1870450080, PH.base.pack) ;
        p144_lat_SET((int32_t) -367228611, PH.base.pack) ;
        {
            float position_cov[] =  {2.7129729E38F, -3.1803202E38F, -5.605886E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {2.5706717E38F, -3.0066502E38F, -2.6188266E38F, -3.2808438E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-1.2611791E38F, -2.2535998E38F, -3.3308633E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-7.6297096E37F, -1.0794055E37F, 1.6980832E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)2229766762357037528L, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)2532045347439496764L, PH.base.pack) ;
        p144_alt_SET((float)3.1193137E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_airspeed_SET((float) -1.936391E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -3.1456214E38F, PH.base.pack) ;
        {
            float q[] =  {-1.616339E38F, -3.1587916E38F, -1.4472298E38F, 2.3064278E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)5943053024702942339L, PH.base.pack) ;
        p146_x_pos_SET((float) -2.4112706E38F, PH.base.pack) ;
        p146_z_vel_SET((float)1.5645908E38F, PH.base.pack) ;
        p146_z_pos_SET((float)1.7984183E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {4.964177E37F, -2.9974884E38F, -1.885777E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float)9.0160565E36F, PH.base.pack) ;
        p146_x_vel_SET((float) -3.398022E38F, PH.base.pack) ;
        p146_y_pos_SET((float)1.3048024E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -7.145255E36F, PH.base.pack) ;
        {
            float pos_variance[] =  {-3.344565E36F, -4.755511E37F, -6.698451E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float) -5.1510845E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.4582325E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -3.029926E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)2.1895904E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t)1240343418, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -5860, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)47199, (uint16_t)17683, (uint16_t)11456, (uint16_t)20671, (uint16_t)18215, (uint16_t)10922, (uint16_t)28316, (uint16_t)53461, (uint16_t)29626, (uint16_t)15810};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_battery_remaining_SET((int8_t)(int8_t)104, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)389849567, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -22096, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t flight_custom_version[] =  {(uint8_t)246, (uint8_t)224, (uint8_t)65, (uint8_t)84, (uint8_t)38, (uint8_t)141, (uint8_t)8, (uint8_t)11};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)2397672179L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)2449564781L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)213, (uint8_t)193, (uint8_t)57, (uint8_t)61, (uint8_t)138, (uint8_t)209, (uint8_t)17, (uint8_t)184, (uint8_t)75, (uint8_t)5, (uint8_t)10, (uint8_t)7, (uint8_t)154, (uint8_t)234, (uint8_t)40, (uint8_t)241, (uint8_t)241, (uint8_t)104};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)33348, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2387807743L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)126, (uint8_t)80, (uint8_t)104, (uint8_t)23, (uint8_t)60, (uint8_t)249, (uint8_t)147, (uint8_t)41};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)5638697991911850844L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)41596, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT), PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)695412048L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)207, (uint8_t)109, (uint8_t)225, (uint8_t)234, (uint8_t)231, (uint8_t)136, (uint8_t)126, (uint8_t)175};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_time_usec_SET((uint64_t)7768412383148311239L, PH.base.pack) ;
        p149_z_SET((float)9.824239E37F, &PH) ;
        p149_angle_x_SET((float)3.116952E38F, PH.base.pack) ;
        p149_angle_y_SET((float) -2.4744043E38F, PH.base.pack) ;
        p149_size_y_SET((float) -3.3769675E38F, PH.base.pack) ;
        {
            float q[] =  {-1.2529333E38F, -9.347312E37F, -5.9791377E37F, 2.9348933E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_x_SET((float) -1.354389E38F, &PH) ;
        p149_distance_SET((float) -2.204709E38F, PH.base.pack) ;
        p149_y_SET((float)3.1520678E38F, &PH) ;
        p149_size_x_SET((float)3.3777145E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)203, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_SET_150(), &PH);
        p150_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p150_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_READ_REQ_151(), &PH);
        p151_data_index_SET((int16_t)(int16_t) -29336, PH.base.pack) ;
        p151_read_req_type_SET((int16_t)(int16_t)14039, PH.base.pack) ;
        p151_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_152(), &PH);
        p152_func_index_SET((uint16_t)(uint16_t)58208, PH.base.pack) ;
        p152_data_size_SET((uint16_t)(uint16_t)65258, PH.base.pack) ;
        {
            int8_t data_[] =  {(int8_t) -13, (int8_t)111, (int8_t) -67, (int8_t)3, (int8_t) -77, (int8_t)6, (int8_t) -111, (int8_t) -5, (int8_t)88, (int8_t) -13, (int8_t) -125, (int8_t) -71, (int8_t) -43, (int8_t) -89, (int8_t)97, (int8_t)72, (int8_t)43, (int8_t) -49, (int8_t)43, (int8_t)6, (int8_t) -31, (int8_t)109, (int8_t)99, (int8_t)73, (int8_t) -60, (int8_t) -94, (int8_t)20, (int8_t) -47, (int8_t)71, (int8_t) -52, (int8_t) -26, (int8_t)9, (int8_t)16, (int8_t) -3, (int8_t)88, (int8_t)20, (int8_t)87, (int8_t)112, (int8_t)24, (int8_t) -87, (int8_t) -13, (int8_t) -27, (int8_t)42, (int8_t) -126, (int8_t) -113, (int8_t)94, (int8_t) -126, (int8_t) -101};
            p152_data__SET(&data_, 0, PH.base.pack) ;
        }
        p152_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p152_func_count_SET((uint16_t)(uint16_t)6592, PH.base.pack) ;
        p152_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p152_data_address_SET((uint16_t)(uint16_t)956, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(), &PH);
        p153_target_component_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p153_func_index_SET((uint16_t)(uint16_t)23154, PH.base.pack) ;
        p153_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p153_result_SET((uint16_t)(uint16_t)36893, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_155(), &PH);
        p155_directory_type_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p155_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        {
            int8_t directory_data[] =  {(int8_t) -127, (int8_t)85, (int8_t)94, (int8_t)1, (int8_t)4, (int8_t) -6, (int8_t)68, (int8_t)67, (int8_t) -11, (int8_t) -32, (int8_t)99, (int8_t) -48, (int8_t)7, (int8_t) -94, (int8_t)52, (int8_t)70, (int8_t)37, (int8_t)102, (int8_t) -57, (int8_t)98, (int8_t)78, (int8_t) -122, (int8_t)45, (int8_t)64, (int8_t) -118, (int8_t) -17, (int8_t) -113, (int8_t) -10, (int8_t)56, (int8_t) -111, (int8_t) -36, (int8_t) -57, (int8_t)125, (int8_t) -65, (int8_t)6, (int8_t) -88, (int8_t) -102, (int8_t) -84, (int8_t)60, (int8_t) -44, (int8_t)20, (int8_t)58, (int8_t)125, (int8_t) -58, (int8_t)29, (int8_t) -125, (int8_t)50, (int8_t) -8};
            p155_directory_data_SET(&directory_data, 0, PH.base.pack) ;
        }
        p155_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p155_count_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p155_start_index_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_ACK_156(), &PH);
        p156_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p156_count_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p156_result_SET((uint16_t)(uint16_t)19646, PH.base.pack) ;
        p156_start_index_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p156_directory_type_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_157(), &PH);
        p157_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p157_command_type_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p157_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_ACK_158(), &PH);
        p158_command_type_SET((uint16_t)(uint16_t)11328, PH.base.pack) ;
        p158_result_SET((uint16_t)(uint16_t)56774, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_A_170(), &PH);
        p170_sue_sog_SET((int16_t)(int16_t) -10243, PH.base.pack) ;
        p170_sue_hdop_SET((int16_t)(int16_t) -5693, PH.base.pack) ;
        p170_sue_estimated_wind_2_SET((int16_t)(int16_t) -25725, PH.base.pack) ;
        p170_sue_longitude_SET((int32_t)645671859, PH.base.pack) ;
        p170_sue_altitude_SET((int32_t)411183183, PH.base.pack) ;
        p170_sue_latitude_SET((int32_t)1760906038, PH.base.pack) ;
        p170_sue_cog_SET((uint16_t)(uint16_t)22745, PH.base.pack) ;
        p170_sue_waypoint_index_SET((uint16_t)(uint16_t)12823, PH.base.pack) ;
        p170_sue_svs_SET((int16_t)(int16_t) -4963, PH.base.pack) ;
        p170_sue_rmat4_SET((int16_t)(int16_t) -3328, PH.base.pack) ;
        p170_sue_cpu_load_SET((uint16_t)(uint16_t)35421, PH.base.pack) ;
        p170_sue_status_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p170_sue_magFieldEarth2_SET((int16_t)(int16_t) -4606, PH.base.pack) ;
        p170_sue_rmat1_SET((int16_t)(int16_t) -1432, PH.base.pack) ;
        p170_sue_rmat2_SET((int16_t)(int16_t) -19467, PH.base.pack) ;
        p170_sue_air_speed_3DIMU_SET((uint16_t)(uint16_t)65095, PH.base.pack) ;
        p170_sue_rmat7_SET((int16_t)(int16_t)6225, PH.base.pack) ;
        p170_sue_rmat5_SET((int16_t)(int16_t)3727, PH.base.pack) ;
        p170_sue_magFieldEarth1_SET((int16_t)(int16_t)23175, PH.base.pack) ;
        p170_sue_rmat0_SET((int16_t)(int16_t) -28396, PH.base.pack) ;
        p170_sue_magFieldEarth0_SET((int16_t)(int16_t)10695, PH.base.pack) ;
        p170_sue_rmat3_SET((int16_t)(int16_t)3111, PH.base.pack) ;
        p170_sue_rmat6_SET((int16_t)(int16_t)4717, PH.base.pack) ;
        p170_sue_time_SET((uint32_t)2670418949L, PH.base.pack) ;
        p170_sue_estimated_wind_1_SET((int16_t)(int16_t) -20861, PH.base.pack) ;
        p170_sue_rmat8_SET((int16_t)(int16_t) -9003, PH.base.pack) ;
        p170_sue_estimated_wind_0_SET((int16_t)(int16_t)1282, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_B_171(), &PH);
        p171_sue_aero_z_SET((int16_t)(int16_t) -10995, PH.base.pack) ;
        p171_sue_pwm_input_12_SET((int16_t)(int16_t)29850, PH.base.pack) ;
        p171_sue_pwm_output_1_SET((int16_t)(int16_t) -3734, PH.base.pack) ;
        p171_sue_location_error_earth_y_SET((int16_t)(int16_t)27158, PH.base.pack) ;
        p171_sue_pwm_output_11_SET((int16_t)(int16_t)4206, PH.base.pack) ;
        p171_sue_pwm_input_7_SET((int16_t)(int16_t) -26931, PH.base.pack) ;
        p171_sue_imu_location_y_SET((int16_t)(int16_t) -29509, PH.base.pack) ;
        p171_sue_waypoint_goal_z_SET((int16_t)(int16_t)16762, PH.base.pack) ;
        p171_sue_pwm_output_8_SET((int16_t)(int16_t)14949, PH.base.pack) ;
        p171_sue_pwm_input_3_SET((int16_t)(int16_t)28841, PH.base.pack) ;
        p171_sue_pwm_output_7_SET((int16_t)(int16_t)31266, PH.base.pack) ;
        p171_sue_waypoint_goal_y_SET((int16_t)(int16_t)8294, PH.base.pack) ;
        p171_sue_pwm_input_5_SET((int16_t)(int16_t) -16282, PH.base.pack) ;
        p171_sue_imu_velocity_z_SET((int16_t)(int16_t) -12036, PH.base.pack) ;
        p171_sue_pwm_output_12_SET((int16_t)(int16_t) -11958, PH.base.pack) ;
        p171_sue_pwm_input_10_SET((int16_t)(int16_t)783, PH.base.pack) ;
        p171_sue_pwm_output_10_SET((int16_t)(int16_t)14673, PH.base.pack) ;
        p171_sue_pwm_input_11_SET((int16_t)(int16_t)3778, PH.base.pack) ;
        p171_sue_barom_alt_SET((int32_t) -2094213193, PH.base.pack) ;
        p171_sue_osc_fails_SET((int16_t)(int16_t)7325, PH.base.pack) ;
        p171_sue_imu_location_z_SET((int16_t)(int16_t) -23824, PH.base.pack) ;
        p171_sue_pwm_input_8_SET((int16_t)(int16_t) -1628, PH.base.pack) ;
        p171_sue_imu_velocity_x_SET((int16_t)(int16_t)25954, PH.base.pack) ;
        p171_sue_pwm_input_6_SET((int16_t)(int16_t)6153, PH.base.pack) ;
        p171_sue_pwm_output_9_SET((int16_t)(int16_t)30550, PH.base.pack) ;
        p171_sue_flags_SET((uint32_t)1118076292L, PH.base.pack) ;
        p171_sue_location_error_earth_z_SET((int16_t)(int16_t)5281, PH.base.pack) ;
        p171_sue_pwm_output_2_SET((int16_t)(int16_t) -23976, PH.base.pack) ;
        p171_sue_barom_temp_SET((int16_t)(int16_t) -31116, PH.base.pack) ;
        p171_sue_aero_x_SET((int16_t)(int16_t) -30526, PH.base.pack) ;
        p171_sue_bat_amp_SET((int16_t)(int16_t) -2457, PH.base.pack) ;
        p171_sue_pwm_output_3_SET((int16_t)(int16_t)2344, PH.base.pack) ;
        p171_sue_pwm_input_1_SET((int16_t)(int16_t) -22547, PH.base.pack) ;
        p171_sue_memory_stack_free_SET((int16_t)(int16_t) -28170, PH.base.pack) ;
        p171_sue_aero_y_SET((int16_t)(int16_t) -2, PH.base.pack) ;
        p171_sue_pwm_output_6_SET((int16_t)(int16_t) -31976, PH.base.pack) ;
        p171_sue_pwm_input_4_SET((int16_t)(int16_t)27318, PH.base.pack) ;
        p171_sue_barom_press_SET((int32_t) -1852177511, PH.base.pack) ;
        p171_sue_pwm_input_9_SET((int16_t)(int16_t)15402, PH.base.pack) ;
        p171_sue_bat_volt_SET((int16_t)(int16_t)24091, PH.base.pack) ;
        p171_sue_waypoint_goal_x_SET((int16_t)(int16_t) -30927, PH.base.pack) ;
        p171_sue_pwm_output_4_SET((int16_t)(int16_t) -7348, PH.base.pack) ;
        p171_sue_time_SET((uint32_t)3265850006L, PH.base.pack) ;
        p171_sue_imu_location_x_SET((int16_t)(int16_t)20688, PH.base.pack) ;
        p171_sue_location_error_earth_x_SET((int16_t)(int16_t)21210, PH.base.pack) ;
        p171_sue_imu_velocity_y_SET((int16_t)(int16_t)32071, PH.base.pack) ;
        p171_sue_pwm_output_5_SET((int16_t)(int16_t)21671, PH.base.pack) ;
        p171_sue_desired_height_SET((int16_t)(int16_t) -20291, PH.base.pack) ;
        p171_sue_bat_amp_hours_SET((int16_t)(int16_t)24133, PH.base.pack) ;
        p171_sue_pwm_input_2_SET((int16_t)(int16_t) -10383, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F4_172(), &PH);
        p172_sue_ROLL_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p172_sue_AILERON_NAVIGATION_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p172_sue_YAW_STABILIZATION_AILERON_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_WAYPOINT_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p172_sue_RUDDER_NAVIGATION_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p172_sue_YAW_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p172_sue_RACING_MODE_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_STABILIZED_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p172_sue_PITCH_STABILIZATION_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p172_sue_ROLL_STABILIZATION_AILERONS_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F5_173(), &PH);
        p173_sue_ROLLKD_SET((float)1.0593349E37F, PH.base.pack) ;
        p173_sue_YAWKP_AILERON_SET((float)4.579536E37F, PH.base.pack) ;
        p173_sue_ROLLKP_SET((float) -1.1622743E38F, PH.base.pack) ;
        p173_sue_YAWKD_AILERON_SET((float) -8.68622E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F6_174(), &PH);
        p174_sue_PITCHKD_SET((float)2.9710836E38F, PH.base.pack) ;
        p174_sue_ROLL_ELEV_MIX_SET((float)1.3124364E38F, PH.base.pack) ;
        p174_sue_RUDDER_ELEV_MIX_SET((float)9.226391E37F, PH.base.pack) ;
        p174_sue_PITCHGAIN_SET((float) -3.3416633E38F, PH.base.pack) ;
        p174_sue_ELEVATOR_BOOST_SET((float) -3.2297537E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F7_175(), &PH);
        p175_sue_RUDDER_BOOST_SET((float) -3.18434E37F, PH.base.pack) ;
        p175_sue_YAWKP_RUDDER_SET((float)8.816493E37F, PH.base.pack) ;
        p175_sue_YAWKD_RUDDER_SET((float)3.0743646E38F, PH.base.pack) ;
        p175_sue_ROLLKD_RUDDER_SET((float)1.874043E38F, PH.base.pack) ;
        p175_sue_ROLLKP_RUDDER_SET((float) -2.9256206E38F, PH.base.pack) ;
        p175_sue_RTL_PITCH_DOWN_SET((float)1.5029126E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F8_176(), &PH);
        p176_sue_ALT_HOLD_PITCH_HIGH_SET((float) -1.4148535E37F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_MAX_SET((float) -2.094649E38F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MAX_SET((float) -4.2112867E37F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MIN_SET((float)1.2345561E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_THROTTLE_MAX_SET((float)3.101237E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_THROTTLE_MIN_SET((float)2.400362E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_MIN_SET((float) -1.7382579E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F13_177(), &PH);
        p177_sue_lat_origin_SET((int32_t) -274288077, PH.base.pack) ;
        p177_sue_lon_origin_SET((int32_t)905789817, PH.base.pack) ;
        p177_sue_week_no_SET((int16_t)(int16_t)20260, PH.base.pack) ;
        p177_sue_alt_origin_SET((int32_t) -556089570, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F14_178(), &PH);
        p178_sue_TRAP_SOURCE_SET((uint32_t)1362836943L, PH.base.pack) ;
        p178_sue_osc_fail_count_SET((int16_t)(int16_t)3825, PH.base.pack) ;
        p178_sue_GPS_TYPE_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p178_sue_DR_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p178_sue_CLOCK_CONFIG_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p178_sue_WIND_ESTIMATION_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p178_sue_RCON_SET((int16_t)(int16_t)5121, PH.base.pack) ;
        p178_sue_TRAP_FLAGS_SET((int16_t)(int16_t) -31365, PH.base.pack) ;
        p178_sue_BOARD_TYPE_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p178_sue_FLIGHT_PLAN_TYPE_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p178_sue_AIRFRAME_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F15_179(), &PH);
        {
            uint8_t sue_ID_VEHICLE_MODEL_NAME[] =  {(uint8_t)194, (uint8_t)251, (uint8_t)23, (uint8_t)168, (uint8_t)246, (uint8_t)89, (uint8_t)33, (uint8_t)9, (uint8_t)195, (uint8_t)61, (uint8_t)242, (uint8_t)129, (uint8_t)19, (uint8_t)177, (uint8_t)64, (uint8_t)86, (uint8_t)232, (uint8_t)212, (uint8_t)105, (uint8_t)174, (uint8_t)195, (uint8_t)251, (uint8_t)12, (uint8_t)188, (uint8_t)154, (uint8_t)158, (uint8_t)19, (uint8_t)100, (uint8_t)38, (uint8_t)100, (uint8_t)127, (uint8_t)66, (uint8_t)180, (uint8_t)113, (uint8_t)112, (uint8_t)252, (uint8_t)57, (uint8_t)239, (uint8_t)28, (uint8_t)91};
            p179_sue_ID_VEHICLE_MODEL_NAME_SET(&sue_ID_VEHICLE_MODEL_NAME, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_VEHICLE_REGISTRATION[] =  {(uint8_t)242, (uint8_t)235, (uint8_t)72, (uint8_t)56, (uint8_t)140, (uint8_t)108, (uint8_t)225, (uint8_t)192, (uint8_t)163, (uint8_t)113, (uint8_t)234, (uint8_t)9, (uint8_t)23, (uint8_t)167, (uint8_t)129, (uint8_t)89, (uint8_t)126, (uint8_t)216, (uint8_t)125, (uint8_t)175};
            p179_sue_ID_VEHICLE_REGISTRATION_SET(&sue_ID_VEHICLE_REGISTRATION, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F16_180(), &PH);
        {
            uint8_t sue_ID_DIY_DRONES_URL[] =  {(uint8_t)157, (uint8_t)135, (uint8_t)164, (uint8_t)39, (uint8_t)9, (uint8_t)11, (uint8_t)75, (uint8_t)223, (uint8_t)163, (uint8_t)126, (uint8_t)177, (uint8_t)59, (uint8_t)73, (uint8_t)206, (uint8_t)183, (uint8_t)207, (uint8_t)226, (uint8_t)109, (uint8_t)201, (uint8_t)139, (uint8_t)196, (uint8_t)135, (uint8_t)218, (uint8_t)223, (uint8_t)12, (uint8_t)81, (uint8_t)75, (uint8_t)50, (uint8_t)180, (uint8_t)205, (uint8_t)137, (uint8_t)197, (uint8_t)129, (uint8_t)200, (uint8_t)106, (uint8_t)181, (uint8_t)24, (uint8_t)131, (uint8_t)209, (uint8_t)179, (uint8_t)47, (uint8_t)173, (uint8_t)159, (uint8_t)69, (uint8_t)156, (uint8_t)138, (uint8_t)155, (uint8_t)227, (uint8_t)119, (uint8_t)49, (uint8_t)119, (uint8_t)50, (uint8_t)214, (uint8_t)152, (uint8_t)16, (uint8_t)195, (uint8_t)245, (uint8_t)123, (uint8_t)155, (uint8_t)160, (uint8_t)11, (uint8_t)184, (uint8_t)8, (uint8_t)170, (uint8_t)75, (uint8_t)91, (uint8_t)169, (uint8_t)147, (uint8_t)20, (uint8_t)165};
            p180_sue_ID_DIY_DRONES_URL_SET(&sue_ID_DIY_DRONES_URL, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_LEAD_PILOT[] =  {(uint8_t)107, (uint8_t)41, (uint8_t)16, (uint8_t)70, (uint8_t)123, (uint8_t)189, (uint8_t)63, (uint8_t)181, (uint8_t)44, (uint8_t)248, (uint8_t)150, (uint8_t)247, (uint8_t)187, (uint8_t)182, (uint8_t)61, (uint8_t)244, (uint8_t)198, (uint8_t)2, (uint8_t)48, (uint8_t)236, (uint8_t)134, (uint8_t)164, (uint8_t)67, (uint8_t)122, (uint8_t)190, (uint8_t)57, (uint8_t)77, (uint8_t)47, (uint8_t)88, (uint8_t)57, (uint8_t)160, (uint8_t)169, (uint8_t)51, (uint8_t)169, (uint8_t)216, (uint8_t)229, (uint8_t)80, (uint8_t)109, (uint8_t)12, (uint8_t)49};
            p180_sue_ID_LEAD_PILOT_SET(&sue_ID_LEAD_PILOT, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDES_181(), &PH);
        p181_alt_extra_SET((int32_t) -2119122694, PH.base.pack) ;
        p181_time_boot_ms_SET((uint32_t)4047212822L, PH.base.pack) ;
        p181_alt_range_finder_SET((int32_t)14704953, PH.base.pack) ;
        p181_alt_optical_flow_SET((int32_t)736994397, PH.base.pack) ;
        p181_alt_imu_SET((int32_t) -1205856627, PH.base.pack) ;
        p181_alt_gps_SET((int32_t)1854661769, PH.base.pack) ;
        p181_alt_barometric_SET((int32_t) -28947551, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDES_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AIRSPEEDS_182(), &PH);
        p182_airspeed_ultrasonic_SET((int16_t)(int16_t)2076, PH.base.pack) ;
        p182_airspeed_hot_wire_SET((int16_t)(int16_t) -13934, PH.base.pack) ;
        p182_time_boot_ms_SET((uint32_t)2000606745L, PH.base.pack) ;
        p182_aoy_SET((int16_t)(int16_t)17904, PH.base.pack) ;
        p182_aoa_SET((int16_t)(int16_t)16535, PH.base.pack) ;
        p182_airspeed_pitot_SET((int16_t)(int16_t) -26665, PH.base.pack) ;
        p182_airspeed_imu_SET((int16_t)(int16_t) -21637, PH.base.pack) ;
        c_CommunicationChannel_on_AIRSPEEDS_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F17_183(), &PH);
        p183_sue_feed_forward_SET((float)3.2580728E38F, PH.base.pack) ;
        p183_sue_turn_rate_fbw_SET((float) -8.3958223E37F, PH.base.pack) ;
        p183_sue_turn_rate_nav_SET((float) -1.8538587E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F18_184(), &PH);
        p184_elevator_trim_normal_SET((float) -1.5195805E38F, PH.base.pack) ;
        p184_angle_of_attack_inverted_SET((float) -6.5883574E37F, PH.base.pack) ;
        p184_reference_speed_SET((float) -1.9352962E37F, PH.base.pack) ;
        p184_elevator_trim_inverted_SET((float)3.2226429E38F, PH.base.pack) ;
        p184_angle_of_attack_normal_SET((float) -4.413074E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F19_185(), &PH);
        p185_sue_elevator_reversed_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p185_sue_elevator_output_channel_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p185_sue_throttle_output_channel_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p185_sue_aileron_reversed_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p185_sue_aileron_output_channel_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p185_sue_throttle_reversed_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p185_sue_rudder_reversed_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p185_sue_rudder_output_channel_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F20_186(), &PH);
        p186_sue_trim_value_input_11_SET((int16_t)(int16_t)3020, PH.base.pack) ;
        p186_sue_trim_value_input_7_SET((int16_t)(int16_t) -17933, PH.base.pack) ;
        p186_sue_trim_value_input_5_SET((int16_t)(int16_t)19323, PH.base.pack) ;
        p186_sue_trim_value_input_10_SET((int16_t)(int16_t) -7419, PH.base.pack) ;
        p186_sue_number_of_inputs_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p186_sue_trim_value_input_3_SET((int16_t)(int16_t)13140, PH.base.pack) ;
        p186_sue_trim_value_input_4_SET((int16_t)(int16_t) -9655, PH.base.pack) ;
        p186_sue_trim_value_input_1_SET((int16_t)(int16_t)14448, PH.base.pack) ;
        p186_sue_trim_value_input_2_SET((int16_t)(int16_t) -10824, PH.base.pack) ;
        p186_sue_trim_value_input_6_SET((int16_t)(int16_t) -30991, PH.base.pack) ;
        p186_sue_trim_value_input_9_SET((int16_t)(int16_t)30536, PH.base.pack) ;
        p186_sue_trim_value_input_8_SET((int16_t)(int16_t) -14659, PH.base.pack) ;
        p186_sue_trim_value_input_12_SET((int16_t)(int16_t) -11015, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F21_187(), &PH);
        p187_sue_gyro_x_offset_SET((int16_t)(int16_t) -4058, PH.base.pack) ;
        p187_sue_accel_z_offset_SET((int16_t)(int16_t) -25422, PH.base.pack) ;
        p187_sue_accel_x_offset_SET((int16_t)(int16_t)7693, PH.base.pack) ;
        p187_sue_gyro_y_offset_SET((int16_t)(int16_t)29160, PH.base.pack) ;
        p187_sue_gyro_z_offset_SET((int16_t)(int16_t) -12587, PH.base.pack) ;
        p187_sue_accel_y_offset_SET((int16_t)(int16_t) -796, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F22_188(), &PH);
        p188_sue_gyro_z_at_calibration_SET((int16_t)(int16_t)5698, PH.base.pack) ;
        p188_sue_gyro_x_at_calibration_SET((int16_t)(int16_t)2202, PH.base.pack) ;
        p188_sue_accel_x_at_calibration_SET((int16_t)(int16_t) -9211, PH.base.pack) ;
        p188_sue_gyro_y_at_calibration_SET((int16_t)(int16_t) -20026, PH.base.pack) ;
        p188_sue_accel_y_at_calibration_SET((int16_t)(int16_t) -14418, PH.base.pack) ;
        p188_sue_accel_z_at_calibration_SET((int16_t)(int16_t) -15398, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_hagl_ratio_SET((float) -6.653195E37F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.6199287E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)3.3661884E38F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL), PH.base.pack) ;
        p230_mag_ratio_SET((float) -7.037302E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.8286976E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)6248845200717246395L, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.9698682E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.6870002E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.9117432E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_vert_accuracy_SET((float) -5.8000973E37F, PH.base.pack) ;
        p231_var_vert_SET((float) -2.391233E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.752714E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.2053025E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -2.6074413E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)9147695598010197187L, PH.base.pack) ;
        p231_wind_alt_SET((float) -2.4341015E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)3.2857788E38F, PH.base.pack) ;
        p231_wind_y_SET((float)1.3419783E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_time_week_ms_SET((uint32_t)1412210928L, PH.base.pack) ;
        p232_lat_SET((int32_t)1486587317, PH.base.pack) ;
        p232_hdop_SET((float) -1.958829E38F, PH.base.pack) ;
        p232_alt_SET((float)4.556866E37F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p232_vn_SET((float) -2.8135014E38F, PH.base.pack) ;
        p232_vdop_SET((float)3.0729035E37F, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY), PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p232_vd_SET((float) -1.7292868E38F, PH.base.pack) ;
        p232_lon_SET((int32_t)1752001633, PH.base.pack) ;
        p232_vert_accuracy_SET((float)8.426687E37F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)8737193965070143742L, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)1.1791403E37F, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -1.5101175E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)29808, PH.base.pack) ;
        p232_ve_SET((float) -1.1583307E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)183, (uint8_t)192, (uint8_t)161, (uint8_t)81, (uint8_t)91, (uint8_t)51, (uint8_t)169, (uint8_t)94, (uint8_t)31, (uint8_t)82, (uint8_t)200, (uint8_t)112, (uint8_t)250, (uint8_t)104, (uint8_t)142, (uint8_t)231, (uint8_t)202, (uint8_t)22, (uint8_t)69, (uint8_t)222, (uint8_t)129, (uint8_t)190, (uint8_t)140, (uint8_t)180, (uint8_t)204, (uint8_t)101, (uint8_t)149, (uint8_t)219, (uint8_t)179, (uint8_t)56, (uint8_t)209, (uint8_t)241, (uint8_t)109, (uint8_t)207, (uint8_t)34, (uint8_t)2, (uint8_t)183, (uint8_t)97, (uint8_t)110, (uint8_t)150, (uint8_t)25, (uint8_t)232, (uint8_t)151, (uint8_t)204, (uint8_t)233, (uint8_t)242, (uint8_t)17, (uint8_t)215, (uint8_t)236, (uint8_t)38, (uint8_t)100, (uint8_t)103, (uint8_t)224, (uint8_t)178, (uint8_t)50, (uint8_t)84, (uint8_t)166, (uint8_t)116, (uint8_t)234, (uint8_t)54, (uint8_t)157, (uint8_t)198, (uint8_t)237, (uint8_t)53, (uint8_t)188, (uint8_t)223, (uint8_t)251, (uint8_t)152, (uint8_t)143, (uint8_t)211, (uint8_t)124, (uint8_t)22, (uint8_t)7, (uint8_t)251, (uint8_t)223, (uint8_t)215, (uint8_t)86, (uint8_t)114, (uint8_t)66, (uint8_t)164, (uint8_t)150, (uint8_t)221, (uint8_t)253, (uint8_t)124, (uint8_t)35, (uint8_t)143, (uint8_t)192, (uint8_t)61, (uint8_t)179, (uint8_t)149, (uint8_t)63, (uint8_t)185, (uint8_t)148, (uint8_t)249, (uint8_t)16, (uint8_t)174, (uint8_t)33, (uint8_t)69, (uint8_t)168, (uint8_t)77, (uint8_t)252, (uint8_t)185, (uint8_t)228, (uint8_t)139, (uint8_t)186, (uint8_t)7, (uint8_t)25, (uint8_t)187, (uint8_t)152, (uint8_t)214, (uint8_t)195, (uint8_t)180, (uint8_t)232, (uint8_t)29, (uint8_t)89, (uint8_t)10, (uint8_t)113, (uint8_t)166, (uint8_t)241, (uint8_t)204, (uint8_t)199, (uint8_t)239, (uint8_t)99, (uint8_t)62, (uint8_t)148, (uint8_t)27, (uint8_t)2, (uint8_t)187, (uint8_t)206, (uint8_t)51, (uint8_t)105, (uint8_t)199, (uint8_t)236, (uint8_t)30, (uint8_t)0, (uint8_t)105, (uint8_t)161, (uint8_t)13, (uint8_t)251, (uint8_t)119, (uint8_t)109, (uint8_t)254, (uint8_t)103, (uint8_t)145, (uint8_t)45, (uint8_t)83, (uint8_t)154, (uint8_t)101, (uint8_t)211, (uint8_t)193, (uint8_t)245, (uint8_t)66, (uint8_t)119, (uint8_t)135, (uint8_t)73, (uint8_t)98, (uint8_t)5, (uint8_t)49, (uint8_t)119, (uint8_t)236, (uint8_t)74, (uint8_t)43, (uint8_t)140, (uint8_t)33, (uint8_t)91, (uint8_t)32, (uint8_t)87, (uint8_t)134, (uint8_t)3, (uint8_t)178, (uint8_t)193, (uint8_t)98, (uint8_t)221, (uint8_t)150, (uint8_t)220, (uint8_t)72, (uint8_t)135, (uint8_t)29, (uint8_t)238, (uint8_t)42};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
        p234_airspeed_sp_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1013337221L, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -32515, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)70, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)55, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)10749, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)20913, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)32601, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)22, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)2509, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -16040, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p234_longitude_SET((int32_t)18936226, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p234_latitude_SET((int32_t)2065493425, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)7, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)44682, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
        p241_clipping_0_SET((uint32_t)2325159602L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.8669227E37F, PH.base.pack) ;
        p241_vibration_z_SET((float)2.585756E37F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)684067955L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6870660482108680559L, PH.base.pack) ;
        p241_vibration_y_SET((float) -2.9226975E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1437993313L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
        p242_y_SET((float)1.0823807E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)1951866910, PH.base.pack) ;
        {
            float q[] =  {-3.0066474E38F, 3.256559E38F, -1.0524146E38F, -1.1424947E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_z_SET((float) -3.2795052E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)7393614016295478337L, &PH) ;
        p242_x_SET((float) -6.2000867E37F, PH.base.pack) ;
        p242_approach_y_SET((float)1.7058685E38F, PH.base.pack) ;
        p242_z_SET((float) -1.8634437E38F, PH.base.pack) ;
        p242_approach_x_SET((float)1.8458406E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -647985461, PH.base.pack) ;
        p242_longitude_SET((int32_t)2074321680, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_x_SET((float) -6.7831436E37F, PH.base.pack) ;
        p243_latitude_SET((int32_t)1449617055, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p243_approach_y_SET((float)1.2787696E38F, PH.base.pack) ;
        p243_z_SET((float)1.3558568E37F, PH.base.pack) ;
        p243_approach_x_SET((float) -5.7618923E37F, PH.base.pack) ;
        {
            float q[] =  {1.030865E38F, -8.626237E37F, 1.3287596E38F, 1.0957855E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_time_usec_SET((uint64_t)7386741990352710258L, &PH) ;
        p243_altitude_SET((int32_t)354314507, PH.base.pack) ;
        p243_approach_z_SET((float)4.861447E37F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -717784874, PH.base.pack) ;
        p243_y_SET((float)7.5168877E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -363911996, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)61708, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_ver_velocity_SET((int16_t)(int16_t) -15786, PH.base.pack) ;
        p246_lon_SET((int32_t)932303807, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)65151, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT, PH.base.pack) ;
        p246_altitude_SET((int32_t) -36135835, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        {
            char16_t* callsign = u"nMlFfzh";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_lat_SET((int32_t) -1141306723, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING), PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3642377243L, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)32437, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)12247, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_horizontal_minimum_delta_SET((float)2.9160507E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -2.5402337E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.0233439E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_id_SET((uint32_t)190520276L, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)1772, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)78, (uint8_t)193, (uint8_t)77, (uint8_t)111, (uint8_t)190, (uint8_t)46, (uint8_t)164, (uint8_t)254, (uint8_t)129, (uint8_t)69, (uint8_t)157, (uint8_t)172, (uint8_t)51, (uint8_t)176, (uint8_t)39, (uint8_t)219, (uint8_t)57, (uint8_t)6, (uint8_t)215, (uint8_t)141, (uint8_t)210, (uint8_t)158, (uint8_t)161, (uint8_t)83, (uint8_t)210, (uint8_t)49, (uint8_t)126, (uint8_t)93, (uint8_t)194, (uint8_t)66, (uint8_t)163, (uint8_t)156, (uint8_t)19, (uint8_t)18, (uint8_t)41, (uint8_t)54, (uint8_t)202, (uint8_t)103, (uint8_t)218, (uint8_t)167, (uint8_t)242, (uint8_t)4, (uint8_t)44, (uint8_t)161, (uint8_t)166, (uint8_t)93, (uint8_t)247, (uint8_t)254, (uint8_t)188, (uint8_t)224, (uint8_t)61, (uint8_t)149, (uint8_t)79, (uint8_t)70, (uint8_t)16, (uint8_t)130, (uint8_t)117, (uint8_t)39, (uint8_t)177, (uint8_t)18, (uint8_t)194, (uint8_t)104, (uint8_t)211, (uint8_t)127, (uint8_t)122, (uint8_t)83, (uint8_t)145, (uint8_t)218, (uint8_t)238, (uint8_t)11, (uint8_t)220, (uint8_t)24, (uint8_t)232, (uint8_t)150, (uint8_t)24, (uint8_t)108, (uint8_t)29, (uint8_t)100, (uint8_t)234, (uint8_t)85, (uint8_t)230, (uint8_t)181, (uint8_t)112, (uint8_t)2, (uint8_t)201, (uint8_t)28, (uint8_t)31, (uint8_t)70, (uint8_t)169, (uint8_t)186, (uint8_t)197, (uint8_t)233, (uint8_t)24, (uint8_t)251, (uint8_t)147, (uint8_t)23, (uint8_t)218, (uint8_t)200, (uint8_t)118, (uint8_t)246, (uint8_t)66, (uint8_t)27, (uint8_t)78, (uint8_t)44, (uint8_t)200, (uint8_t)136, (uint8_t)80, (uint8_t)164, (uint8_t)215, (uint8_t)180, (uint8_t)237, (uint8_t)224, (uint8_t)39, (uint8_t)23, (uint8_t)56, (uint8_t)220, (uint8_t)134, (uint8_t)252, (uint8_t)57, (uint8_t)55, (uint8_t)55, (uint8_t)189, (uint8_t)22, (uint8_t)55, (uint8_t)90, (uint8_t)108, (uint8_t)148, (uint8_t)232, (uint8_t)244, (uint8_t)199, (uint8_t)172, (uint8_t)102, (uint8_t)23, (uint8_t)20, (uint8_t)233, (uint8_t)213, (uint8_t)120, (uint8_t)123, (uint8_t)188, (uint8_t)70, (uint8_t)181, (uint8_t)87, (uint8_t)186, (uint8_t)104, (uint8_t)57, (uint8_t)20, (uint8_t)39, (uint8_t)129, (uint8_t)87, (uint8_t)96, (uint8_t)158, (uint8_t)168, (uint8_t)82, (uint8_t)233, (uint8_t)176, (uint8_t)21, (uint8_t)73, (uint8_t)14, (uint8_t)212, (uint8_t)63, (uint8_t)148, (uint8_t)59, (uint8_t)110, (uint8_t)209, (uint8_t)239, (uint8_t)150, (uint8_t)241, (uint8_t)21, (uint8_t)3, (uint8_t)29, (uint8_t)150, (uint8_t)218, (uint8_t)198, (uint8_t)105, (uint8_t)163, (uint8_t)51, (uint8_t)220, (uint8_t)59, (uint8_t)76, (uint8_t)107, (uint8_t)193, (uint8_t)185, (uint8_t)149, (uint8_t)25, (uint8_t)196, (uint8_t)235, (uint8_t)88, (uint8_t)158, (uint8_t)190, (uint8_t)217, (uint8_t)68, (uint8_t)92, (uint8_t)179, (uint8_t)65, (uint8_t)116, (uint8_t)166, (uint8_t)252, (uint8_t)79, (uint8_t)236, (uint8_t)213, (uint8_t)0, (uint8_t)58, (uint8_t)245, (uint8_t)21, (uint8_t)78, (uint8_t)189, (uint8_t)98, (uint8_t)36, (uint8_t)241, (uint8_t)248, (uint8_t)77, (uint8_t)70, (uint8_t)121, (uint8_t)27, (uint8_t)210, (uint8_t)77, (uint8_t)158, (uint8_t)63, (uint8_t)15, (uint8_t)174, (uint8_t)86, (uint8_t)68, (uint8_t)34, (uint8_t)112, (uint8_t)164, (uint8_t)80, (uint8_t)11, (uint8_t)194, (uint8_t)44, (uint8_t)92, (uint8_t)228, (uint8_t)96, (uint8_t)107, (uint8_t)137, (uint8_t)152, (uint8_t)23, (uint8_t)80, (uint8_t)221, (uint8_t)137, (uint8_t)36, (uint8_t)137, (uint8_t)240, (uint8_t)8, (uint8_t)104, (uint8_t)16, (uint8_t)226, (uint8_t)58, (uint8_t)94, (uint8_t)118};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -51, (int8_t) -76, (int8_t)7, (int8_t)116, (int8_t) -69, (int8_t) -114, (int8_t)125, (int8_t) -12, (int8_t) -100, (int8_t)10, (int8_t)20, (int8_t) -48, (int8_t) -45, (int8_t) -89, (int8_t) -116, (int8_t) -59, (int8_t) -30, (int8_t) -15, (int8_t) -126, (int8_t)114, (int8_t)107, (int8_t)42, (int8_t)76, (int8_t) -82, (int8_t) -104, (int8_t)113, (int8_t) -101, (int8_t)3, (int8_t) -78, (int8_t) -106, (int8_t)127, (int8_t) -32};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)14809, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float) -3.0378654E38F, PH.base.pack) ;
        {
            char16_t* name = u"gxzkihczv";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -1.0313853E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)3352360465841404862L, PH.base.pack) ;
        p250_x_SET((float)1.1586995E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)3143771925L, PH.base.pack) ;
        {
            char16_t* name = u"mauuluhlla";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -1.4263979E38F, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"wWxzt";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)3431075127L, PH.base.pack) ;
        p252_value_SET((int32_t)67229384, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
        {
            char16_t* text = u"cuwcozSesgAkkzXpfkbvdtbwbv";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p254_value_SET((float)8.188972E37F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)2944420054L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)6254431061429079679L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)248, (uint8_t)209, (uint8_t)34, (uint8_t)179, (uint8_t)140, (uint8_t)51, (uint8_t)76, (uint8_t)35, (uint8_t)3, (uint8_t)226, (uint8_t)42, (uint8_t)34, (uint8_t)29, (uint8_t)118, (uint8_t)47, (uint8_t)225, (uint8_t)202, (uint8_t)35, (uint8_t)216, (uint8_t)227, (uint8_t)127, (uint8_t)86, (uint8_t)230, (uint8_t)158, (uint8_t)153, (uint8_t)23, (uint8_t)23, (uint8_t)58, (uint8_t)236, (uint8_t)27, (uint8_t)122, (uint8_t)11};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)3149641927L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)4171507764L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"yhotauigcybpcdwhwuehjtPylgnu";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_firmware_version_SET((uint32_t)2551108048L, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)162, (uint8_t)17, (uint8_t)105, (uint8_t)182, (uint8_t)197, (uint8_t)126, (uint8_t)124, (uint8_t)127, (uint8_t)91, (uint8_t)41, (uint8_t)197, (uint8_t)237, (uint8_t)185, (uint8_t)137, (uint8_t)51, (uint8_t)157, (uint8_t)79, (uint8_t)93, (uint8_t)154, (uint8_t)115, (uint8_t)47, (uint8_t)49, (uint8_t)62, (uint8_t)121, (uint8_t)195, (uint8_t)134, (uint8_t)63, (uint8_t)25, (uint8_t)250, (uint8_t)116, (uint8_t)99, (uint8_t)24};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_time_boot_ms_SET((uint32_t)3159776246L, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)26448, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -3.6758394E37F, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -3.0767428E38F, PH.base.pack) ;
        p259_focal_length_SET((float) -2.4084051E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"EhqFmMzEjwspzeJqzdmcEt";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        {
            uint8_t model_name[] =  {(uint8_t)22, (uint8_t)98, (uint8_t)126, (uint8_t)148, (uint8_t)187, (uint8_t)168, (uint8_t)92, (uint8_t)143, (uint8_t)37, (uint8_t)255, (uint8_t)205, (uint8_t)33, (uint8_t)35, (uint8_t)107, (uint8_t)39, (uint8_t)165, (uint8_t)140, (uint8_t)137, (uint8_t)173, (uint8_t)154, (uint8_t)160, (uint8_t)42, (uint8_t)11, (uint8_t)120, (uint8_t)68, (uint8_t)124, (uint8_t)224, (uint8_t)193, (uint8_t)84, (uint8_t)222, (uint8_t)232, (uint8_t)140};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_resolution_v_SET((uint16_t)(uint16_t)33351, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE), PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)46709, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)1937968843L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_total_capacity_SET((float) -2.708585E38F, PH.base.pack) ;
        p261_used_capacity_SET((float)1.6512937E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1788017670L, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p261_read_speed_SET((float) -2.2926374E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.5798219E37F, PH.base.pack) ;
        p261_write_speed_SET((float) -2.7819704E38F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)429483433L, PH.base.pack) ;
        p262_image_interval_SET((float) -1.9878287E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)1011099220L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p262_available_capacity_SET((float)9.13089E37F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lon_SET((int32_t)1328366276, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -3, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2904395782L, PH.base.pack) ;
        p263_lat_SET((int32_t) -1649261467, PH.base.pack) ;
        p263_image_index_SET((int32_t)719444745, PH.base.pack) ;
        p263_alt_SET((int32_t) -1712015757, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)4874047863655874151L, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        {
            char16_t* file_url = u"hjy";
            p263_file_url_SET_(file_url, &PH) ;
        }
        {
            float q[] =  {2.853934E38F, 3.5989876E37F, 2.42668E38F, -1.3845336E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_relative_alt_SET((int32_t)603713508, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)5706306694792320464L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)1155851505789340459L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)9055510902869687740L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)1400505981L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float) -2.052244E38F, PH.base.pack) ;
        p265_pitch_SET((float)2.124808E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1579683753L, PH.base.pack) ;
        p265_roll_SET((float) -2.1851869E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_target_component_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)157, (uint8_t)182, (uint8_t)237, (uint8_t)190, (uint8_t)6, (uint8_t)203, (uint8_t)65, (uint8_t)226, (uint8_t)152, (uint8_t)70, (uint8_t)130, (uint8_t)144, (uint8_t)116, (uint8_t)228, (uint8_t)47, (uint8_t)62, (uint8_t)45, (uint8_t)234, (uint8_t)171, (uint8_t)125, (uint8_t)77, (uint8_t)144, (uint8_t)36, (uint8_t)130, (uint8_t)76, (uint8_t)159, (uint8_t)158, (uint8_t)97, (uint8_t)152, (uint8_t)4, (uint8_t)232, (uint8_t)216, (uint8_t)34, (uint8_t)153, (uint8_t)59, (uint8_t)18, (uint8_t)37, (uint8_t)240, (uint8_t)95, (uint8_t)162, (uint8_t)39, (uint8_t)116, (uint8_t)35, (uint8_t)129, (uint8_t)72, (uint8_t)15, (uint8_t)145, (uint8_t)96, (uint8_t)184, (uint8_t)77, (uint8_t)113, (uint8_t)37, (uint8_t)75, (uint8_t)114, (uint8_t)36, (uint8_t)157, (uint8_t)107, (uint8_t)179, (uint8_t)175, (uint8_t)5, (uint8_t)177, (uint8_t)170, (uint8_t)80, (uint8_t)10, (uint8_t)220, (uint8_t)54, (uint8_t)36, (uint8_t)32, (uint8_t)28, (uint8_t)218, (uint8_t)220, (uint8_t)91, (uint8_t)23, (uint8_t)19, (uint8_t)169, (uint8_t)192, (uint8_t)3, (uint8_t)216, (uint8_t)61, (uint8_t)98, (uint8_t)37, (uint8_t)65, (uint8_t)57, (uint8_t)201, (uint8_t)225, (uint8_t)76, (uint8_t)80, (uint8_t)174, (uint8_t)93, (uint8_t)162, (uint8_t)115, (uint8_t)80, (uint8_t)7, (uint8_t)117, (uint8_t)245, (uint8_t)214, (uint8_t)175, (uint8_t)197, (uint8_t)199, (uint8_t)196, (uint8_t)192, (uint8_t)230, (uint8_t)209, (uint8_t)203, (uint8_t)213, (uint8_t)216, (uint8_t)247, (uint8_t)140, (uint8_t)49, (uint8_t)84, (uint8_t)25, (uint8_t)45, (uint8_t)223, (uint8_t)30, (uint8_t)180, (uint8_t)181, (uint8_t)65, (uint8_t)41, (uint8_t)43, (uint8_t)231, (uint8_t)43, (uint8_t)35, (uint8_t)219, (uint8_t)120, (uint8_t)38, (uint8_t)118, (uint8_t)91, (uint8_t)250, (uint8_t)17, (uint8_t)103, (uint8_t)215, (uint8_t)209, (uint8_t)54, (uint8_t)150, (uint8_t)188, (uint8_t)115, (uint8_t)160, (uint8_t)68, (uint8_t)197, (uint8_t)6, (uint8_t)165, (uint8_t)185, (uint8_t)57, (uint8_t)29, (uint8_t)172, (uint8_t)226, (uint8_t)21, (uint8_t)27, (uint8_t)124, (uint8_t)249, (uint8_t)229, (uint8_t)221, (uint8_t)1, (uint8_t)212, (uint8_t)20, (uint8_t)47, (uint8_t)32, (uint8_t)101, (uint8_t)154, (uint8_t)28, (uint8_t)252, (uint8_t)148, (uint8_t)210, (uint8_t)172, (uint8_t)116, (uint8_t)146, (uint8_t)254, (uint8_t)208, (uint8_t)209, (uint8_t)198, (uint8_t)2, (uint8_t)185, (uint8_t)32, (uint8_t)85, (uint8_t)84, (uint8_t)156, (uint8_t)239, (uint8_t)122, (uint8_t)43, (uint8_t)141, (uint8_t)119, (uint8_t)39, (uint8_t)147, (uint8_t)203, (uint8_t)167, (uint8_t)129, (uint8_t)66, (uint8_t)85, (uint8_t)123, (uint8_t)58, (uint8_t)57, (uint8_t)217, (uint8_t)142, (uint8_t)106, (uint8_t)37, (uint8_t)217, (uint8_t)116, (uint8_t)48, (uint8_t)248, (uint8_t)45, (uint8_t)234, (uint8_t)133, (uint8_t)241, (uint8_t)8, (uint8_t)194, (uint8_t)152, (uint8_t)34, (uint8_t)71, (uint8_t)51, (uint8_t)133, (uint8_t)49, (uint8_t)246, (uint8_t)14, (uint8_t)134, (uint8_t)2, (uint8_t)71, (uint8_t)158, (uint8_t)211, (uint8_t)165, (uint8_t)176, (uint8_t)250, (uint8_t)118, (uint8_t)159, (uint8_t)20, (uint8_t)162, (uint8_t)140, (uint8_t)100, (uint8_t)224, (uint8_t)204, (uint8_t)108, (uint8_t)248, (uint8_t)128, (uint8_t)228, (uint8_t)70, (uint8_t)114, (uint8_t)248, (uint8_t)66, (uint8_t)25, (uint8_t)163, (uint8_t)137, (uint8_t)35, (uint8_t)37, (uint8_t)222, (uint8_t)72, (uint8_t)224, (uint8_t)82, (uint8_t)53, (uint8_t)195, (uint8_t)194};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_first_message_offset_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)42432, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)101, (uint8_t)255, (uint8_t)221, (uint8_t)14, (uint8_t)152, (uint8_t)210, (uint8_t)167, (uint8_t)218, (uint8_t)103, (uint8_t)218, (uint8_t)245, (uint8_t)27, (uint8_t)183, (uint8_t)84, (uint8_t)245, (uint8_t)136, (uint8_t)40, (uint8_t)66, (uint8_t)241, (uint8_t)98, (uint8_t)234, (uint8_t)206, (uint8_t)13, (uint8_t)102, (uint8_t)56, (uint8_t)85, (uint8_t)239, (uint8_t)148, (uint8_t)166, (uint8_t)194, (uint8_t)170, (uint8_t)168, (uint8_t)222, (uint8_t)163, (uint8_t)107, (uint8_t)213, (uint8_t)136, (uint8_t)232, (uint8_t)87, (uint8_t)113, (uint8_t)118, (uint8_t)193, (uint8_t)90, (uint8_t)21, (uint8_t)54, (uint8_t)75, (uint8_t)126, (uint8_t)145, (uint8_t)215, (uint8_t)247, (uint8_t)28, (uint8_t)240, (uint8_t)24, (uint8_t)12, (uint8_t)28, (uint8_t)21, (uint8_t)60, (uint8_t)231, (uint8_t)46, (uint8_t)161, (uint8_t)198, (uint8_t)110, (uint8_t)220, (uint8_t)44, (uint8_t)156, (uint8_t)109, (uint8_t)139, (uint8_t)232, (uint8_t)101, (uint8_t)126, (uint8_t)153, (uint8_t)124, (uint8_t)215, (uint8_t)194, (uint8_t)84, (uint8_t)236, (uint8_t)112, (uint8_t)159, (uint8_t)70, (uint8_t)191, (uint8_t)101, (uint8_t)95, (uint8_t)77, (uint8_t)59, (uint8_t)47, (uint8_t)141, (uint8_t)2, (uint8_t)75, (uint8_t)241, (uint8_t)249, (uint8_t)228, (uint8_t)22, (uint8_t)41, (uint8_t)133, (uint8_t)41, (uint8_t)231, (uint8_t)254, (uint8_t)240, (uint8_t)197, (uint8_t)217, (uint8_t)184, (uint8_t)220, (uint8_t)239, (uint8_t)201, (uint8_t)217, (uint8_t)87, (uint8_t)36, (uint8_t)190, (uint8_t)138, (uint8_t)99, (uint8_t)226, (uint8_t)83, (uint8_t)106, (uint8_t)251, (uint8_t)170, (uint8_t)41, (uint8_t)203, (uint8_t)188, (uint8_t)0, (uint8_t)216, (uint8_t)69, (uint8_t)34, (uint8_t)200, (uint8_t)246, (uint8_t)199, (uint8_t)0, (uint8_t)59, (uint8_t)26, (uint8_t)136, (uint8_t)190, (uint8_t)137, (uint8_t)82, (uint8_t)242, (uint8_t)2, (uint8_t)33, (uint8_t)52, (uint8_t)184, (uint8_t)254, (uint8_t)60, (uint8_t)178, (uint8_t)200, (uint8_t)108, (uint8_t)173, (uint8_t)46, (uint8_t)153, (uint8_t)24, (uint8_t)16, (uint8_t)208, (uint8_t)124, (uint8_t)104, (uint8_t)134, (uint8_t)122, (uint8_t)14, (uint8_t)233, (uint8_t)180, (uint8_t)34, (uint8_t)7, (uint8_t)100, (uint8_t)20, (uint8_t)111, (uint8_t)61, (uint8_t)86, (uint8_t)69, (uint8_t)147, (uint8_t)117, (uint8_t)224, (uint8_t)255, (uint8_t)107, (uint8_t)65, (uint8_t)123, (uint8_t)191, (uint8_t)20, (uint8_t)210, (uint8_t)77, (uint8_t)206, (uint8_t)180, (uint8_t)198, (uint8_t)171, (uint8_t)202, (uint8_t)144, (uint8_t)170, (uint8_t)41, (uint8_t)158, (uint8_t)147, (uint8_t)111, (uint8_t)56, (uint8_t)60, (uint8_t)6, (uint8_t)171, (uint8_t)143, (uint8_t)125, (uint8_t)23, (uint8_t)172, (uint8_t)91, (uint8_t)69, (uint8_t)220, (uint8_t)129, (uint8_t)129, (uint8_t)208, (uint8_t)209, (uint8_t)92, (uint8_t)193, (uint8_t)150, (uint8_t)146, (uint8_t)109, (uint8_t)6, (uint8_t)235, (uint8_t)194, (uint8_t)248, (uint8_t)40, (uint8_t)152, (uint8_t)224, (uint8_t)109, (uint8_t)60, (uint8_t)49, (uint8_t)255, (uint8_t)155, (uint8_t)113, (uint8_t)193, (uint8_t)201, (uint8_t)192, (uint8_t)223, (uint8_t)214, (uint8_t)198, (uint8_t)199, (uint8_t)67, (uint8_t)68, (uint8_t)169, (uint8_t)125, (uint8_t)218, (uint8_t)87, (uint8_t)150, (uint8_t)55, (uint8_t)212, (uint8_t)197, (uint8_t)5, (uint8_t)133, (uint8_t)23, (uint8_t)201, (uint8_t)26, (uint8_t)174, (uint8_t)102, (uint8_t)166, (uint8_t)171, (uint8_t)218, (uint8_t)5, (uint8_t)61, (uint8_t)246, (uint8_t)236};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_first_message_offset_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)20679, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)49947, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_resolution_v_SET((uint16_t)(uint16_t)62315, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)52681, PH.base.pack) ;
        p269_framerate_SET((float) -1.7971075E38F, PH.base.pack) ;
        {
            char16_t* uri = u"lxifdbqyffjwdfoqgfqhkeribpkrdmhYjtZuvrgefspsovxxwidjtmpjeayqqwgglgqnwocdvllKkldvCjeoLUxpzcfluggyLfiGurcjcwtjfylxshacbjtxqlsFxdxwzyoVvgtfwvyynnntimjDjfldfgclgjgjyRpebifbxyjFnuxsmslkgkgxnhbksjkiraiSt";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_bitrate_SET((uint32_t)3314246950L, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)65470, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_rotation_SET((uint16_t)(uint16_t)22769, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)8495, PH.base.pack) ;
        p270_framerate_SET((float) -3.223009E38F, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)3853658581L, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        {
            char16_t* uri = u"xkxlzrmcggxMqOiszzjopnizmjxgghocTtpdhuGAfsmldgwgkbaxXcxAsjzNhaitjGhwqhrheSwannsnzmujdqHslevbqpecuHaipupklPzYgqlndfsoynlwavckkfgqbkWfczSyqjvmclrdrrjmtjdtXzxkozqxmpfpbxevhUpcmfpwzdebIcgmbCClqumLpnzjTY";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_resolution_h_SET((uint16_t)(uint16_t)19158, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"gluDnsfZclnmrlzvygmwlfiey";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"xnthzjwowouzlAhbcQQ";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_version_SET((uint16_t)(uint16_t)45523, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)82, (uint8_t)113, (uint8_t)69, (uint8_t)226, (uint8_t)48, (uint8_t)29, (uint8_t)229, (uint8_t)176};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)4406, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)115, (uint8_t)201, (uint8_t)125, (uint8_t)49, (uint8_t)164, (uint8_t)192, (uint8_t)177, (uint8_t)161};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)35946, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)41776, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)5477902761537677911L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1127211702L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_uptime_sec_SET((uint32_t)2758417286L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)53, (uint8_t)222, (uint8_t)49, (uint8_t)154, (uint8_t)9, (uint8_t)136, (uint8_t)68, (uint8_t)50, (uint8_t)13, (uint8_t)26, (uint8_t)0, (uint8_t)233, (uint8_t)233, (uint8_t)23, (uint8_t)110, (uint8_t)145};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        {
            char16_t* name = u"dgqypidhizxvyfuQPHOufbqLcYBekanfgadtbzKsmehjrffjdjirjarhkwcor";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_vcs_commit_SET((uint32_t)216760096L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)267766422710197405L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        {
            char16_t* param_id = u"guspqdjaynjOIBJ";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t)12149, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"bu";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)17263, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
        {
            char16_t* param_value = u"zavxjjyejeNe";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)11556, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        {
            char16_t* param_id = u"pzlpjc";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        {
            char16_t* param_value = u"ixjjhebxederddsdlpldflssgcmtupitikugJyxplbnejdpoMgpGHl";
            p323_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"Owdngewsuahtxqxx";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"xoesfaKwrqsrkziidrdsqgTmwmddcxfFmrOUouvdyezkcsLvczqfyqxphfhxfseZjksmyG";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        {
            uint16_t distances[] =  {(uint16_t)46799, (uint16_t)48988, (uint16_t)57200, (uint16_t)9517, (uint16_t)43415, (uint16_t)58942, (uint16_t)58091, (uint16_t)12285, (uint16_t)10847, (uint16_t)58247, (uint16_t)34047, (uint16_t)50637, (uint16_t)42595, (uint16_t)15006, (uint16_t)10625, (uint16_t)12483, (uint16_t)35611, (uint16_t)40532, (uint16_t)53142, (uint16_t)64896, (uint16_t)59866, (uint16_t)5036, (uint16_t)18916, (uint16_t)39510, (uint16_t)25125, (uint16_t)46870, (uint16_t)25779, (uint16_t)44303, (uint16_t)57591, (uint16_t)34736, (uint16_t)9684, (uint16_t)33964, (uint16_t)8594, (uint16_t)51822, (uint16_t)6371, (uint16_t)16522, (uint16_t)59753, (uint16_t)44564, (uint16_t)27684, (uint16_t)51687, (uint16_t)2440, (uint16_t)15797, (uint16_t)43600, (uint16_t)31748, (uint16_t)47251, (uint16_t)8985, (uint16_t)35960, (uint16_t)20472, (uint16_t)9980, (uint16_t)21554, (uint16_t)44324, (uint16_t)13878, (uint16_t)61611, (uint16_t)53054, (uint16_t)39596, (uint16_t)10170, (uint16_t)10871, (uint16_t)53111, (uint16_t)22135, (uint16_t)35605, (uint16_t)40774, (uint16_t)52109, (uint16_t)21740, (uint16_t)53319, (uint16_t)8913, (uint16_t)64909, (uint16_t)21890, (uint16_t)51741, (uint16_t)1752, (uint16_t)37041, (uint16_t)53690, (uint16_t)54269};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_min_distance_SET((uint16_t)(uint16_t)8920, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)8484876117719229488L, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)49068, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}



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
*	 (packets that were corrupted on reception on the MAV*/
INLINER uint16_t p1_drop_rate_comm_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  4, 2)));
}
/**
*Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted
*	 on reception on the MAV*/
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
*	 present. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_present_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 152, 26);
}
/**
*Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
*	 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_enabled_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  1 + (int)get_bits(data, 178, 26);
}
/**
*Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
*	 enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
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
*	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	 bit 11: yaw, bit 12: yaw rat*/
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
*	 =*/
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
*	 the system id of the requesting syste*/
INLINER uint8_t p4_target_system_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  12, 1)));
}
/**
*0: request ping from all receiving components, if greater than 0: message is a ping response and number
*	 is the system id of the requesting syste*/
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
*	 the safest mode possible initially and then gradually move down the encryption level if it gets a NACK
*	 message indicating an encryption mismatch*/
INLINER uint8_t p5_version_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  2, 1)));
}
/**
*Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The
*	 characters may involve A-Z, a-z, 0-9, and "!?,.-*/
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
*	 contro*/
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
    return  _en__W(get_bits(data, 40, 4));
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
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
*	 unknown, set to: UINT16_MA*/
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
*	 the AMSL altitude in addition to the WGS84 altitude*/
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
*	 provide the AMSL as well*/
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
*	 8 servos*/
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
*	 8 servos*/
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
*	 more than 8 servos*/
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
*	 send -2 to disable any existing map for this rc_channel_index*/
INLINER int16_t p50_param_index_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  2, 2)));
}
/**
*Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob
*	 on the RC*/
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
*	 on implementation*/
INLINER float p50_param_value_min_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  13, 4)));
}
/**
*Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends
*	 on implementation*/
INLINER float p50_param_value_max_GET(Pack * src)
{
    uint8_t * data = src->data;
    return (intBitsToFloat(get_bytes(data,  17, 4)));
}
/**
*Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT
*	 null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
*	 storage if the ID is stored as strin*/
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
*	 with Z axis up or local, right handed, Z axis down*/
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
*	 with Z axis up or local, right handed, Z axis down*/
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
*	 the second row, etc.*/
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
*	 the second row, etc.*/

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
*	 are available but not given in this message. This value should be 0 when no RC channels are available*/
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
*	 bit corresponds to Button 1*/
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
*	 Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle*/
INLINER int16_t p69_x_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  3, 2)));
}
/**
*Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	 Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle*/
INLINER int16_t p69_y_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  5, 2)));
}
/**
*Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	 Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on
*	 a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
*	 thrust*/
INLINER int16_t p69_z_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((int16_t)(get_bytes(data,  7, 2)));
}
/**
*R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
*	 Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise
*	 being -1000, and the yaw of a vehicle*/
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
*	 sequence (0,1,2,3,4)*/
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
*	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitud*/
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
*	 bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitud*/
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
*	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	 bit 11: yaw, bit 12: yaw rat*/
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
*	 =*/
INLINER e_MAV_FRAME p84_coordinate_frame_GET(Pack * src)
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 416, 4);
}
/**
*Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or
*	 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set
*	 the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit
*	 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint,
*	 bit 11: yaw, bit 12: yaw rat*/
INLINER uint16_t p86_type_mask_GET(Pack * src)
{
    uint8_t * data = src->data;
    return ((get_bytes(data,  0, 2)));
}
/**
*Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow
*	 the system to compensate for the transport delay of the setpoint. This allows the system to compensate
*	 processing latency*/
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
*	 = 1*/
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
    set_bits(- 0 +   src, 3, data, 132);
}
Pack * c_TEST_Channel_new_COLLISION_247()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 247));
};/**
*A code that identifies the software component that understands this message (analogous to usb device classes
*	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
*	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
*	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
*	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
*	 widely distributed codebase*/
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
*	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
*	 message_type.  The particular encoding used can be extension specific and might not always be documented
*	 as part of the mavlink specification*/
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
    set_bits(- 0 +   src, 3, data, 32);
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
*	 set and capture in progress*/
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
*	 lost (set to 255 if no start exists)*/
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
*	 lost (set to 255 if no start exists)*/
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
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
INLINER void p320_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 32 && insert_field(dst, 32, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
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
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
INLINER void p322_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 38 && insert_field(dst, 38, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
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
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
INLINER void p323_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 22 && insert_field(dst, 22, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
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
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
INLINER void p324_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 8 && insert_field(dst, 8, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	 (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	 ID is stored as strin*/
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
*	 is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX
*	 for unknown/not used. In a array element, each unit corresponds to 1cm*/
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4);
    assert(p0_custom_mode_GET(pack) == (uint32_t)3408125283L);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_BOOT);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)39716);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)53221);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)6787);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL));
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)24211);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)34449);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)2311);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)60399);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)17747);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)2312);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS));
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -85);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2210121159848586267L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2939771029L);
};


void c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_z_GET(pack) == (float) -1.2099859E38F);
    assert(p3_yaw_rate_GET(pack) == (float)3.740571E37F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)3686);
    assert(p3_x_GET(pack) == (float) -3.0771308E38F);
    assert(p3_afx_GET(pack) == (float) -2.5023027E38F);
    assert(p3_y_GET(pack) == (float)1.1768226E38F);
    assert(p3_yaw_GET(pack) == (float)1.0427875E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p3_afy_GET(pack) == (float) -2.3183098E37F);
    assert(p3_vx_GET(pack) == (float)3.4970748E37F);
    assert(p3_afz_GET(pack) == (float) -2.5463553E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)310102113L);
    assert(p3_vy_GET(pack) == (float) -2.9346152E38F);
    assert(p3_vz_GET(pack) == (float) -1.0083937E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)8520959309834784821L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p4_seq_GET(pack) == (uint32_t)491620138L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p5_passkey_LEN(ph) == 3);
    {
        char16_t * exemplary = u"ulm";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 28);
    {
        char16_t * exemplary = u"uxnmrvhekhsldxzhaiqfKafmuecg";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p11_custom_mode_GET(pack) == (uint32_t)3910606134L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p20_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"swmUrsYof";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)3301);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float)2.3896311E38F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)11646);
    assert(p22_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"hmuckxdkzozup";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)42167);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p23_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"piEff";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p23_param_value_GET(pack) == (float)2.6484462E38F);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_time_usec_GET(pack) == (uint64_t)1377389755253240027L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1699839210L);
    assert(p24_lat_GET(pack) == (int32_t) -283930369);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)593074329L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p24_lon_GET(pack) == (int32_t)913978713);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)23862);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)23586);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1596762773L);
    assert(p24_alt_GET(pack) == (int32_t)1621952165);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)62994);
    assert(p24_v_acc_TRY(ph) == (uint32_t)156293354L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)56520);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -99271133);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)53, (uint8_t)250, (uint8_t)93, (uint8_t)198, (uint8_t)253, (uint8_t)92, (uint8_t)175, (uint8_t)36, (uint8_t)116, (uint8_t)177, (uint8_t)63, (uint8_t)66, (uint8_t)169, (uint8_t)81, (uint8_t)153, (uint8_t)203, (uint8_t)238, (uint8_t)78, (uint8_t)73, (uint8_t)86} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)0, (uint8_t)110, (uint8_t)4, (uint8_t)201, (uint8_t)55, (uint8_t)45, (uint8_t)42, (uint8_t)92, (uint8_t)191, (uint8_t)167, (uint8_t)108, (uint8_t)55, (uint8_t)35, (uint8_t)204, (uint8_t)222, (uint8_t)209, (uint8_t)100, (uint8_t)46, (uint8_t)175, (uint8_t)132} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)67);
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)156, (uint8_t)126, (uint8_t)151, (uint8_t)215, (uint8_t)50, (uint8_t)137, (uint8_t)214, (uint8_t)36, (uint8_t)22, (uint8_t)36, (uint8_t)28, (uint8_t)110, (uint8_t)25, (uint8_t)135, (uint8_t)235, (uint8_t)233, (uint8_t)227, (uint8_t)211, (uint8_t)98} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)144, (uint8_t)148, (uint8_t)84, (uint8_t)207, (uint8_t)197, (uint8_t)155, (uint8_t)94, (uint8_t)195, (uint8_t)138, (uint8_t)94, (uint8_t)172, (uint8_t)64, (uint8_t)43, (uint8_t)217, (uint8_t)106, (uint8_t)57, (uint8_t)233, (uint8_t)125, (uint8_t)248, (uint8_t)143} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)238, (uint8_t)164, (uint8_t)11, (uint8_t)124, (uint8_t)29, (uint8_t)202, (uint8_t)34, (uint8_t)32, (uint8_t)30, (uint8_t)87, (uint8_t)11, (uint8_t)133, (uint8_t)58, (uint8_t)10, (uint8_t)203, (uint8_t)221, (uint8_t)25, (uint8_t)84, (uint8_t)133, (uint8_t)194} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)14915);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)8996);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -25304);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)23572);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -20722);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -15768);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3135617430L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -14703);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)21348);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)9253);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)8897);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)14034);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -3178);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)12875);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)28148);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)15890);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -32714);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)17389);
    assert(p27_time_usec_GET(pack) == (uint64_t)8785221584954152343L);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -16140);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)11795);
    assert(p28_time_usec_GET(pack) == (uint64_t)3044431427196297846L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -5351);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)16344);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)25140);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_diff_GET(pack) == (float)8.981826E37F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -5872);
    assert(p29_press_abs_GET(pack) == (float)2.7227048E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3809589523L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitch_GET(pack) == (float)7.725193E37F);
    assert(p30_roll_GET(pack) == (float)7.616329E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3888653946L);
    assert(p30_pitchspeed_GET(pack) == (float)2.9606886E38F);
    assert(p30_yawspeed_GET(pack) == (float)1.5972253E38F);
    assert(p30_yaw_GET(pack) == (float)7.42736E37F);
    assert(p30_rollspeed_GET(pack) == (float)4.3112584E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_rollspeed_GET(pack) == (float)4.2462834E36F);
    assert(p31_q2_GET(pack) == (float) -2.6135726E38F);
    assert(p31_yawspeed_GET(pack) == (float) -3.8435164E37F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1366300314L);
    assert(p31_pitchspeed_GET(pack) == (float)3.1715089E38F);
    assert(p31_q3_GET(pack) == (float) -4.7666776E37F);
    assert(p31_q4_GET(pack) == (float) -7.867445E37F);
    assert(p31_q1_GET(pack) == (float)1.1537495E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vy_GET(pack) == (float) -9.588899E37F);
    assert(p32_vx_GET(pack) == (float)1.8392336E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2741608465L);
    assert(p32_y_GET(pack) == (float) -1.6349509E38F);
    assert(p32_vz_GET(pack) == (float)2.287399E36F);
    assert(p32_x_GET(pack) == (float)2.9518984E38F);
    assert(p32_z_GET(pack) == (float)2.6041683E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_relative_alt_GET(pack) == (int32_t) -1200391901);
    assert(p33_alt_GET(pack) == (int32_t) -774662098);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)8372);
    assert(p33_lon_GET(pack) == (int32_t) -984172246);
    assert(p33_lat_GET(pack) == (int32_t)1214081414);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)2182682524L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)48278);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)15458);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -30655);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -22019);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -1114);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)4109088018L);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)10833);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)22209);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -736);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -15300);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)16637);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)32284);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)37051);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3647044073L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)25038);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)43011);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)27887);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)51854);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)8);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)4608);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)48541);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)52573);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)45495);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)37925);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)42737);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)19417);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)44043);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)47833);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)49220);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)44683);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)46010);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)56385);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p36_time_usec_GET(pack) == (uint32_t)1147717692L);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)52926);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)9305);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)30310);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)36856);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)35655);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)6916);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -3014);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)10278);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)17339);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_param1_GET(pack) == (float)3.3997101E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p39_param3_GET(pack) == (float) -2.6828186E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING);
    assert(p39_param2_GET(pack) == (float)5.8257997E37F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p39_param4_GET(pack) == (float) -9.842333E37F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)6414);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p39_y_GET(pack) == (float) -1.4387047E38F);
    assert(p39_x_GET(pack) == (float) -2.3491963E38F);
    assert(p39_z_GET(pack) == (float) -2.534054E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)24764);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)15350);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)30208);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)53744);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)43958);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)124);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_time_usec_TRY(ph) == (uint64_t)8012218089819038089L);
    assert(p48_latitude_GET(pack) == (int32_t) -2098047369);
    assert(p48_altitude_GET(pack) == (int32_t)1218214308);
    assert(p48_longitude_GET(pack) == (int32_t) -1027835251);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)4598053584464042712L);
    assert(p49_longitude_GET(pack) == (int32_t) -1022061667);
    assert(p49_latitude_GET(pack) == (int32_t) -1643106871);
    assert(p49_altitude_GET(pack) == (int32_t) -1132883023);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -5061);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p50_param_value0_GET(pack) == (float)1.4335475E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p50_scale_GET(pack) == (float)6.727706E35F);
    assert(p50_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"oofdlvjchykpr";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_min_GET(pack) == (float)2.787156E38F);
    assert(p50_param_value_max_GET(pack) == (float) -2.1994807E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)38600);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p54_p2y_GET(pack) == (float) -1.5402268E38F);
    assert(p54_p1y_GET(pack) == (float)4.535394E37F);
    assert(p54_p2z_GET(pack) == (float) -1.631655E38F);
    assert(p54_p2x_GET(pack) == (float)2.066187E38F);
    assert(p54_p1z_GET(pack) == (float)5.6971996E37F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p54_p1x_GET(pack) == (float) -1.3884734E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)28);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p55_p2y_GET(pack) == (float)2.3703176E38F);
    assert(p55_p2z_GET(pack) == (float)2.6287159E38F);
    assert(p55_p2x_GET(pack) == (float)2.5565703E38F);
    assert(p55_p1y_GET(pack) == (float) -2.860838E38F);
    assert(p55_p1z_GET(pack) == (float)2.3220122E38F);
    assert(p55_p1x_GET(pack) == (float) -1.9920058E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_time_usec_GET(pack) == (uint64_t)1476416739481309040L);
    {
        float exemplary[] =  {1.8776978E37F, 1.2753453E38F, 5.312655E36F, -2.428011E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -7.196586E36F);
    assert(p61_pitchspeed_GET(pack) == (float)2.3827907E38F);
    assert(p61_yawspeed_GET(pack) == (float) -7.383447E37F);
    {
        float exemplary[] =  {-3.9966747E37F, 2.3834547E38F, 2.9093802E38F, -1.2407778E38F, -6.1389296E37F, 2.7123322E38F, 7.8381504E37F, -4.8769E36F, 2.4767532E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)26866);
    assert(p62_alt_error_GET(pack) == (float) -2.2710664E38F);
    assert(p62_xtrack_error_GET(pack) == (float)2.2246706E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)22909);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)25068);
    assert(p62_nav_pitch_GET(pack) == (float) -8.714565E37F);
    assert(p62_aspd_error_GET(pack) == (float)1.3008247E38F);
    assert(p62_nav_roll_GET(pack) == (float) -1.6832221E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lat_GET(pack) == (int32_t) -937771913);
    assert(p63_vx_GET(pack) == (float)7.5062353E37F);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
    assert(p63_vz_GET(pack) == (float)2.2299232E38F);
    assert(p63_vy_GET(pack) == (float)3.8559538E37F);
    assert(p63_relative_alt_GET(pack) == (int32_t)699567222);
    assert(p63_alt_GET(pack) == (int32_t) -1550055601);
    assert(p63_time_usec_GET(pack) == (uint64_t)3885578181752161293L);
    assert(p63_lon_GET(pack) == (int32_t)1751785758);
    {
        float exemplary[] =  {9.324785E37F, 2.6157185E38F, -7.0132247E37F, -4.7117356E37F, -9.877655E37F, -2.5082923E38F, 2.3969099E38F, 1.7351109E38F, -3.059852E38F, -1.6860039E38F, -5.168891E37F, 2.8911853E38F, 1.3700216E38F, 2.7587935E38F, -2.8177147E38F, -9.628201E37F, 1.1249308E37F, 3.0308278E37F, 3.310603E38F, -2.2104959E38F, 1.092786E38F, 2.389361E38F, -1.8456376E38F, -5.515554E37F, 2.8615659E38F, -9.113652E37F, 2.3112864E38F, -2.5115604E38F, 1.6472251E38F, 4.1204843E37F, 1.6704361E38F, 1.6287173E38F, -8.255146E37F, -1.9028194E38F, 2.3393182E38F, 4.1643453E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vz_GET(pack) == (float) -2.9015812E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)7122628453472856922L);
    assert(p64_ay_GET(pack) == (float)1.6585729E37F);
    assert(p64_vy_GET(pack) == (float)2.4942176E38F);
    assert(p64_z_GET(pack) == (float) -1.7959096E38F);
    {
        float exemplary[] =  {-7.248239E37F, 1.9247346E38F, 7.2695766E37F, 6.4051875E37F, 2.421203E38F, 2.0794496E38F, -3.1653463E38F, 2.9187935E38F, -4.772443E37F, 4.4942777E37F, -6.6966285E37F, -1.9004153E38F, 2.6027473E38F, 2.897851E38F, 3.0796484E38F, -2.8835047E38F, 5.0073776E37F, -2.911965E38F, 9.321932E37F, 1.3026383E38F, 1.1992775E37F, 2.5530343E38F, 2.7850799E38F, -1.2500509E38F, 1.4858595E38F, 3.2688362E38F, -1.3239683E38F, 6.4079723E37F, -2.6356097E38F, 1.7542402E38F, -3.1600213E38F, 2.9776768E37F, 8.2389703E37F, 3.3286375E38F, 1.1505628E38F, 6.162478E37F, 1.9231385E38F, -3.300018E38F, 1.1858987E38F, 2.7304259E38F, 1.3303357E38F, -2.6037065E38F, 7.0148083E37F, -1.0175381E38F, 1.090745E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vx_GET(pack) == (float) -3.0817086E37F);
    assert(p64_x_GET(pack) == (float) -2.2921423E38F);
    assert(p64_ax_GET(pack) == (float) -1.3484916E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p64_y_GET(pack) == (float) -2.086658E38F);
    assert(p64_az_GET(pack) == (float)2.7949503E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)61897);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)20739);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)15162);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)12148);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)37565);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)43265);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)61618);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)42112);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)12282);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2075181232L);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)48040);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)27800);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)34219);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)15710);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)46297);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)26353);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)59087);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)17865);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)44509);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)35161);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)27904);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_y_GET(pack) == (int16_t)(int16_t)5271);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)61418);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)4705);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -28069);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)20183);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)12799);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)22455);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)31762);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)52566);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)28942);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)38945);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)25071);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)59382);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_z_GET(pack) == (float) -2.1508818E38F);
    assert(p73_y_GET(pack) == (int32_t) -691584949);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_GO_AROUND);
    assert(p73_param2_GET(pack) == (float)1.6653952E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)33146);
    assert(p73_param1_GET(pack) == (float)1.5238137E38F);
    assert(p73_param3_GET(pack) == (float) -1.1841184E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p73_x_GET(pack) == (int32_t)1309228102);
    assert(p73_param4_GET(pack) == (float)2.6643535E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float) -3.2403275E38F);
    assert(p74_alt_GET(pack) == (float) -1.1152988E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)11399);
    assert(p74_climb_GET(pack) == (float)3.2512216E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)35027);
    assert(p74_airspeed_GET(pack) == (float) -9.336258E37F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p75_x_GET(pack) == (int32_t)54124348);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p75_y_GET(pack) == (int32_t) -1780211533);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_LOGGING_START);
    assert(p75_param2_GET(pack) == (float)6.8993065E37F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p75_param1_GET(pack) == (float)3.832021E37F);
    assert(p75_z_GET(pack) == (float) -1.0211023E38F);
    assert(p75_param4_GET(pack) == (float) -9.780052E37F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p75_param3_GET(pack) == (float) -1.9511516E38F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p76_param1_GET(pack) == (float) -3.4920367E37F);
    assert(p76_param6_GET(pack) == (float)1.6724991E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p76_param2_GET(pack) == (float) -1.6360408E38F);
    assert(p76_param3_GET(pack) == (float)2.357917E38F);
    assert(p76_param5_GET(pack) == (float)1.6929635E38F);
    assert(p76_param7_GET(pack) == (float)2.7327888E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p76_param4_GET(pack) == (float)1.5896953E37F);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)180);
    assert(p77_result_param2_TRY(ph) == (int32_t) -969857554);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPOSITION);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)28);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)243);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_UNSUPPORTED);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_yaw_GET(pack) == (float) -2.5799333E38F);
    assert(p81_thrust_GET(pack) == (float) -1.0362808E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p81_pitch_GET(pack) == (float)1.0201292E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p81_roll_GET(pack) == (float)3.1842489E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)464306958L);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p82_thrust_GET(pack) == (float)6.4892165E37F);
    assert(p82_body_roll_rate_GET(pack) == (float)1.6625863E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float)2.384996E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.7373874E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3434475015L);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)105);
    {
        float exemplary[] =  {-1.3091588E38F, 1.1677438E38F, 3.2577065E38F, 3.272849E36F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float) -1.1061844E38F);
    {
        float exemplary[] =  {7.271168E37F, -7.240945E37F, 1.2080687E38F, -3.1068636E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_pitch_rate_GET(pack) == (float)9.710877E37F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p83_body_yaw_rate_GET(pack) == (float) -2.6974676E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)1.2746782E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)4214411413L);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float)2.5241232E38F);
    assert(p84_yaw_rate_GET(pack) == (float)8.0418375E37F);
    assert(p84_afz_GET(pack) == (float) -1.8517879E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2315897290L);
    assert(p84_vx_GET(pack) == (float)2.386658E37F);
    assert(p84_vz_GET(pack) == (float) -1.7253996E38F);
    assert(p84_z_GET(pack) == (float) -1.6515804E38F);
    assert(p84_y_GET(pack) == (float)4.403152E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p84_afx_GET(pack) == (float) -2.4772392E38F);
    assert(p84_vy_GET(pack) == (float)1.4258091E38F);
    assert(p84_x_GET(pack) == (float)2.6157173E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)40769);
    assert(p84_yaw_GET(pack) == (float)5.718891E37F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p86_afy_GET(pack) == (float)2.1797844E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3447054801L);
    assert(p86_yaw_rate_GET(pack) == (float) -3.3072787E38F);
    assert(p86_vy_GET(pack) == (float) -1.5734852E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p86_lat_int_GET(pack) == (int32_t) -59010296);
    assert(p86_lon_int_GET(pack) == (int32_t)551422805);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)10588);
    assert(p86_vz_GET(pack) == (float)1.1816582E38F);
    assert(p86_vx_GET(pack) == (float) -8.3626915E37F);
    assert(p86_yaw_GET(pack) == (float)1.4180437E36F);
    assert(p86_afx_GET(pack) == (float) -1.2065997E38F);
    assert(p86_alt_GET(pack) == (float)3.390394E37F);
    assert(p86_afz_GET(pack) == (float) -2.3566618E37F);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afy_GET(pack) == (float) -8.2943915E37F);
    assert(p87_vx_GET(pack) == (float)1.4241926E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.1998096E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -1805121297);
    assert(p87_lon_int_GET(pack) == (int32_t) -372721867);
    assert(p87_afx_GET(pack) == (float)2.2615974E38F);
    assert(p87_yaw_GET(pack) == (float)3.3450141E38F);
    assert(p87_vz_GET(pack) == (float)1.5820491E38F);
    assert(p87_vy_GET(pack) == (float)1.817087E38F);
    assert(p87_alt_GET(pack) == (float)3.4993475E37F);
    assert(p87_afz_GET(pack) == (float) -6.49064E37F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)4280176301L);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)3316);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float)5.6132446E37F);
    assert(p89_x_GET(pack) == (float)2.388116E37F);
    assert(p89_yaw_GET(pack) == (float)1.6509772E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)1031710358L);
    assert(p89_z_GET(pack) == (float)1.8513128E38F);
    assert(p89_roll_GET(pack) == (float) -2.3735129E38F);
    assert(p89_pitch_GET(pack) == (float) -1.5158474E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -13548);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -21624);
    assert(p90_pitch_GET(pack) == (float)2.199528E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -7781);
    assert(p90_yawspeed_GET(pack) == (float) -2.8195632E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -15188);
    assert(p90_lon_GET(pack) == (int32_t) -1638797615);
    assert(p90_alt_GET(pack) == (int32_t) -611291160);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -3403);
    assert(p90_rollspeed_GET(pack) == (float) -9.668737E37F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -18988);
    assert(p90_lat_GET(pack) == (int32_t) -458507348);
    assert(p90_time_usec_GET(pack) == (uint64_t)6621160391877624044L);
    assert(p90_yaw_GET(pack) == (float)3.3412317E38F);
    assert(p90_roll_GET(pack) == (float) -1.2982056E38F);
    assert(p90_pitchspeed_GET(pack) == (float) -5.2651274E37F);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float)1.056376E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p91_time_usec_GET(pack) == (uint64_t)6892959032905017374L);
    assert(p91_aux2_GET(pack) == (float)8.86115E37F);
    assert(p91_throttle_GET(pack) == (float) -3.2100757E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -8.4173607E37F);
    assert(p91_roll_ailerons_GET(pack) == (float)2.6149948E38F);
    assert(p91_aux4_GET(pack) == (float)3.363948E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)2.2188548E38F);
    assert(p91_aux3_GET(pack) == (float)2.0460816E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)51740);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)12491);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)54620);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)1716);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)63063);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)49673);
    assert(p92_time_usec_GET(pack) == (uint64_t)8870994278679958258L);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)53813);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)6323);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)26914);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)35951);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)32598);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)41862);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.9929374E38F, 1.5941744E38F, 2.0445494E38F, -2.2081379E36F, -2.90828E38F, -1.3222085E38F, 1.5573441E38F, -1.6553245E38F, -4.226708E36F, 3.1148777E38F, -6.164555E36F, 1.2376298E38F, -1.7526821E38F, 1.772962E38F, 2.7983949E38F, 1.1335693E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)5103971221524761329L);
    assert(p93_flags_GET(pack) == (uint64_t)9143718469304279823L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)27721);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.3740337E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -17528);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p100_flow_rate_x_TRY(ph) == (float) -3.1296571E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float) -1.274092E38F);
    assert(p100_ground_distance_GET(pack) == (float) -9.132041E36F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)6.966749E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)3041448896953357617L);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_z_GET(pack) == (float) -3.312887E38F);
    assert(p101_y_GET(pack) == (float)2.8735998E38F);
    assert(p101_x_GET(pack) == (float) -3.3322263E38F);
    assert(p101_yaw_GET(pack) == (float)7.110774E37F);
    assert(p101_usec_GET(pack) == (uint64_t)3431526208652439574L);
    assert(p101_pitch_GET(pack) == (float)1.9795944E38F);
    assert(p101_roll_GET(pack) == (float) -1.9811667E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_x_GET(pack) == (float) -2.9839505E38F);
    assert(p102_yaw_GET(pack) == (float) -1.6015866E38F);
    assert(p102_pitch_GET(pack) == (float)1.5387557E38F);
    assert(p102_roll_GET(pack) == (float)5.7036636E37F);
    assert(p102_y_GET(pack) == (float)2.0572022E37F);
    assert(p102_z_GET(pack) == (float)1.0745515E38F);
    assert(p102_usec_GET(pack) == (uint64_t)3333953873928107621L);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)4135466684440552753L);
    assert(p103_y_GET(pack) == (float) -2.3928775E38F);
    assert(p103_z_GET(pack) == (float) -3.269458E38F);
    assert(p103_x_GET(pack) == (float) -2.6107917E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_roll_GET(pack) == (float)3.370568E38F);
    assert(p104_pitch_GET(pack) == (float) -5.666146E36F);
    assert(p104_y_GET(pack) == (float) -2.9684824E38F);
    assert(p104_yaw_GET(pack) == (float) -2.4165818E38F);
    assert(p104_x_GET(pack) == (float)1.187243E38F);
    assert(p104_z_GET(pack) == (float) -8.4727454E37F);
    assert(p104_usec_GET(pack) == (uint64_t)9147514444017557728L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_zgyro_GET(pack) == (float) -8.49169E37F);
    assert(p105_xacc_GET(pack) == (float) -2.2828886E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8785828000405431647L);
    assert(p105_zmag_GET(pack) == (float)3.0339661E38F);
    assert(p105_ymag_GET(pack) == (float)6.002739E37F);
    assert(p105_xmag_GET(pack) == (float)2.4954692E38F);
    assert(p105_abs_pressure_GET(pack) == (float)1.6027243E38F);
    assert(p105_temperature_GET(pack) == (float) -1.0168329E38F);
    assert(p105_zacc_GET(pack) == (float) -1.732016E38F);
    assert(p105_ygyro_GET(pack) == (float) -3.2566494E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)19109);
    assert(p105_diff_pressure_GET(pack) == (float) -1.3490089E38F);
    assert(p105_yacc_GET(pack) == (float) -1.5299391E38F);
    assert(p105_pressure_alt_GET(pack) == (float)1.6148027E38F);
    assert(p105_xgyro_GET(pack) == (float) -4.9607366E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)24262);
    assert(p106_time_usec_GET(pack) == (uint64_t)8003489233565677399L);
    assert(p106_integrated_zgyro_GET(pack) == (float)1.2008081E37F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -3.126758E38F);
    assert(p106_integrated_x_GET(pack) == (float) -5.5657275E37F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2980944931L);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p106_distance_GET(pack) == (float)1.728803E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -2.1543537E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1118798630L);
    assert(p106_integrated_y_GET(pack) == (float)2.0022422E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float) -1.6249568E37F);
    assert(p107_yacc_GET(pack) == (float) -2.747708E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.7110931E38F);
    assert(p107_diff_pressure_GET(pack) == (float)1.6983134E38F);
    assert(p107_xgyro_GET(pack) == (float) -2.647673E38F);
    assert(p107_zacc_GET(pack) == (float)2.1877051E38F);
    assert(p107_zgyro_GET(pack) == (float) -4.1061066E37F);
    assert(p107_abs_pressure_GET(pack) == (float) -1.9200455E38F);
    assert(p107_ymag_GET(pack) == (float) -2.1291788E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)3361173134042086987L);
    assert(p107_temperature_GET(pack) == (float) -1.6725603E38F);
    assert(p107_xmag_GET(pack) == (float) -8.3108167E37F);
    assert(p107_ygyro_GET(pack) == (float)2.4385166E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)534919615L);
    assert(p107_xacc_GET(pack) == (float)1.5725522E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_ve_GET(pack) == (float)4.230914E37F);
    assert(p108_zacc_GET(pack) == (float)2.4667706E38F);
    assert(p108_q3_GET(pack) == (float) -2.9554069E38F);
    assert(p108_xgyro_GET(pack) == (float)2.0043899E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)3.370778E37F);
    assert(p108_zgyro_GET(pack) == (float)8.983614E36F);
    assert(p108_alt_GET(pack) == (float)1.3209895E38F);
    assert(p108_vn_GET(pack) == (float)3.0255019E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -1.4873698E38F);
    assert(p108_roll_GET(pack) == (float)4.3616216E37F);
    assert(p108_lat_GET(pack) == (float) -2.5727918E38F);
    assert(p108_xacc_GET(pack) == (float) -2.9350882E38F);
    assert(p108_q1_GET(pack) == (float) -3.936162E37F);
    assert(p108_lon_GET(pack) == (float) -2.0154574E38F);
    assert(p108_yacc_GET(pack) == (float) -1.5963142E38F);
    assert(p108_yaw_GET(pack) == (float)1.1899777E38F);
    assert(p108_q2_GET(pack) == (float) -2.455119E38F);
    assert(p108_q4_GET(pack) == (float)1.3679149E38F);
    assert(p108_pitch_GET(pack) == (float) -7.187141E37F);
    assert(p108_vd_GET(pack) == (float) -2.8891988E38F);
    assert(p108_ygyro_GET(pack) == (float)9.946257E37F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)6606);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)41758);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)47);
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)235, (uint8_t)96, (uint8_t)77, (uint8_t)31, (uint8_t)64, (uint8_t)204, (uint8_t)221, (uint8_t)151, (uint8_t)87, (uint8_t)35, (uint8_t)252, (uint8_t)135, (uint8_t)3, (uint8_t)145, (uint8_t)26, (uint8_t)222, (uint8_t)159, (uint8_t)140, (uint8_t)75, (uint8_t)132, (uint8_t)116, (uint8_t)178, (uint8_t)64, (uint8_t)194, (uint8_t)112, (uint8_t)240, (uint8_t)136, (uint8_t)62, (uint8_t)238, (uint8_t)221, (uint8_t)50, (uint8_t)150, (uint8_t)190, (uint8_t)46, (uint8_t)212, (uint8_t)3, (uint8_t)63, (uint8_t)129, (uint8_t)5, (uint8_t)176, (uint8_t)2, (uint8_t)215, (uint8_t)151, (uint8_t)248, (uint8_t)235, (uint8_t)48, (uint8_t)6, (uint8_t)145, (uint8_t)87, (uint8_t)139, (uint8_t)57, (uint8_t)117, (uint8_t)64, (uint8_t)115, (uint8_t)85, (uint8_t)207, (uint8_t)201, (uint8_t)221, (uint8_t)211, (uint8_t)6, (uint8_t)185, (uint8_t)6, (uint8_t)176, (uint8_t)83, (uint8_t)214, (uint8_t)67, (uint8_t)99, (uint8_t)225, (uint8_t)207, (uint8_t)189, (uint8_t)2, (uint8_t)52, (uint8_t)82, (uint8_t)140, (uint8_t)132, (uint8_t)17, (uint8_t)177, (uint8_t)127, (uint8_t)112, (uint8_t)201, (uint8_t)68, (uint8_t)28, (uint8_t)216, (uint8_t)162, (uint8_t)250, (uint8_t)135, (uint8_t)120, (uint8_t)173, (uint8_t)175, (uint8_t)218, (uint8_t)245, (uint8_t)92, (uint8_t)52, (uint8_t)170, (uint8_t)146, (uint8_t)129, (uint8_t)22, (uint8_t)154, (uint8_t)162, (uint8_t)251, (uint8_t)134, (uint8_t)83, (uint8_t)114, (uint8_t)234, (uint8_t)81, (uint8_t)108, (uint8_t)72, (uint8_t)1, (uint8_t)174, (uint8_t)211, (uint8_t)144, (uint8_t)183, (uint8_t)4, (uint8_t)183, (uint8_t)82, (uint8_t)204, (uint8_t)167, (uint8_t)162, (uint8_t)196, (uint8_t)94, (uint8_t)68, (uint8_t)132, (uint8_t)81, (uint8_t)35, (uint8_t)255, (uint8_t)123, (uint8_t)203, (uint8_t)228, (uint8_t)10, (uint8_t)244, (uint8_t)3, (uint8_t)206, (uint8_t)169, (uint8_t)116, (uint8_t)110, (uint8_t)51, (uint8_t)158, (uint8_t)51, (uint8_t)204, (uint8_t)174, (uint8_t)71, (uint8_t)49, (uint8_t)254, (uint8_t)47, (uint8_t)160, (uint8_t)41, (uint8_t)116, (uint8_t)38, (uint8_t)201, (uint8_t)55, (uint8_t)94, (uint8_t)111, (uint8_t)115, (uint8_t)202, (uint8_t)221, (uint8_t)46, (uint8_t)165, (uint8_t)119, (uint8_t)88, (uint8_t)151, (uint8_t)224, (uint8_t)60, (uint8_t)140, (uint8_t)143, (uint8_t)23, (uint8_t)44, (uint8_t)208, (uint8_t)17, (uint8_t)103, (uint8_t)94, (uint8_t)92, (uint8_t)241, (uint8_t)198, (uint8_t)45, (uint8_t)28, (uint8_t)25, (uint8_t)145, (uint8_t)74, (uint8_t)32, (uint8_t)102, (uint8_t)247, (uint8_t)65, (uint8_t)128, (uint8_t)160, (uint8_t)1, (uint8_t)169, (uint8_t)222, (uint8_t)243, (uint8_t)57, (uint8_t)77, (uint8_t)142, (uint8_t)6, (uint8_t)7, (uint8_t)42, (uint8_t)134, (uint8_t)251, (uint8_t)94, (uint8_t)3, (uint8_t)68, (uint8_t)247, (uint8_t)136, (uint8_t)90, (uint8_t)111, (uint8_t)78, (uint8_t)214, (uint8_t)166, (uint8_t)240, (uint8_t)36, (uint8_t)110, (uint8_t)195, (uint8_t)148, (uint8_t)72, (uint8_t)91, (uint8_t)210, (uint8_t)232, (uint8_t)99, (uint8_t)199, (uint8_t)193, (uint8_t)144, (uint8_t)95, (uint8_t)253, (uint8_t)195, (uint8_t)226, (uint8_t)125, (uint8_t)110, (uint8_t)153, (uint8_t)106, (uint8_t)159, (uint8_t)76, (uint8_t)150, (uint8_t)137, (uint8_t)62, (uint8_t)203, (uint8_t)219, (uint8_t)25, (uint8_t)166, (uint8_t)110, (uint8_t)183, (uint8_t)37, (uint8_t)226, (uint8_t)111, (uint8_t)199, (uint8_t)6, (uint8_t)5, (uint8_t)46, (uint8_t)19, (uint8_t)216, (uint8_t)155, (uint8_t)49, (uint8_t)182} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -5063514078532442807L);
    assert(p111_ts1_GET(pack) == (int64_t) -2323993713934035355L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)1041708171683274456L);
    assert(p112_seq_GET(pack) == (uint32_t)1467868336L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -5979);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p113_lat_GET(pack) == (int32_t)525980105);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)30529);
    assert(p113_time_usec_GET(pack) == (uint64_t)3852410421844451087L);
    assert(p113_lon_GET(pack) == (int32_t) -35568768);
    assert(p113_alt_GET(pack) == (int32_t) -329712364);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)28802);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)22069);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)30011);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)40573);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)42419);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)1201527756L);
    assert(p114_integrated_zgyro_GET(pack) == (float) -2.1005709E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float)1.7418978E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)7710763142997791616L);
    assert(p114_integrated_ygyro_GET(pack) == (float) -8.9400545E36F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p114_integrated_y_GET(pack) == (float)1.1937839E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)3672929141L);
    assert(p114_distance_GET(pack) == (float) -2.301962E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)12345);
    assert(p114_integrated_x_GET(pack) == (float) -1.8280124E38F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -6884);
    assert(p115_lat_GET(pack) == (int32_t)777840675);
    assert(p115_alt_GET(pack) == (int32_t) -1867536338);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)3128);
    assert(p115_time_usec_GET(pack) == (uint64_t)5148363539189495964L);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -4610);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)9775);
    assert(p115_lon_GET(pack) == (int32_t)1244990876);
    assert(p115_yawspeed_GET(pack) == (float)1.8527659E38F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -547);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)19246);
    {
        float exemplary[] =  {-2.7165245E38F, 3.0161422E38F, -3.0812862E38F, -2.0315806E36F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -4552);
    assert(p115_rollspeed_GET(pack) == (float)3.2589826E38F);
    assert(p115_pitchspeed_GET(pack) == (float) -1.9807193E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)17666);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)9938);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -26711);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -27279);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)21283);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -1790);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2482794757L);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)12171);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -4136);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -20186);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)20954);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)9244);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)49251);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)0);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_time_utc_GET(pack) == (uint32_t)571211698L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)22166);
    assert(p118_size_GET(pack) == (uint32_t)158582314L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)25073);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)21164);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)17449);
    assert(p119_ofs_GET(pack) == (uint32_t)631404769L);
    assert(p119_count_GET(pack) == (uint32_t)3285822087L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)3379386499L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)40597);
    {
        uint8_t exemplary[] =  {(uint8_t)160, (uint8_t)66, (uint8_t)117, (uint8_t)182, (uint8_t)218, (uint8_t)10, (uint8_t)248, (uint8_t)166, (uint8_t)106, (uint8_t)132, (uint8_t)81, (uint8_t)215, (uint8_t)208, (uint8_t)149, (uint8_t)23, (uint8_t)191, (uint8_t)37, (uint8_t)110, (uint8_t)239, (uint8_t)52, (uint8_t)82, (uint8_t)179, (uint8_t)141, (uint8_t)171, (uint8_t)49, (uint8_t)24, (uint8_t)193, (uint8_t)178, (uint8_t)130, (uint8_t)244, (uint8_t)242, (uint8_t)249, (uint8_t)74, (uint8_t)98, (uint8_t)32, (uint8_t)220, (uint8_t)243, (uint8_t)208, (uint8_t)60, (uint8_t)155, (uint8_t)93, (uint8_t)204, (uint8_t)201, (uint8_t)121, (uint8_t)196, (uint8_t)29, (uint8_t)160, (uint8_t)201, (uint8_t)238, (uint8_t)205, (uint8_t)169, (uint8_t)24, (uint8_t)92, (uint8_t)61, (uint8_t)7, (uint8_t)127, (uint8_t)65, (uint8_t)55, (uint8_t)172, (uint8_t)107, (uint8_t)128, (uint8_t)178, (uint8_t)15, (uint8_t)216, (uint8_t)124, (uint8_t)54, (uint8_t)110, (uint8_t)14, (uint8_t)28, (uint8_t)239, (uint8_t)100, (uint8_t)47, (uint8_t)19, (uint8_t)224, (uint8_t)83, (uint8_t)216, (uint8_t)58, (uint8_t)115, (uint8_t)107, (uint8_t)67, (uint8_t)198, (uint8_t)68, (uint8_t)7, (uint8_t)226, (uint8_t)50, (uint8_t)251, (uint8_t)69, (uint8_t)9, (uint8_t)236, (uint8_t)215} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)228);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)134);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)69);
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)125, (uint8_t)48, (uint8_t)165, (uint8_t)21, (uint8_t)105, (uint8_t)39, (uint8_t)188, (uint8_t)36, (uint8_t)11, (uint8_t)81, (uint8_t)200, (uint8_t)177, (uint8_t)94, (uint8_t)137, (uint8_t)103, (uint8_t)70, (uint8_t)84, (uint8_t)252, (uint8_t)184, (uint8_t)209, (uint8_t)21, (uint8_t)124, (uint8_t)105, (uint8_t)15, (uint8_t)97, (uint8_t)21, (uint8_t)126, (uint8_t)91, (uint8_t)88, (uint8_t)220, (uint8_t)63, (uint8_t)25, (uint8_t)18, (uint8_t)241, (uint8_t)186, (uint8_t)252, (uint8_t)46, (uint8_t)194, (uint8_t)169, (uint8_t)129, (uint8_t)152, (uint8_t)216, (uint8_t)254, (uint8_t)224, (uint8_t)229, (uint8_t)157, (uint8_t)17, (uint8_t)194, (uint8_t)151, (uint8_t)167, (uint8_t)82, (uint8_t)236, (uint8_t)38, (uint8_t)234, (uint8_t)80, (uint8_t)20, (uint8_t)255, (uint8_t)170, (uint8_t)224, (uint8_t)248, (uint8_t)5, (uint8_t)16, (uint8_t)81, (uint8_t)146, (uint8_t)158, (uint8_t)230, (uint8_t)248, (uint8_t)125, (uint8_t)90, (uint8_t)40, (uint8_t)110, (uint8_t)142, (uint8_t)1, (uint8_t)44, (uint8_t)167, (uint8_t)95, (uint8_t)241, (uint8_t)136, (uint8_t)37, (uint8_t)49, (uint8_t)138, (uint8_t)134, (uint8_t)25, (uint8_t)128, (uint8_t)10, (uint8_t)191, (uint8_t)232, (uint8_t)160, (uint8_t)171, (uint8_t)92, (uint8_t)249, (uint8_t)83, (uint8_t)227, (uint8_t)171, (uint8_t)229, (uint8_t)66, (uint8_t)22, (uint8_t)249, (uint8_t)83, (uint8_t)216, (uint8_t)36, (uint8_t)219, (uint8_t)232, (uint8_t)54, (uint8_t)46, (uint8_t)56, (uint8_t)24, (uint8_t)53, (uint8_t)205} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p124_alt_GET(pack) == (int32_t) -1959231154);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p124_time_usec_GET(pack) == (uint64_t)5047329237285931048L);
    assert(p124_lon_GET(pack) == (int32_t)1509377437);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)57801);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)45556);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)50155);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1508189028L);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)14136);
    assert(p124_lat_GET(pack) == (int32_t)970947804);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)11168);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID));
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)13212);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI));
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p126_baudrate_GET(pack) == (uint32_t)240174066L);
    {
        uint8_t exemplary[] =  {(uint8_t)222, (uint8_t)55, (uint8_t)60, (uint8_t)143, (uint8_t)237, (uint8_t)64, (uint8_t)98, (uint8_t)232, (uint8_t)36, (uint8_t)236, (uint8_t)133, (uint8_t)103, (uint8_t)79, (uint8_t)84, (uint8_t)142, (uint8_t)173, (uint8_t)208, (uint8_t)199, (uint8_t)215, (uint8_t)19, (uint8_t)224, (uint8_t)86, (uint8_t)24, (uint8_t)146, (uint8_t)220, (uint8_t)219, (uint8_t)31, (uint8_t)184, (uint8_t)194, (uint8_t)201, (uint8_t)196, (uint8_t)44, (uint8_t)223, (uint8_t)146, (uint8_t)129, (uint8_t)68, (uint8_t)239, (uint8_t)14, (uint8_t)227, (uint8_t)167, (uint8_t)157, (uint8_t)212, (uint8_t)153, (uint8_t)29, (uint8_t)88, (uint8_t)176, (uint8_t)209, (uint8_t)79, (uint8_t)34, (uint8_t)74, (uint8_t)151, (uint8_t)110, (uint8_t)11, (uint8_t)41, (uint8_t)172, (uint8_t)196, (uint8_t)72, (uint8_t)120, (uint8_t)98, (uint8_t)246, (uint8_t)116, (uint8_t)174, (uint8_t)57, (uint8_t)182, (uint8_t)131, (uint8_t)132, (uint8_t)93, (uint8_t)86, (uint8_t)251, (uint8_t)253} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)23598);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)249546146);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)21534);
    assert(p127_tow_GET(pack) == (uint32_t)3407032253L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)4282744504L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -2076951099);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1245591404);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1032322648);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p127_accuracy_GET(pack) == (uint32_t)695798664L);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_b_mm_GET(pack) == (int32_t)25944539);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4251687131L);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)1460436438);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)155176704);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p128_accuracy_GET(pack) == (uint32_t)3967430616L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)26899);
    assert(p128_tow_GET(pack) == (uint32_t)4230919457L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -726066085);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)19913);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -26855);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -10989);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)3917002210L);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -10448);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)24513);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)7510);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)3125);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)23563);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)23056);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)6884);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)60531);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p130_size_GET(pack) == (uint32_t)2844311304L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)48093);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)37, (uint8_t)172, (uint8_t)118, (uint8_t)68, (uint8_t)164, (uint8_t)177, (uint8_t)60, (uint8_t)232, (uint8_t)129, (uint8_t)20, (uint8_t)126, (uint8_t)142, (uint8_t)37, (uint8_t)162, (uint8_t)41, (uint8_t)171, (uint8_t)226, (uint8_t)253, (uint8_t)218, (uint8_t)11, (uint8_t)121, (uint8_t)157, (uint8_t)12, (uint8_t)56, (uint8_t)158, (uint8_t)111, (uint8_t)237, (uint8_t)103, (uint8_t)73, (uint8_t)186, (uint8_t)219, (uint8_t)246, (uint8_t)45, (uint8_t)41, (uint8_t)251, (uint8_t)247, (uint8_t)103, (uint8_t)240, (uint8_t)14, (uint8_t)4, (uint8_t)54, (uint8_t)76, (uint8_t)252, (uint8_t)114, (uint8_t)61, (uint8_t)192, (uint8_t)220, (uint8_t)39, (uint8_t)25, (uint8_t)220, (uint8_t)227, (uint8_t)41, (uint8_t)132, (uint8_t)123, (uint8_t)120, (uint8_t)56, (uint8_t)214, (uint8_t)203, (uint8_t)146, (uint8_t)21, (uint8_t)78, (uint8_t)3, (uint8_t)13, (uint8_t)209, (uint8_t)194, (uint8_t)139, (uint8_t)206, (uint8_t)90, (uint8_t)179, (uint8_t)185, (uint8_t)207, (uint8_t)217, (uint8_t)86, (uint8_t)5, (uint8_t)2, (uint8_t)154, (uint8_t)223, (uint8_t)62, (uint8_t)14, (uint8_t)44, (uint8_t)182, (uint8_t)248, (uint8_t)130, (uint8_t)208, (uint8_t)158, (uint8_t)102, (uint8_t)55, (uint8_t)98, (uint8_t)16, (uint8_t)16, (uint8_t)6, (uint8_t)10, (uint8_t)233, (uint8_t)179, (uint8_t)18, (uint8_t)92, (uint8_t)157, (uint8_t)48, (uint8_t)187, (uint8_t)9, (uint8_t)244, (uint8_t)66, (uint8_t)150, (uint8_t)144, (uint8_t)189, (uint8_t)73, (uint8_t)130, (uint8_t)62, (uint8_t)91, (uint8_t)189, (uint8_t)22, (uint8_t)138, (uint8_t)225, (uint8_t)242, (uint8_t)180, (uint8_t)82, (uint8_t)120, (uint8_t)126, (uint8_t)54, (uint8_t)222, (uint8_t)52, (uint8_t)227, (uint8_t)143, (uint8_t)122, (uint8_t)156, (uint8_t)122, (uint8_t)128, (uint8_t)179, (uint8_t)31, (uint8_t)64, (uint8_t)1, (uint8_t)127, (uint8_t)199, (uint8_t)93, (uint8_t)134, (uint8_t)87, (uint8_t)67, (uint8_t)253, (uint8_t)241, (uint8_t)50, (uint8_t)46, (uint8_t)124, (uint8_t)42, (uint8_t)172, (uint8_t)93, (uint8_t)166, (uint8_t)199, (uint8_t)84, (uint8_t)193, (uint8_t)22, (uint8_t)225, (uint8_t)251, (uint8_t)165, (uint8_t)6, (uint8_t)160, (uint8_t)49, (uint8_t)73, (uint8_t)209, (uint8_t)190, (uint8_t)237, (uint8_t)11, (uint8_t)8, (uint8_t)43, (uint8_t)227, (uint8_t)53, (uint8_t)69, (uint8_t)136, (uint8_t)129, (uint8_t)94, (uint8_t)8, (uint8_t)71, (uint8_t)173, (uint8_t)35, (uint8_t)45, (uint8_t)225, (uint8_t)173, (uint8_t)124, (uint8_t)115, (uint8_t)94, (uint8_t)178, (uint8_t)95, (uint8_t)156, (uint8_t)56, (uint8_t)25, (uint8_t)142, (uint8_t)230, (uint8_t)236, (uint8_t)244, (uint8_t)18, (uint8_t)116, (uint8_t)218, (uint8_t)83, (uint8_t)41, (uint8_t)79, (uint8_t)170, (uint8_t)157, (uint8_t)69, (uint8_t)248, (uint8_t)144, (uint8_t)114, (uint8_t)133, (uint8_t)35, (uint8_t)199, (uint8_t)217, (uint8_t)59, (uint8_t)185, (uint8_t)193, (uint8_t)82, (uint8_t)158, (uint8_t)98, (uint8_t)208, (uint8_t)111, (uint8_t)78, (uint8_t)32, (uint8_t)247, (uint8_t)226, (uint8_t)185, (uint8_t)215, (uint8_t)185, (uint8_t)111, (uint8_t)121, (uint8_t)30, (uint8_t)16, (uint8_t)94, (uint8_t)27, (uint8_t)22, (uint8_t)105, (uint8_t)208, (uint8_t)136, (uint8_t)84, (uint8_t)166, (uint8_t)235, (uint8_t)57, (uint8_t)79, (uint8_t)59, (uint8_t)67, (uint8_t)187, (uint8_t)214, (uint8_t)182, (uint8_t)123, (uint8_t)164, (uint8_t)70, (uint8_t)177, (uint8_t)188, (uint8_t)154, (uint8_t)87, (uint8_t)199, (uint8_t)246, (uint8_t)30, (uint8_t)45, (uint8_t)154, (uint8_t)61, (uint8_t)31} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)13503);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3463580808L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)48148);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)9096);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)22040);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)220);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)5596105160408365810L);
    assert(p133_lat_GET(pack) == (int32_t) -353986407);
    assert(p133_lon_GET(pack) == (int32_t)1228916023);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)51678);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t) -2254, (int16_t)32524, (int16_t)21867, (int16_t) -15860, (int16_t) -17866, (int16_t) -14735, (int16_t)29875, (int16_t) -14333, (int16_t) -1384, (int16_t)15236, (int16_t)20204, (int16_t)21868, (int16_t)26179, (int16_t)9819, (int16_t) -6454, (int16_t) -4824} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lat_GET(pack) == (int32_t)525840344);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p134_lon_GET(pack) == (int32_t)1111815485);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)35551);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -1725774593);
    assert(p135_lat_GET(pack) == (int32_t) -2053563032);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float)6.563776E37F);
    assert(p136_lon_GET(pack) == (int32_t) -409978620);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)7248);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)57338);
    assert(p136_lat_GET(pack) == (int32_t) -481373278);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)58949);
    assert(p136_current_height_GET(pack) == (float)3.7730816E37F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)2360107720L);
    assert(p137_press_abs_GET(pack) == (float)2.9765318E38F);
    assert(p137_press_diff_GET(pack) == (float) -1.5490682E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -12012);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float) -2.7708603E37F);
    assert(p138_y_GET(pack) == (float) -2.5599212E38F);
    assert(p138_z_GET(pack) == (float) -3.3231834E38F);
    {
        float exemplary[] =  {-2.0121323E38F, 2.763543E38F, 1.921227E38F, -1.9454709E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_time_usec_GET(pack) == (uint64_t)48889841410749994L);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)236);
    {
        float exemplary[] =  {-1.7845634E38F, 9.547923E37F, 3.4436824E37F, 1.5197521E38F, 3.1687324E38F, -2.660771E38F, -1.90448E38F, 2.9291227E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p139_time_usec_GET(pack) == (uint64_t)8187692198451616338L);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)7078152087248273356L);
    {
        float exemplary[] =  {1.9711056E38F, -2.3320836E38F, 1.6010648E38F, 2.286581E37F, -2.8492932E38F, 2.1954207E38F, -2.0070035E37F, 1.4504018E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)187);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float)6.357735E37F);
    assert(p141_altitude_amsl_GET(pack) == (float)1.2614218E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)9192553716770657361L);
    assert(p141_altitude_local_GET(pack) == (float) -4.7031866E37F);
    assert(p141_altitude_terrain_GET(pack) == (float)3.0146364E37F);
    assert(p141_altitude_relative_GET(pack) == (float) -2.7667722E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -3.630095E37F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)63, (uint8_t)32, (uint8_t)135, (uint8_t)177, (uint8_t)105, (uint8_t)24, (uint8_t)54, (uint8_t)8, (uint8_t)139, (uint8_t)26, (uint8_t)216, (uint8_t)222, (uint8_t)182, (uint8_t)158, (uint8_t)0, (uint8_t)249, (uint8_t)149, (uint8_t)180, (uint8_t)181, (uint8_t)171, (uint8_t)19, (uint8_t)188, (uint8_t)223, (uint8_t)191, (uint8_t)61, (uint8_t)1, (uint8_t)125, (uint8_t)84, (uint8_t)146, (uint8_t)200, (uint8_t)218, (uint8_t)39, (uint8_t)95, (uint8_t)122, (uint8_t)124, (uint8_t)150, (uint8_t)196, (uint8_t)96, (uint8_t)38, (uint8_t)78, (uint8_t)24, (uint8_t)23, (uint8_t)47, (uint8_t)255, (uint8_t)192, (uint8_t)154, (uint8_t)16, (uint8_t)69, (uint8_t)75, (uint8_t)10, (uint8_t)97, (uint8_t)88, (uint8_t)38, (uint8_t)35, (uint8_t)249, (uint8_t)155, (uint8_t)199, (uint8_t)38, (uint8_t)86, (uint8_t)59, (uint8_t)205, (uint8_t)235, (uint8_t)114, (uint8_t)152, (uint8_t)243, (uint8_t)179, (uint8_t)49, (uint8_t)110, (uint8_t)158, (uint8_t)29, (uint8_t)6, (uint8_t)75, (uint8_t)184, (uint8_t)132, (uint8_t)23, (uint8_t)217, (uint8_t)191, (uint8_t)211, (uint8_t)191, (uint8_t)200, (uint8_t)68, (uint8_t)163, (uint8_t)80, (uint8_t)34, (uint8_t)135, (uint8_t)96, (uint8_t)111, (uint8_t)44, (uint8_t)159, (uint8_t)220, (uint8_t)11, (uint8_t)172, (uint8_t)207, (uint8_t)53, (uint8_t)68, (uint8_t)223, (uint8_t)202, (uint8_t)131, (uint8_t)223, (uint8_t)117, (uint8_t)132, (uint8_t)137, (uint8_t)141, (uint8_t)24, (uint8_t)11, (uint8_t)243, (uint8_t)10, (uint8_t)164, (uint8_t)29, (uint8_t)110, (uint8_t)147, (uint8_t)106, (uint8_t)225, (uint8_t)179, (uint8_t)197, (uint8_t)114, (uint8_t)112, (uint8_t)170, (uint8_t)53, (uint8_t)247} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)74);
    {
        uint8_t exemplary[] =  {(uint8_t)200, (uint8_t)22, (uint8_t)239, (uint8_t)224, (uint8_t)112, (uint8_t)217, (uint8_t)144, (uint8_t)0, (uint8_t)196, (uint8_t)100, (uint8_t)103, (uint8_t)176, (uint8_t)160, (uint8_t)219, (uint8_t)254, (uint8_t)143, (uint8_t)77, (uint8_t)171, (uint8_t)224, (uint8_t)34, (uint8_t)218, (uint8_t)229, (uint8_t)159, (uint8_t)133, (uint8_t)140, (uint8_t)157, (uint8_t)102, (uint8_t)162, (uint8_t)81, (uint8_t)141, (uint8_t)112, (uint8_t)172, (uint8_t)170, (uint8_t)117, (uint8_t)44, (uint8_t)86, (uint8_t)71, (uint8_t)8, (uint8_t)31, (uint8_t)139, (uint8_t)61, (uint8_t)83, (uint8_t)36, (uint8_t)219, (uint8_t)181, (uint8_t)124, (uint8_t)25, (uint8_t)44, (uint8_t)163, (uint8_t)152, (uint8_t)110, (uint8_t)216, (uint8_t)141, (uint8_t)66, (uint8_t)177, (uint8_t)78, (uint8_t)68, (uint8_t)191, (uint8_t)236, (uint8_t)28, (uint8_t)8, (uint8_t)0, (uint8_t)120, (uint8_t)222, (uint8_t)202, (uint8_t)198, (uint8_t)111, (uint8_t)195, (uint8_t)103, (uint8_t)80, (uint8_t)230, (uint8_t)58, (uint8_t)225, (uint8_t)64, (uint8_t)128, (uint8_t)191, (uint8_t)143, (uint8_t)219, (uint8_t)141, (uint8_t)206, (uint8_t)82, (uint8_t)90, (uint8_t)1, (uint8_t)34, (uint8_t)74, (uint8_t)74, (uint8_t)96, (uint8_t)158, (uint8_t)215, (uint8_t)159, (uint8_t)57, (uint8_t)174, (uint8_t)228, (uint8_t)78, (uint8_t)83, (uint8_t)138, (uint8_t)22, (uint8_t)13, (uint8_t)158, (uint8_t)153, (uint8_t)139, (uint8_t)135, (uint8_t)41, (uint8_t)119, (uint8_t)77, (uint8_t)11, (uint8_t)214, (uint8_t)169, (uint8_t)159, (uint8_t)70, (uint8_t)197, (uint8_t)23, (uint8_t)70, (uint8_t)248, (uint8_t)168, (uint8_t)242, (uint8_t)58, (uint8_t)22, (uint8_t)251, (uint8_t)216} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -18086);
    assert(p143_press_abs_GET(pack) == (float)3.3939408E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2014288752L);
    assert(p143_press_diff_GET(pack) == (float)3.0538936E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-4.88537E37F, -3.273396E37F, -1.150865E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1477549743);
    assert(p144_alt_GET(pack) == (float)2.9031365E38F);
    {
        float exemplary[] =  {-1.855359E38F, 1.2217056E38F, 2.2981958E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)4398681343650275183L);
    assert(p144_lat_GET(pack) == (int32_t) -50041618);
    {
        float exemplary[] =  {-2.8763728E38F, 1.7474837E38F, -2.6111511E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p144_custom_state_GET(pack) == (uint64_t)5110574854055931875L);
    {
        float exemplary[] =  {-1.0855069E38F, -1.9052853E38F, -1.9823129E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.6011703E38F, -1.77578E38F, 2.854077E38F, -2.3945754E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_z_vel_GET(pack) == (float)2.6914687E38F);
    assert(p146_x_acc_GET(pack) == (float) -3.1555716E38F);
    assert(p146_y_acc_GET(pack) == (float)1.2805517E38F);
    assert(p146_y_pos_GET(pack) == (float)2.7326904E37F);
    assert(p146_roll_rate_GET(pack) == (float)2.2405136E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)4533545213163992157L);
    assert(p146_z_pos_GET(pack) == (float) -1.820332E38F);
    assert(p146_y_vel_GET(pack) == (float)1.5403712E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.7829088E38F);
    assert(p146_x_pos_GET(pack) == (float) -4.0819784E37F);
    {
        float exemplary[] =  {-1.9709093E37F, -2.410432E38F, 5.154598E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float) -1.4197945E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.0289302E38F);
    {
        float exemplary[] =  {1.6007995E38F, 8.93943E37F, 2.9130193E38F, -2.8801474E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.2827171E38F, -2.981915E38F, 2.2982057E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float)2.8567638E38F);
    assert(p146_z_acc_GET(pack) == (float)1.5866212E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)54);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -17858);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH);
    assert(p147_current_consumed_GET(pack) == (int32_t)1113908176);
    assert(p147_energy_consumed_GET(pack) == (int32_t)195197439);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)26171);
    {
        uint16_t exemplary[] =  {(uint16_t)14921, (uint16_t)29688, (uint16_t)19325, (uint16_t)61912, (uint16_t)62550, (uint16_t)38599, (uint16_t)14247, (uint16_t)12259, (uint16_t)15708, (uint16_t)46458} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1219949014L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)29820);
    {
        uint8_t exemplary[] =  {(uint8_t)154, (uint8_t)192, (uint8_t)101, (uint8_t)191, (uint8_t)212, (uint8_t)248, (uint8_t)92, (uint8_t)123} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION));
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)106, (uint8_t)108, (uint8_t)61, (uint8_t)94, (uint8_t)132, (uint8_t)212, (uint8_t)213} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)852342072L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)63770);
    assert(p148_uid_GET(pack) == (uint64_t)9067564254384443685L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)3933482815L);
    {
        uint8_t exemplary[] =  {(uint8_t)216, (uint8_t)64, (uint8_t)228, (uint8_t)87, (uint8_t)232, (uint8_t)155, (uint8_t)31, (uint8_t)66} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)852891055L);
    {
        uint8_t exemplary[] =  {(uint8_t)137, (uint8_t)243, (uint8_t)151, (uint8_t)138, (uint8_t)14, (uint8_t)251, (uint8_t)117, (uint8_t)225, (uint8_t)73, (uint8_t)42, (uint8_t)15, (uint8_t)86, (uint8_t)25, (uint8_t)249, (uint8_t)48, (uint8_t)10, (uint8_t)91, (uint8_t)168} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_time_usec_GET(pack) == (uint64_t)3074707530477782986L);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)252);
    {
        float exemplary[] =  {1.0530255E38F, 2.7881131E38F, 2.7828659E38F, -2.020443E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_z_TRY(ph) == (float) -1.4770808E38F);
    assert(p149_y_TRY(ph) == (float)1.6411516E38F);
    assert(p149_angle_x_GET(pack) == (float)3.2276252E38F);
    assert(p149_x_TRY(ph) == (float)3.0193352E38F);
    assert(p149_angle_y_GET(pack) == (float)1.2160274E37F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p149_size_y_GET(pack) == (float)7.263641E37F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)255);
    assert(p149_distance_GET(pack) == (float) -1.601615E38F);
    assert(p149_size_x_GET(pack) == (float)1.9876605E38F);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_target_system_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p150_target_component_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p151_read_req_type_GET(pack) == (int16_t)(int16_t) -4815);
    assert(p151_data_index_GET(pack) == (int16_t)(int16_t) -15074);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_target_system_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p152_data_size_GET(pack) == (uint16_t)(uint16_t)8714);
    assert(p152_data_address_GET(pack) == (uint16_t)(uint16_t)60302);
    assert(p152_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    {
        int8_t exemplary[] =  {(int8_t)93, (int8_t)35, (int8_t)108, (int8_t) -58, (int8_t) -87, (int8_t)104, (int8_t) -69, (int8_t) -31, (int8_t) -91, (int8_t) -53, (int8_t) -6, (int8_t) -26, (int8_t) -123, (int8_t)100, (int8_t) -110, (int8_t) -101, (int8_t)93, (int8_t)126, (int8_t)82, (int8_t) -112, (int8_t) -59, (int8_t) -66, (int8_t) -18, (int8_t)10, (int8_t) -29, (int8_t)119, (int8_t) -113, (int8_t) -99, (int8_t)117, (int8_t) -103, (int8_t)91, (int8_t)64, (int8_t)71, (int8_t)71, (int8_t)81, (int8_t)96, (int8_t)68, (int8_t) -74, (int8_t)83, (int8_t)89, (int8_t)105, (int8_t) -111, (int8_t)79, (int8_t) -117, (int8_t)21, (int8_t)38, (int8_t)30, (int8_t) -63} ;
        int8_t*  sample = p152_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_func_index_GET(pack) == (uint16_t)(uint16_t)58171);
    assert(p152_func_count_GET(pack) == (uint16_t)(uint16_t)28135);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_func_index_GET(pack) == (uint16_t)(uint16_t)21210);
    assert(p153_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p153_result_GET(pack) == (uint16_t)(uint16_t)45228);
    assert(p153_target_component_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_directory_type_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p155_start_index_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p155_count_GET(pack) == (uint8_t)(uint8_t)75);
    {
        int8_t exemplary[] =  {(int8_t) -29, (int8_t)122, (int8_t)34, (int8_t) -99, (int8_t)76, (int8_t) -125, (int8_t) -124, (int8_t)97, (int8_t)56, (int8_t) -83, (int8_t) -27, (int8_t) -24, (int8_t) -121, (int8_t)122, (int8_t) -76, (int8_t) -24, (int8_t)106, (int8_t) -62, (int8_t)94, (int8_t) -52, (int8_t)49, (int8_t) -22, (int8_t) -25, (int8_t) -63, (int8_t)82, (int8_t) -29, (int8_t) -126, (int8_t) -56, (int8_t) -79, (int8_t)30, (int8_t)111, (int8_t)87, (int8_t) -25, (int8_t)119, (int8_t)33, (int8_t) -108, (int8_t)102, (int8_t) -56, (int8_t)114, (int8_t) -105, (int8_t)43, (int8_t)71, (int8_t)74, (int8_t) -41, (int8_t)4, (int8_t)93, (int8_t)79, (int8_t)62} ;
        int8_t*  sample = p155_directory_data_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_count_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p156_result_GET(pack) == (uint16_t)(uint16_t)38204);
    assert(p156_start_index_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p156_directory_type_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_command_type_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)117);
};


void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_command_type_GET(pack) == (uint16_t)(uint16_t)17452);
    assert(p158_result_GET(pack) == (uint16_t)(uint16_t)40800);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_sue_rmat7_GET(pack) == (int16_t)(int16_t) -16170);
    assert(p170_sue_cpu_load_GET(pack) == (uint16_t)(uint16_t)19197);
    assert(p170_sue_rmat3_GET(pack) == (int16_t)(int16_t)26516);
    assert(p170_sue_latitude_GET(pack) == (int32_t) -693891273);
    assert(p170_sue_altitude_GET(pack) == (int32_t)148488354);
    assert(p170_sue_rmat6_GET(pack) == (int16_t)(int16_t)32157);
    assert(p170_sue_sog_GET(pack) == (int16_t)(int16_t) -31571);
    assert(p170_sue_rmat1_GET(pack) == (int16_t)(int16_t) -24245);
    assert(p170_sue_rmat2_GET(pack) == (int16_t)(int16_t) -10012);
    assert(p170_sue_estimated_wind_1_GET(pack) == (int16_t)(int16_t)31101);
    assert(p170_sue_rmat0_GET(pack) == (int16_t)(int16_t)6663);
    assert(p170_sue_waypoint_index_GET(pack) == (uint16_t)(uint16_t)49667);
    assert(p170_sue_estimated_wind_2_GET(pack) == (int16_t)(int16_t)18899);
    assert(p170_sue_estimated_wind_0_GET(pack) == (int16_t)(int16_t)11191);
    assert(p170_sue_magFieldEarth1_GET(pack) == (int16_t)(int16_t)25105);
    assert(p170_sue_longitude_GET(pack) == (int32_t)2069303475);
    assert(p170_sue_magFieldEarth2_GET(pack) == (int16_t)(int16_t)23762);
    assert(p170_sue_rmat5_GET(pack) == (int16_t)(int16_t)14992);
    assert(p170_sue_cog_GET(pack) == (uint16_t)(uint16_t)61663);
    assert(p170_sue_time_GET(pack) == (uint32_t)3484252340L);
    assert(p170_sue_hdop_GET(pack) == (int16_t)(int16_t) -11680);
    assert(p170_sue_status_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p170_sue_rmat8_GET(pack) == (int16_t)(int16_t)4809);
    assert(p170_sue_svs_GET(pack) == (int16_t)(int16_t) -19214);
    assert(p170_sue_magFieldEarth0_GET(pack) == (int16_t)(int16_t) -5745);
    assert(p170_sue_rmat4_GET(pack) == (int16_t)(int16_t) -8816);
    assert(p170_sue_air_speed_3DIMU_GET(pack) == (uint16_t)(uint16_t)43264);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_sue_pwm_output_10_GET(pack) == (int16_t)(int16_t)30722);
    assert(p171_sue_pwm_input_12_GET(pack) == (int16_t)(int16_t)26934);
    assert(p171_sue_waypoint_goal_x_GET(pack) == (int16_t)(int16_t) -26670);
    assert(p171_sue_imu_location_x_GET(pack) == (int16_t)(int16_t) -13597);
    assert(p171_sue_waypoint_goal_y_GET(pack) == (int16_t)(int16_t)23775);
    assert(p171_sue_barom_temp_GET(pack) == (int16_t)(int16_t)12287);
    assert(p171_sue_time_GET(pack) == (uint32_t)55687832L);
    assert(p171_sue_imu_velocity_z_GET(pack) == (int16_t)(int16_t)19031);
    assert(p171_sue_pwm_input_1_GET(pack) == (int16_t)(int16_t)16661);
    assert(p171_sue_osc_fails_GET(pack) == (int16_t)(int16_t) -16411);
    assert(p171_sue_desired_height_GET(pack) == (int16_t)(int16_t)19161);
    assert(p171_sue_memory_stack_free_GET(pack) == (int16_t)(int16_t) -4709);
    assert(p171_sue_location_error_earth_x_GET(pack) == (int16_t)(int16_t)10397);
    assert(p171_sue_barom_press_GET(pack) == (int32_t) -846929416);
    assert(p171_sue_pwm_input_3_GET(pack) == (int16_t)(int16_t)25951);
    assert(p171_sue_pwm_output_7_GET(pack) == (int16_t)(int16_t)3847);
    assert(p171_sue_aero_y_GET(pack) == (int16_t)(int16_t) -30510);
    assert(p171_sue_bat_volt_GET(pack) == (int16_t)(int16_t) -2933);
    assert(p171_sue_imu_location_z_GET(pack) == (int16_t)(int16_t)6443);
    assert(p171_sue_pwm_output_12_GET(pack) == (int16_t)(int16_t) -11904);
    assert(p171_sue_barom_alt_GET(pack) == (int32_t) -2091844492);
    assert(p171_sue_bat_amp_GET(pack) == (int16_t)(int16_t)13443);
    assert(p171_sue_pwm_input_11_GET(pack) == (int16_t)(int16_t)20321);
    assert(p171_sue_imu_location_y_GET(pack) == (int16_t)(int16_t)23580);
    assert(p171_sue_pwm_output_1_GET(pack) == (int16_t)(int16_t) -1308);
    assert(p171_sue_pwm_input_9_GET(pack) == (int16_t)(int16_t) -25839);
    assert(p171_sue_pwm_input_7_GET(pack) == (int16_t)(int16_t) -16618);
    assert(p171_sue_aero_z_GET(pack) == (int16_t)(int16_t)22265);
    assert(p171_sue_pwm_output_9_GET(pack) == (int16_t)(int16_t) -23124);
    assert(p171_sue_aero_x_GET(pack) == (int16_t)(int16_t)9178);
    assert(p171_sue_pwm_output_8_GET(pack) == (int16_t)(int16_t) -974);
    assert(p171_sue_location_error_earth_z_GET(pack) == (int16_t)(int16_t) -24808);
    assert(p171_sue_pwm_output_5_GET(pack) == (int16_t)(int16_t)14746);
    assert(p171_sue_location_error_earth_y_GET(pack) == (int16_t)(int16_t)22502);
    assert(p171_sue_imu_velocity_x_GET(pack) == (int16_t)(int16_t) -11678);
    assert(p171_sue_bat_amp_hours_GET(pack) == (int16_t)(int16_t) -25578);
    assert(p171_sue_pwm_output_2_GET(pack) == (int16_t)(int16_t)14158);
    assert(p171_sue_waypoint_goal_z_GET(pack) == (int16_t)(int16_t)18269);
    assert(p171_sue_pwm_input_4_GET(pack) == (int16_t)(int16_t)9858);
    assert(p171_sue_pwm_input_5_GET(pack) == (int16_t)(int16_t)7360);
    assert(p171_sue_pwm_output_11_GET(pack) == (int16_t)(int16_t)30410);
    assert(p171_sue_pwm_output_6_GET(pack) == (int16_t)(int16_t)28949);
    assert(p171_sue_imu_velocity_y_GET(pack) == (int16_t)(int16_t) -5879);
    assert(p171_sue_pwm_output_4_GET(pack) == (int16_t)(int16_t) -9654);
    assert(p171_sue_pwm_input_2_GET(pack) == (int16_t)(int16_t) -4269);
    assert(p171_sue_pwm_input_6_GET(pack) == (int16_t)(int16_t) -23743);
    assert(p171_sue_pwm_input_8_GET(pack) == (int16_t)(int16_t) -27934);
    assert(p171_sue_flags_GET(pack) == (uint32_t)4185152880L);
    assert(p171_sue_pwm_input_10_GET(pack) == (int16_t)(int16_t) -2733);
    assert(p171_sue_pwm_output_3_GET(pack) == (int16_t)(int16_t) -25170);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_sue_ROLL_STABILIZATION_AILERONS_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p172_sue_PITCH_STABILIZATION_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p172_sue_YAW_STABILIZATION_AILERON_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p172_sue_ROLL_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p172_sue_RACING_MODE_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p172_sue_ALTITUDEHOLD_STABILIZED_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p172_sue_RUDDER_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p172_sue_YAW_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p172_sue_AILERON_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p172_sue_ALTITUDEHOLD_WAYPOINT_GET(pack) == (uint8_t)(uint8_t)170);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_sue_ROLLKP_GET(pack) == (float)3.037306E38F);
    assert(p173_sue_YAWKD_AILERON_GET(pack) == (float) -2.8088142E38F);
    assert(p173_sue_ROLLKD_GET(pack) == (float)7.223927E37F);
    assert(p173_sue_YAWKP_AILERON_GET(pack) == (float)8.450248E37F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_sue_ROLL_ELEV_MIX_GET(pack) == (float)1.8291379E38F);
    assert(p174_sue_RUDDER_ELEV_MIX_GET(pack) == (float) -2.4906174E38F);
    assert(p174_sue_ELEVATOR_BOOST_GET(pack) == (float)6.379799E37F);
    assert(p174_sue_PITCHGAIN_GET(pack) == (float)2.9056334E37F);
    assert(p174_sue_PITCHKD_GET(pack) == (float) -1.5711111E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_sue_ROLLKP_RUDDER_GET(pack) == (float)9.828309E37F);
    assert(p175_sue_ROLLKD_RUDDER_GET(pack) == (float)1.8181799E38F);
    assert(p175_sue_RTL_PITCH_DOWN_GET(pack) == (float)2.9663704E38F);
    assert(p175_sue_RUDDER_BOOST_GET(pack) == (float)3.2700168E37F);
    assert(p175_sue_YAWKD_RUDDER_GET(pack) == (float)1.1987283E38F);
    assert(p175_sue_YAWKP_RUDDER_GET(pack) == (float) -1.6174207E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_sue_ALT_HOLD_PITCH_MIN_GET(pack) == (float) -1.6597462E37F);
    assert(p176_sue_ALT_HOLD_PITCH_MAX_GET(pack) == (float) -7.1720217E36F);
    assert(p176_sue_HEIGHT_TARGET_MAX_GET(pack) == (float) -1.0712995E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack) == (float) -2.6825527E38F);
    assert(p176_sue_HEIGHT_TARGET_MIN_GET(pack) == (float)1.0155596E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack) == (float) -7.2600043E37F);
    assert(p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack) == (float)3.4019544E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_sue_alt_origin_GET(pack) == (int32_t)338786242);
    assert(p177_sue_lat_origin_GET(pack) == (int32_t)2139992110);
    assert(p177_sue_week_no_GET(pack) == (int16_t)(int16_t) -20405);
    assert(p177_sue_lon_origin_GET(pack) == (int32_t) -1889006497);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_sue_osc_fail_count_GET(pack) == (int16_t)(int16_t) -31542);
    assert(p178_sue_RCON_GET(pack) == (int16_t)(int16_t) -4969);
    assert(p178_sue_AIRFRAME_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p178_sue_TRAP_SOURCE_GET(pack) == (uint32_t)2317053118L);
    assert(p178_sue_GPS_TYPE_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p178_sue_DR_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p178_sue_CLOCK_CONFIG_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p178_sue_TRAP_FLAGS_GET(pack) == (int16_t)(int16_t)15274);
    assert(p178_sue_BOARD_TYPE_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p178_sue_FLIGHT_PLAN_TYPE_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p178_sue_WIND_ESTIMATION_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)30, (uint8_t)148, (uint8_t)35, (uint8_t)177, (uint8_t)149, (uint8_t)88, (uint8_t)72, (uint8_t)182, (uint8_t)34, (uint8_t)107, (uint8_t)137, (uint8_t)240, (uint8_t)117, (uint8_t)239, (uint8_t)222, (uint8_t)93, (uint8_t)236, (uint8_t)52, (uint8_t)209, (uint8_t)208, (uint8_t)94, (uint8_t)109, (uint8_t)129, (uint8_t)251, (uint8_t)236, (uint8_t)250, (uint8_t)43, (uint8_t)201, (uint8_t)98, (uint8_t)109, (uint8_t)145, (uint8_t)50, (uint8_t)111, (uint8_t)94, (uint8_t)16, (uint8_t)112, (uint8_t)177, (uint8_t)132, (uint8_t)200} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)223, (uint8_t)208, (uint8_t)27, (uint8_t)22, (uint8_t)243, (uint8_t)252, (uint8_t)191, (uint8_t)153, (uint8_t)195, (uint8_t)142, (uint8_t)27, (uint8_t)244, (uint8_t)24, (uint8_t)199, (uint8_t)37, (uint8_t)6, (uint8_t)137, (uint8_t)14, (uint8_t)128, (uint8_t)182} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)217, (uint8_t)52, (uint8_t)143, (uint8_t)253, (uint8_t)183, (uint8_t)121, (uint8_t)28, (uint8_t)176, (uint8_t)210, (uint8_t)70, (uint8_t)36, (uint8_t)117, (uint8_t)97, (uint8_t)184, (uint8_t)53, (uint8_t)172, (uint8_t)95, (uint8_t)172, (uint8_t)91, (uint8_t)236, (uint8_t)63, (uint8_t)140, (uint8_t)126, (uint8_t)9, (uint8_t)167, (uint8_t)186, (uint8_t)38, (uint8_t)94, (uint8_t)41, (uint8_t)119, (uint8_t)189, (uint8_t)176, (uint8_t)190, (uint8_t)84, (uint8_t)119, (uint8_t)2, (uint8_t)88, (uint8_t)184, (uint8_t)98} ;
        uint8_t*  sample = p180_sue_ID_LEAD_PILOT_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)106, (uint8_t)91, (uint8_t)18, (uint8_t)109, (uint8_t)14, (uint8_t)241, (uint8_t)52, (uint8_t)23, (uint8_t)47, (uint8_t)142, (uint8_t)248, (uint8_t)175, (uint8_t)49, (uint8_t)90, (uint8_t)4, (uint8_t)109, (uint8_t)249, (uint8_t)244, (uint8_t)249, (uint8_t)227, (uint8_t)76, (uint8_t)9, (uint8_t)1, (uint8_t)82, (uint8_t)151, (uint8_t)169, (uint8_t)43, (uint8_t)162, (uint8_t)6, (uint8_t)4, (uint8_t)38, (uint8_t)123, (uint8_t)135, (uint8_t)38, (uint8_t)197, (uint8_t)31, (uint8_t)37, (uint8_t)10, (uint8_t)190, (uint8_t)97, (uint8_t)173, (uint8_t)9, (uint8_t)169, (uint8_t)203, (uint8_t)179, (uint8_t)175, (uint8_t)59, (uint8_t)204, (uint8_t)162, (uint8_t)183, (uint8_t)175, (uint8_t)251, (uint8_t)61, (uint8_t)235, (uint8_t)5, (uint8_t)91, (uint8_t)47, (uint8_t)132, (uint8_t)92, (uint8_t)109, (uint8_t)92, (uint8_t)10, (uint8_t)85, (uint8_t)51, (uint8_t)182, (uint8_t)244, (uint8_t)196, (uint8_t)45, (uint8_t)51} ;
        uint8_t*  sample = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_alt_imu_GET(pack) == (int32_t)704035024);
    assert(p181_alt_range_finder_GET(pack) == (int32_t)544756421);
    assert(p181_alt_optical_flow_GET(pack) == (int32_t) -607629615);
    assert(p181_alt_gps_GET(pack) == (int32_t)2113711184);
    assert(p181_alt_extra_GET(pack) == (int32_t)1310662883);
    assert(p181_alt_barometric_GET(pack) == (int32_t)1868092585);
    assert(p181_time_boot_ms_GET(pack) == (uint32_t)164585507L);
};


void c_CommunicationChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_aoa_GET(pack) == (int16_t)(int16_t) -10075);
    assert(p182_airspeed_ultrasonic_GET(pack) == (int16_t)(int16_t)28330);
    assert(p182_airspeed_imu_GET(pack) == (int16_t)(int16_t) -21164);
    assert(p182_time_boot_ms_GET(pack) == (uint32_t)1659293800L);
    assert(p182_airspeed_pitot_GET(pack) == (int16_t)(int16_t)25121);
    assert(p182_airspeed_hot_wire_GET(pack) == (int16_t)(int16_t)21550);
    assert(p182_aoy_GET(pack) == (int16_t)(int16_t) -5633);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_sue_feed_forward_GET(pack) == (float)2.7609266E38F);
    assert(p183_sue_turn_rate_fbw_GET(pack) == (float)3.8468716E37F);
    assert(p183_sue_turn_rate_nav_GET(pack) == (float) -1.601833E38F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_angle_of_attack_normal_GET(pack) == (float) -2.1624276E38F);
    assert(p184_elevator_trim_normal_GET(pack) == (float) -2.4729016E38F);
    assert(p184_angle_of_attack_inverted_GET(pack) == (float) -7.1742113E37F);
    assert(p184_elevator_trim_inverted_GET(pack) == (float) -1.6673615E38F);
    assert(p184_reference_speed_GET(pack) == (float) -4.7042316E37F);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_sue_aileron_reversed_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p185_sue_rudder_reversed_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p185_sue_throttle_output_channel_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p185_sue_throttle_reversed_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p185_sue_aileron_output_channel_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p185_sue_elevator_output_channel_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p185_sue_elevator_reversed_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p185_sue_rudder_output_channel_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_sue_trim_value_input_12_GET(pack) == (int16_t)(int16_t) -19531);
    assert(p186_sue_trim_value_input_2_GET(pack) == (int16_t)(int16_t)5248);
    assert(p186_sue_trim_value_input_10_GET(pack) == (int16_t)(int16_t)23091);
    assert(p186_sue_trim_value_input_1_GET(pack) == (int16_t)(int16_t) -9748);
    assert(p186_sue_trim_value_input_5_GET(pack) == (int16_t)(int16_t) -22804);
    assert(p186_sue_trim_value_input_3_GET(pack) == (int16_t)(int16_t)10041);
    assert(p186_sue_trim_value_input_9_GET(pack) == (int16_t)(int16_t) -873);
    assert(p186_sue_trim_value_input_7_GET(pack) == (int16_t)(int16_t)19365);
    assert(p186_sue_number_of_inputs_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p186_sue_trim_value_input_6_GET(pack) == (int16_t)(int16_t) -10146);
    assert(p186_sue_trim_value_input_8_GET(pack) == (int16_t)(int16_t)21664);
    assert(p186_sue_trim_value_input_4_GET(pack) == (int16_t)(int16_t)32475);
    assert(p186_sue_trim_value_input_11_GET(pack) == (int16_t)(int16_t) -9883);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    assert(p187_sue_accel_z_offset_GET(pack) == (int16_t)(int16_t)925);
    assert(p187_sue_gyro_z_offset_GET(pack) == (int16_t)(int16_t)12157);
    assert(p187_sue_accel_y_offset_GET(pack) == (int16_t)(int16_t) -23238);
    assert(p187_sue_gyro_y_offset_GET(pack) == (int16_t)(int16_t)403);
    assert(p187_sue_accel_x_offset_GET(pack) == (int16_t)(int16_t) -31144);
    assert(p187_sue_gyro_x_offset_GET(pack) == (int16_t)(int16_t)511);
};


void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_sue_accel_y_at_calibration_GET(pack) == (int16_t)(int16_t) -7514);
    assert(p188_sue_accel_z_at_calibration_GET(pack) == (int16_t)(int16_t) -3519);
    assert(p188_sue_gyro_y_at_calibration_GET(pack) == (int16_t)(int16_t)12377);
    assert(p188_sue_gyro_z_at_calibration_GET(pack) == (int16_t)(int16_t)22426);
    assert(p188_sue_accel_x_at_calibration_GET(pack) == (int16_t)(int16_t)158);
    assert(p188_sue_gyro_x_at_calibration_GET(pack) == (int16_t)(int16_t)4745);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_hagl_ratio_GET(pack) == (float) -1.4043517E38F);
    assert(p230_vel_ratio_GET(pack) == (float)1.9546217E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -2.6104972E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE));
    assert(p230_time_usec_GET(pack) == (uint64_t)6421752012664311492L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -7.858485E36F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.4211544E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)1.8176619E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -3.208412E38F);
    assert(p230_tas_ratio_GET(pack) == (float)9.271696E37F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_time_usec_GET(pack) == (uint64_t)4196986020672640699L);
    assert(p231_horiz_accuracy_GET(pack) == (float)1.1121925E38F);
    assert(p231_var_horiz_GET(pack) == (float) -3.1657223E38F);
    assert(p231_wind_y_GET(pack) == (float)3.8438018E37F);
    assert(p231_vert_accuracy_GET(pack) == (float)9.319203E37F);
    assert(p231_wind_z_GET(pack) == (float)1.6857309E38F);
    assert(p231_var_vert_GET(pack) == (float)2.1938747E37F);
    assert(p231_wind_x_GET(pack) == (float)3.0694056E38F);
    assert(p231_wind_alt_GET(pack) == (float) -1.7783065E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_ve_GET(pack) == (float)3.260631E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)1472989669L);
    assert(p232_vdop_GET(pack) == (float) -1.5335549E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)1.921051E38F);
    assert(p232_hdop_GET(pack) == (float) -3.350911E38F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p232_time_usec_GET(pack) == (uint64_t)3511799328468511635L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p232_alt_GET(pack) == (float)3.2669133E38F);
    assert(p232_vn_GET(pack) == (float)6.191475E37F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)49682);
    assert(p232_lon_GET(pack) == (int32_t)1564008716);
    assert(p232_speed_accuracy_GET(pack) == (float) -2.3760269E38F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT));
    assert(p232_lat_GET(pack) == (int32_t)905536750);
    assert(p232_vd_GET(pack) == (float)1.053728E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -1.1659512E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)104);
    {
        uint8_t exemplary[] =  {(uint8_t)158, (uint8_t)178, (uint8_t)173, (uint8_t)85, (uint8_t)63, (uint8_t)223, (uint8_t)159, (uint8_t)2, (uint8_t)21, (uint8_t)72, (uint8_t)152, (uint8_t)18, (uint8_t)220, (uint8_t)232, (uint8_t)46, (uint8_t)2, (uint8_t)43, (uint8_t)149, (uint8_t)109, (uint8_t)213, (uint8_t)26, (uint8_t)158, (uint8_t)154, (uint8_t)50, (uint8_t)180, (uint8_t)85, (uint8_t)251, (uint8_t)60, (uint8_t)175, (uint8_t)134, (uint8_t)243, (uint8_t)137, (uint8_t)9, (uint8_t)23, (uint8_t)77, (uint8_t)129, (uint8_t)227, (uint8_t)15, (uint8_t)69, (uint8_t)51, (uint8_t)110, (uint8_t)15, (uint8_t)53, (uint8_t)105, (uint8_t)35, (uint8_t)71, (uint8_t)6, (uint8_t)236, (uint8_t)47, (uint8_t)206, (uint8_t)89, (uint8_t)74, (uint8_t)115, (uint8_t)128, (uint8_t)231, (uint8_t)178, (uint8_t)30, (uint8_t)16, (uint8_t)253, (uint8_t)159, (uint8_t)254, (uint8_t)41, (uint8_t)80, (uint8_t)120, (uint8_t)184, (uint8_t)145, (uint8_t)114, (uint8_t)186, (uint8_t)215, (uint8_t)222, (uint8_t)67, (uint8_t)112, (uint8_t)125, (uint8_t)247, (uint8_t)251, (uint8_t)36, (uint8_t)71, (uint8_t)145, (uint8_t)27, (uint8_t)181, (uint8_t)142, (uint8_t)150, (uint8_t)47, (uint8_t)206, (uint8_t)44, (uint8_t)186, (uint8_t)47, (uint8_t)184, (uint8_t)208, (uint8_t)169, (uint8_t)44, (uint8_t)14, (uint8_t)237, (uint8_t)233, (uint8_t)62, (uint8_t)98, (uint8_t)162, (uint8_t)84, (uint8_t)96, (uint8_t)187, (uint8_t)141, (uint8_t)211, (uint8_t)131, (uint8_t)247, (uint8_t)132, (uint8_t)218, (uint8_t)160, (uint8_t)183, (uint8_t)114, (uint8_t)175, (uint8_t)143, (uint8_t)89, (uint8_t)19, (uint8_t)9, (uint8_t)52, (uint8_t)208, (uint8_t)22, (uint8_t)18, (uint8_t)144, (uint8_t)94, (uint8_t)56, (uint8_t)84, (uint8_t)8, (uint8_t)195, (uint8_t)216, (uint8_t)63, (uint8_t)218, (uint8_t)236, (uint8_t)185, (uint8_t)15, (uint8_t)43, (uint8_t)1, (uint8_t)176, (uint8_t)108, (uint8_t)222, (uint8_t)232, (uint8_t)250, (uint8_t)70, (uint8_t)194, (uint8_t)31, (uint8_t)251, (uint8_t)198, (uint8_t)122, (uint8_t)37, (uint8_t)30, (uint8_t)145, (uint8_t)219, (uint8_t)195, (uint8_t)222, (uint8_t)10, (uint8_t)46, (uint8_t)210, (uint8_t)166, (uint8_t)43, (uint8_t)91, (uint8_t)226, (uint8_t)52, (uint8_t)223, (uint8_t)47, (uint8_t)176, (uint8_t)161, (uint8_t)242, (uint8_t)243, (uint8_t)126, (uint8_t)23, (uint8_t)25, (uint8_t)95, (uint8_t)120, (uint8_t)147, (uint8_t)188, (uint8_t)159, (uint8_t)70, (uint8_t)147, (uint8_t)186, (uint8_t)191, (uint8_t)108, (uint8_t)153, (uint8_t)173, (uint8_t)95, (uint8_t)187} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)41);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)23);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p234_custom_mode_GET(pack) == (uint32_t)4284106940L);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)54767);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)28219);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED));
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -15095);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)28650);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p234_latitude_GET(pack) == (int32_t)1651012546);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -5259);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -3704);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)55160);
    assert(p234_longitude_GET(pack) == (int32_t) -536270369);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)98);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -23);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -125);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)160);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_x_GET(pack) == (float) -1.2192096E38F);
    assert(p241_vibration_y_GET(pack) == (float) -7.9398083E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)2644374815L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3589554316L);
    assert(p241_time_usec_GET(pack) == (uint64_t)299174393081430906L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)4105791902L);
    assert(p241_vibration_z_GET(pack) == (float) -2.893432E38F);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_altitude_GET(pack) == (int32_t)463939088);
    assert(p242_y_GET(pack) == (float)1.2897297E38F);
    {
        float exemplary[] =  {-2.5352432E38F, -5.816463E37F, 2.6609351E38F, -3.7874018E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_latitude_GET(pack) == (int32_t) -719201426);
    assert(p242_time_usec_TRY(ph) == (uint64_t)6798795451552831650L);
    assert(p242_z_GET(pack) == (float)2.4910103E38F);
    assert(p242_longitude_GET(pack) == (int32_t)43234928);
    assert(p242_approach_x_GET(pack) == (float) -1.2967198E38F);
    assert(p242_approach_z_GET(pack) == (float)1.4386479E38F);
    assert(p242_x_GET(pack) == (float) -1.4318507E38F);
    assert(p242_approach_y_GET(pack) == (float) -1.2114207E37F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_altitude_GET(pack) == (int32_t) -1546040692);
    assert(p243_approach_z_GET(pack) == (float) -2.7425142E38F);
    assert(p243_z_GET(pack) == (float)2.7407932E38F);
    {
        float exemplary[] =  {1.8149595E38F, -1.57721E37F, -5.2640855E36F, -3.037708E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_latitude_GET(pack) == (int32_t)2009353739);
    assert(p243_longitude_GET(pack) == (int32_t) -577430565);
    assert(p243_x_GET(pack) == (float)1.7123708E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p243_approach_x_GET(pack) == (float)8.530378E36F);
    assert(p243_y_GET(pack) == (float) -1.4421474E37F);
    assert(p243_approach_y_GET(pack) == (float) -2.0279775E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)3181728163473809569L);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)63861);
    assert(p244_interval_us_GET(pack) == (int32_t) -1304391521);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_callsign_LEN(ph) == 9);
    {
        char16_t * exemplary = u"vmqxvxalg";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lat_GET(pack) == (int32_t)873098128);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3986461236L);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)28020);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY));
    assert(p246_lon_GET(pack) == (int32_t) -507945323);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)54636);
    assert(p246_altitude_GET(pack) == (int32_t)397460727);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -16983);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)35019);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -1.7937818E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT);
    assert(p247_id_GET(pack) == (uint32_t)3578437248L);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -3.1390507E37F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)2.4008035E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_threat_level_GET(pack) == (e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE));
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)19953);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)38);
    {
        uint8_t exemplary[] =  {(uint8_t)87, (uint8_t)100, (uint8_t)60, (uint8_t)120, (uint8_t)39, (uint8_t)21, (uint8_t)208, (uint8_t)64, (uint8_t)159, (uint8_t)173, (uint8_t)86, (uint8_t)49, (uint8_t)134, (uint8_t)215, (uint8_t)239, (uint8_t)153, (uint8_t)61, (uint8_t)114, (uint8_t)177, (uint8_t)90, (uint8_t)223, (uint8_t)152, (uint8_t)177, (uint8_t)76, (uint8_t)133, (uint8_t)135, (uint8_t)32, (uint8_t)63, (uint8_t)235, (uint8_t)152, (uint8_t)213, (uint8_t)240, (uint8_t)133, (uint8_t)38, (uint8_t)213, (uint8_t)242, (uint8_t)99, (uint8_t)87, (uint8_t)16, (uint8_t)113, (uint8_t)197, (uint8_t)181, (uint8_t)188, (uint8_t)198, (uint8_t)67, (uint8_t)181, (uint8_t)160, (uint8_t)179, (uint8_t)8, (uint8_t)122, (uint8_t)246, (uint8_t)106, (uint8_t)73, (uint8_t)95, (uint8_t)49, (uint8_t)90, (uint8_t)177, (uint8_t)201, (uint8_t)45, (uint8_t)201, (uint8_t)138, (uint8_t)73, (uint8_t)86, (uint8_t)226, (uint8_t)154, (uint8_t)53, (uint8_t)247, (uint8_t)4, (uint8_t)61, (uint8_t)202, (uint8_t)132, (uint8_t)105, (uint8_t)143, (uint8_t)220, (uint8_t)142, (uint8_t)233, (uint8_t)113, (uint8_t)175, (uint8_t)86, (uint8_t)63, (uint8_t)29, (uint8_t)63, (uint8_t)115, (uint8_t)76, (uint8_t)28, (uint8_t)5, (uint8_t)105, (uint8_t)146, (uint8_t)230, (uint8_t)180, (uint8_t)5, (uint8_t)181, (uint8_t)157, (uint8_t)101, (uint8_t)219, (uint8_t)137, (uint8_t)105, (uint8_t)75, (uint8_t)39, (uint8_t)19, (uint8_t)253, (uint8_t)248, (uint8_t)150, (uint8_t)92, (uint8_t)193, (uint8_t)102, (uint8_t)112, (uint8_t)33, (uint8_t)169, (uint8_t)218, (uint8_t)131, (uint8_t)207, (uint8_t)200, (uint8_t)195, (uint8_t)254, (uint8_t)205, (uint8_t)102, (uint8_t)50, (uint8_t)170, (uint8_t)233, (uint8_t)75, (uint8_t)254, (uint8_t)148, (uint8_t)216, (uint8_t)253, (uint8_t)206, (uint8_t)70, (uint8_t)74, (uint8_t)7, (uint8_t)136, (uint8_t)252, (uint8_t)252, (uint8_t)193, (uint8_t)174, (uint8_t)147, (uint8_t)50, (uint8_t)170, (uint8_t)146, (uint8_t)174, (uint8_t)58, (uint8_t)163, (uint8_t)150, (uint8_t)174, (uint8_t)150, (uint8_t)196, (uint8_t)6, (uint8_t)77, (uint8_t)33, (uint8_t)6, (uint8_t)123, (uint8_t)27, (uint8_t)31, (uint8_t)223, (uint8_t)109, (uint8_t)116, (uint8_t)221, (uint8_t)21, (uint8_t)168, (uint8_t)36, (uint8_t)102, (uint8_t)56, (uint8_t)29, (uint8_t)107, (uint8_t)237, (uint8_t)89, (uint8_t)31, (uint8_t)234, (uint8_t)34, (uint8_t)254, (uint8_t)227, (uint8_t)83, (uint8_t)17, (uint8_t)3, (uint8_t)142, (uint8_t)158, (uint8_t)70, (uint8_t)150, (uint8_t)86, (uint8_t)161, (uint8_t)81, (uint8_t)201, (uint8_t)74, (uint8_t)112, (uint8_t)151, (uint8_t)165, (uint8_t)42, (uint8_t)115, (uint8_t)84, (uint8_t)203, (uint8_t)120, (uint8_t)110, (uint8_t)32, (uint8_t)159, (uint8_t)90, (uint8_t)37, (uint8_t)30, (uint8_t)210, (uint8_t)15, (uint8_t)115, (uint8_t)168, (uint8_t)17, (uint8_t)131, (uint8_t)88, (uint8_t)216, (uint8_t)196, (uint8_t)18, (uint8_t)196, (uint8_t)231, (uint8_t)17, (uint8_t)32, (uint8_t)47, (uint8_t)114, (uint8_t)204, (uint8_t)73, (uint8_t)164, (uint8_t)85, (uint8_t)168, (uint8_t)12, (uint8_t)222, (uint8_t)193, (uint8_t)92, (uint8_t)159, (uint8_t)67, (uint8_t)25, (uint8_t)35, (uint8_t)164, (uint8_t)238, (uint8_t)139, (uint8_t)71, (uint8_t)172, (uint8_t)253, (uint8_t)86, (uint8_t)19, (uint8_t)54, (uint8_t)50, (uint8_t)174, (uint8_t)85, (uint8_t)207, (uint8_t)203, (uint8_t)17, (uint8_t)93, (uint8_t)198, (uint8_t)9, (uint8_t)23, (uint8_t)68, (uint8_t)220, (uint8_t)184, (uint8_t)245, (uint8_t)1} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)48000);
    {
        int8_t exemplary[] =  {(int8_t)87, (int8_t) -105, (int8_t)105, (int8_t)99, (int8_t)127, (int8_t)34, (int8_t)13, (int8_t) -35, (int8_t) -30, (int8_t)120, (int8_t) -10, (int8_t) -13, (int8_t) -59, (int8_t)126, (int8_t)108, (int8_t)25, (int8_t)0, (int8_t) -75, (int8_t) -20, (int8_t)34, (int8_t)22, (int8_t)51, (int8_t)93, (int8_t) -86, (int8_t)113, (int8_t)39, (int8_t)1, (int8_t) -24, (int8_t)27, (int8_t)92, (int8_t) -66, (int8_t)69} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float)8.510319E37F);
    assert(p250_time_usec_GET(pack) == (uint64_t)8168709550551217930L);
    assert(p250_x_GET(pack) == (float)3.2018215E38F);
    assert(p250_z_GET(pack) == (float)6.7169626E37F);
    assert(p250_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"ssm";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"h";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2032895809L);
    assert(p251_value_GET(pack) == (float) -2.5963955E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3144508605L);
    assert(p252_value_GET(pack) == (int32_t)1763485774);
    assert(p252_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"dpRuf";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 6);
    {
        char16_t * exemplary = u"Grbpme";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)4010610800L);
    assert(p254_value_GET(pack) == (float) -3.49838E37F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)228, (uint8_t)28, (uint8_t)211, (uint8_t)66, (uint8_t)129, (uint8_t)171, (uint8_t)26, (uint8_t)120, (uint8_t)137, (uint8_t)159, (uint8_t)229, (uint8_t)102, (uint8_t)63, (uint8_t)97, (uint8_t)217, (uint8_t)223, (uint8_t)204, (uint8_t)171, (uint8_t)214, (uint8_t)31, (uint8_t)187, (uint8_t)206, (uint8_t)239, (uint8_t)140, (uint8_t)112, (uint8_t)247, (uint8_t)99, (uint8_t)199, (uint8_t)4, (uint8_t)225, (uint8_t)237, (uint8_t)225} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)3575242703178803148L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)80);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3414827530L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)678527081L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 28);
    {
        char16_t * exemplary = u"dfvmctqzJbtbqXsoyfwynbfnymxu";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)184);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3464399766L);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)21589);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
    assert(p259_cam_definition_uri_LEN(ph) == 65);
    {
        char16_t * exemplary = u"leTmlyjoiTdghnsfuehHzqotrznarzxpxpirbsvzwzdreiphdqdmzgecsxxgqauzk";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 130);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float) -1.8936035E38F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)15781);
    assert(p259_sensor_size_h_GET(pack) == (float)7.5836584E37F);
    {
        uint8_t exemplary[] =  {(uint8_t)112, (uint8_t)253, (uint8_t)56, (uint8_t)157, (uint8_t)124, (uint8_t)141, (uint8_t)71, (uint8_t)95, (uint8_t)32, (uint8_t)193, (uint8_t)48, (uint8_t)162, (uint8_t)36, (uint8_t)152, (uint8_t)51, (uint8_t)196, (uint8_t)245, (uint8_t)88, (uint8_t)174, (uint8_t)3, (uint8_t)230, (uint8_t)146, (uint8_t)207, (uint8_t)249, (uint8_t)31, (uint8_t)53, (uint8_t)1, (uint8_t)42, (uint8_t)128, (uint8_t)10, (uint8_t)102, (uint8_t)190} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)200, (uint8_t)47, (uint8_t)184, (uint8_t)248, (uint8_t)155, (uint8_t)235, (uint8_t)211, (uint8_t)30, (uint8_t)238, (uint8_t)179, (uint8_t)174, (uint8_t)126, (uint8_t)6, (uint8_t)203, (uint8_t)232, (uint8_t)178, (uint8_t)56, (uint8_t)185, (uint8_t)167, (uint8_t)127, (uint8_t)58, (uint8_t)83, (uint8_t)135, (uint8_t)43, (uint8_t)170, (uint8_t)165, (uint8_t)56, (uint8_t)225, (uint8_t)177, (uint8_t)165, (uint8_t)250, (uint8_t)63} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float) -1.6800413E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)47768);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1921863614L);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3026728645L);
    assert(p260_mode_id_GET(pack) == (e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY));
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_available_capacity_GET(pack) == (float)7.8911336E37F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1591058872L);
    assert(p261_used_capacity_GET(pack) == (float)1.4094225E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p261_write_speed_GET(pack) == (float) -8.4147494E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p261_read_speed_GET(pack) == (float)5.862127E37F);
    assert(p261_total_capacity_GET(pack) == (float) -8.4666125E37F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)349962624L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p262_image_interval_GET(pack) == (float)9.340211E37F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)1323995604L);
    assert(p262_available_capacity_GET(pack) == (float)5.325937E37F);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_alt_GET(pack) == (int32_t)587152532);
    assert(p263_lat_GET(pack) == (int32_t) -1491043280);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)93);
    assert(p263_file_url_LEN(ph) == 17);
    {
        char16_t * exemplary = u"nqCfurpbafiSpqwUl";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 34);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.1414722E38F, 6.3983604E37F, 1.6361245E38F, 2.0770108E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t)1040048299);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p263_time_utc_GET(pack) == (uint64_t)2573210771851268416L);
    assert(p263_lon_GET(pack) == (int32_t)763489430);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)801565525L);
    assert(p263_image_index_GET(pack) == (int32_t)347978819);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)3719202905706038695L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)6087857316651890716L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)856793907L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)2845270483614826946L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1549602044L);
    assert(p265_yaw_GET(pack) == (float)1.1062974E38F);
    assert(p265_pitch_GET(pack) == (float) -2.4678358E38F);
    assert(p265_roll_GET(pack) == (float)1.455532E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)57);
    {
        uint8_t exemplary[] =  {(uint8_t)129, (uint8_t)33, (uint8_t)226, (uint8_t)129, (uint8_t)216, (uint8_t)13, (uint8_t)232, (uint8_t)13, (uint8_t)109, (uint8_t)65, (uint8_t)171, (uint8_t)169, (uint8_t)13, (uint8_t)247, (uint8_t)39, (uint8_t)237, (uint8_t)153, (uint8_t)192, (uint8_t)68, (uint8_t)159, (uint8_t)193, (uint8_t)0, (uint8_t)106, (uint8_t)185, (uint8_t)175, (uint8_t)88, (uint8_t)149, (uint8_t)206, (uint8_t)151, (uint8_t)72, (uint8_t)178, (uint8_t)103, (uint8_t)237, (uint8_t)10, (uint8_t)134, (uint8_t)23, (uint8_t)160, (uint8_t)59, (uint8_t)9, (uint8_t)67, (uint8_t)10, (uint8_t)110, (uint8_t)60, (uint8_t)120, (uint8_t)162, (uint8_t)13, (uint8_t)111, (uint8_t)137, (uint8_t)182, (uint8_t)252, (uint8_t)155, (uint8_t)185, (uint8_t)68, (uint8_t)247, (uint8_t)177, (uint8_t)6, (uint8_t)140, (uint8_t)144, (uint8_t)185, (uint8_t)109, (uint8_t)244, (uint8_t)153, (uint8_t)69, (uint8_t)134, (uint8_t)216, (uint8_t)186, (uint8_t)87, (uint8_t)163, (uint8_t)146, (uint8_t)37, (uint8_t)49, (uint8_t)107, (uint8_t)4, (uint8_t)93, (uint8_t)174, (uint8_t)36, (uint8_t)242, (uint8_t)165, (uint8_t)8, (uint8_t)202, (uint8_t)143, (uint8_t)25, (uint8_t)154, (uint8_t)156, (uint8_t)115, (uint8_t)47, (uint8_t)208, (uint8_t)231, (uint8_t)3, (uint8_t)83, (uint8_t)5, (uint8_t)130, (uint8_t)226, (uint8_t)2, (uint8_t)238, (uint8_t)41, (uint8_t)20, (uint8_t)46, (uint8_t)100, (uint8_t)84, (uint8_t)35, (uint8_t)153, (uint8_t)207, (uint8_t)82, (uint8_t)105, (uint8_t)59, (uint8_t)114, (uint8_t)213, (uint8_t)146, (uint8_t)20, (uint8_t)240, (uint8_t)45, (uint8_t)220, (uint8_t)106, (uint8_t)120, (uint8_t)90, (uint8_t)195, (uint8_t)38, (uint8_t)168, (uint8_t)169, (uint8_t)109, (uint8_t)47, (uint8_t)135, (uint8_t)248, (uint8_t)230, (uint8_t)233, (uint8_t)198, (uint8_t)175, (uint8_t)90, (uint8_t)180, (uint8_t)86, (uint8_t)103, (uint8_t)102, (uint8_t)222, (uint8_t)126, (uint8_t)2, (uint8_t)45, (uint8_t)148, (uint8_t)100, (uint8_t)213, (uint8_t)5, (uint8_t)238, (uint8_t)32, (uint8_t)171, (uint8_t)138, (uint8_t)65, (uint8_t)191, (uint8_t)220, (uint8_t)201, (uint8_t)55, (uint8_t)66, (uint8_t)103, (uint8_t)158, (uint8_t)249, (uint8_t)110, (uint8_t)167, (uint8_t)55, (uint8_t)227, (uint8_t)150, (uint8_t)15, (uint8_t)96, (uint8_t)200, (uint8_t)61, (uint8_t)129, (uint8_t)237, (uint8_t)29, (uint8_t)195, (uint8_t)79, (uint8_t)249, (uint8_t)132, (uint8_t)195, (uint8_t)200, (uint8_t)9, (uint8_t)63, (uint8_t)74, (uint8_t)196, (uint8_t)77, (uint8_t)6, (uint8_t)255, (uint8_t)223, (uint8_t)124, (uint8_t)122, (uint8_t)189, (uint8_t)110, (uint8_t)41, (uint8_t)188, (uint8_t)197, (uint8_t)79, (uint8_t)2, (uint8_t)73, (uint8_t)234, (uint8_t)34, (uint8_t)203, (uint8_t)147, (uint8_t)31, (uint8_t)157, (uint8_t)92, (uint8_t)121, (uint8_t)44, (uint8_t)232, (uint8_t)36, (uint8_t)217, (uint8_t)138, (uint8_t)240, (uint8_t)27, (uint8_t)114, (uint8_t)83, (uint8_t)90, (uint8_t)233, (uint8_t)171, (uint8_t)95, (uint8_t)194, (uint8_t)151, (uint8_t)198, (uint8_t)139, (uint8_t)128, (uint8_t)120, (uint8_t)85, (uint8_t)56, (uint8_t)71, (uint8_t)1, (uint8_t)175, (uint8_t)214, (uint8_t)107, (uint8_t)131, (uint8_t)5, (uint8_t)192, (uint8_t)187, (uint8_t)165, (uint8_t)207, (uint8_t)170, (uint8_t)84, (uint8_t)51, (uint8_t)200, (uint8_t)195, (uint8_t)229, (uint8_t)225, (uint8_t)123, (uint8_t)61, (uint8_t)189, (uint8_t)77, (uint8_t)114, (uint8_t)108, (uint8_t)42, (uint8_t)245, (uint8_t)129, (uint8_t)125, (uint8_t)117, (uint8_t)51} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)22749);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)56728);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)120);
    {
        uint8_t exemplary[] =  {(uint8_t)8, (uint8_t)178, (uint8_t)249, (uint8_t)103, (uint8_t)64, (uint8_t)36, (uint8_t)157, (uint8_t)89, (uint8_t)245, (uint8_t)251, (uint8_t)0, (uint8_t)8, (uint8_t)219, (uint8_t)94, (uint8_t)119, (uint8_t)18, (uint8_t)229, (uint8_t)227, (uint8_t)176, (uint8_t)133, (uint8_t)77, (uint8_t)98, (uint8_t)54, (uint8_t)37, (uint8_t)138, (uint8_t)36, (uint8_t)181, (uint8_t)13, (uint8_t)115, (uint8_t)7, (uint8_t)26, (uint8_t)60, (uint8_t)83, (uint8_t)36, (uint8_t)3, (uint8_t)206, (uint8_t)93, (uint8_t)67, (uint8_t)174, (uint8_t)60, (uint8_t)180, (uint8_t)186, (uint8_t)70, (uint8_t)86, (uint8_t)22, (uint8_t)38, (uint8_t)170, (uint8_t)226, (uint8_t)143, (uint8_t)78, (uint8_t)186, (uint8_t)22, (uint8_t)135, (uint8_t)207, (uint8_t)187, (uint8_t)245, (uint8_t)201, (uint8_t)251, (uint8_t)142, (uint8_t)210, (uint8_t)70, (uint8_t)156, (uint8_t)81, (uint8_t)16, (uint8_t)17, (uint8_t)90, (uint8_t)148, (uint8_t)179, (uint8_t)147, (uint8_t)245, (uint8_t)164, (uint8_t)46, (uint8_t)7, (uint8_t)102, (uint8_t)23, (uint8_t)158, (uint8_t)56, (uint8_t)121, (uint8_t)109, (uint8_t)68, (uint8_t)112, (uint8_t)171, (uint8_t)124, (uint8_t)150, (uint8_t)149, (uint8_t)172, (uint8_t)52, (uint8_t)17, (uint8_t)189, (uint8_t)37, (uint8_t)196, (uint8_t)198, (uint8_t)240, (uint8_t)253, (uint8_t)132, (uint8_t)119, (uint8_t)48, (uint8_t)100, (uint8_t)56, (uint8_t)214, (uint8_t)76, (uint8_t)15, (uint8_t)135, (uint8_t)174, (uint8_t)230, (uint8_t)18, (uint8_t)44, (uint8_t)7, (uint8_t)169, (uint8_t)142, (uint8_t)151, (uint8_t)10, (uint8_t)135, (uint8_t)93, (uint8_t)101, (uint8_t)218, (uint8_t)67, (uint8_t)91, (uint8_t)100, (uint8_t)111, (uint8_t)169, (uint8_t)114, (uint8_t)139, (uint8_t)85, (uint8_t)192, (uint8_t)147, (uint8_t)36, (uint8_t)187, (uint8_t)60, (uint8_t)227, (uint8_t)139, (uint8_t)232, (uint8_t)220, (uint8_t)245, (uint8_t)113, (uint8_t)141, (uint8_t)245, (uint8_t)114, (uint8_t)1, (uint8_t)181, (uint8_t)233, (uint8_t)188, (uint8_t)35, (uint8_t)110, (uint8_t)37, (uint8_t)99, (uint8_t)14, (uint8_t)237, (uint8_t)180, (uint8_t)112, (uint8_t)65, (uint8_t)94, (uint8_t)189, (uint8_t)169, (uint8_t)127, (uint8_t)187, (uint8_t)172, (uint8_t)50, (uint8_t)45, (uint8_t)155, (uint8_t)180, (uint8_t)2, (uint8_t)144, (uint8_t)85, (uint8_t)217, (uint8_t)214, (uint8_t)250, (uint8_t)209, (uint8_t)35, (uint8_t)82, (uint8_t)214, (uint8_t)171, (uint8_t)215, (uint8_t)67, (uint8_t)136, (uint8_t)201, (uint8_t)168, (uint8_t)53, (uint8_t)72, (uint8_t)101, (uint8_t)153, (uint8_t)223, (uint8_t)254, (uint8_t)133, (uint8_t)228, (uint8_t)155, (uint8_t)174, (uint8_t)95, (uint8_t)208, (uint8_t)106, (uint8_t)162, (uint8_t)196, (uint8_t)117, (uint8_t)96, (uint8_t)10, (uint8_t)210, (uint8_t)64, (uint8_t)175, (uint8_t)210, (uint8_t)217, (uint8_t)248, (uint8_t)159, (uint8_t)35, (uint8_t)140, (uint8_t)3, (uint8_t)177, (uint8_t)1, (uint8_t)2, (uint8_t)93, (uint8_t)201, (uint8_t)94, (uint8_t)248, (uint8_t)1, (uint8_t)16, (uint8_t)152, (uint8_t)142, (uint8_t)164, (uint8_t)146, (uint8_t)185, (uint8_t)212, (uint8_t)169, (uint8_t)85, (uint8_t)221, (uint8_t)126, (uint8_t)2, (uint8_t)29, (uint8_t)175, (uint8_t)189, (uint8_t)173, (uint8_t)247, (uint8_t)137, (uint8_t)184, (uint8_t)41, (uint8_t)230, (uint8_t)207, (uint8_t)89, (uint8_t)42, (uint8_t)116, (uint8_t)144, (uint8_t)219, (uint8_t)59, (uint8_t)83, (uint8_t)227, (uint8_t)215, (uint8_t)61, (uint8_t)143, (uint8_t)17, (uint8_t)182, (uint8_t)122} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)9052);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)56799);
    assert(p269_uri_LEN(ph) == 189);
    {
        char16_t * exemplary = u"zajlrhpsxSwbdknhvpyvxnzfuwvyqbjrNqknbMdBjSdrqkhnnekfIYkwueUownuuvwkkNrxxjdgkOxzxqpnddaopdEktqegzhAwyauzVfmzroslqRTjjhgcHvcYwzvdlsvuuuUebdlzhxhddlpoqnrzhazareaqmdxErhthqtzlrjslythjgjisdmbail";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 378);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_bitrate_GET(pack) == (uint32_t)342645355L);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)58392);
    assert(p269_framerate_GET(pack) == (float)1.6167486E38F);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)62567);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p270_uri_LEN(ph) == 12);
    {
        char16_t * exemplary = u"hipWNuysdtzj";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)33842);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)26669);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p270_bitrate_GET(pack) == (uint32_t)3167113745L);
    assert(p270_framerate_GET(pack) == (float)6.7623196E37F);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)59881);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 34);
    {
        char16_t * exemplary = u"cihpflhwrvdSvbmcssfpagdfGcrpnfcxjy";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 68);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 23);
    {
        char16_t * exemplary = u"kdVvmjwsoiivlxGHzmwablO";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 46);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)5085);
    {
        uint8_t exemplary[] =  {(uint8_t)0, (uint8_t)186, (uint8_t)148, (uint8_t)60, (uint8_t)144, (uint8_t)235, (uint8_t)164, (uint8_t)163} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)37967);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)10288);
    {
        uint8_t exemplary[] =  {(uint8_t)228, (uint8_t)90, (uint8_t)165, (uint8_t)30, (uint8_t)7, (uint8_t)158, (uint8_t)147, (uint8_t)227} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p310_time_usec_GET(pack) == (uint64_t)5947519773673961516L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)49115);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)420361618L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)198);
    {
        uint8_t exemplary[] =  {(uint8_t)135, (uint8_t)27, (uint8_t)239, (uint8_t)70, (uint8_t)253, (uint8_t)248, (uint8_t)190, (uint8_t)127, (uint8_t)189, (uint8_t)222, (uint8_t)234, (uint8_t)225, (uint8_t)243, (uint8_t)156, (uint8_t)168, (uint8_t)197} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2930772623L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p311_name_LEN(ph) == 75);
    {
        char16_t * exemplary = u"ehgmrysslhfVwrjtsidbLmdbmffukcxwsDddmZmxcduMfdGhzkgylwtliXPeiUwelovuogdambc";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 150);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p311_time_usec_GET(pack) == (uint64_t)6554167624591943630L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)1805612457L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p320_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"ojtozFkyZvc";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)24475);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)42307);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p322_param_value_LEN(ph) == 6);
    {
        char16_t * exemplary = u"ienwnj";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)60662);
    assert(p322_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"sqvmlaDj";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p323_param_value_LEN(ph) == 113);
    {
        char16_t * exemplary = u"gmakoxtmabfhDvqiqbrovkupyssMerzrzubttppuDbkejlXsippnaeMtdMiphhgcykhymxudlsEnUtxyemtpjvqtjAbilkboirfykjcvcytaYxvqp";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 226);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"EFqKlibjrbt";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)9);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p324_param_value_LEN(ph) == 13);
    {
        char16_t * exemplary = u"izbbemuoclEwg";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
    assert(p324_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"odlptsq";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)19932);
    {
        uint16_t exemplary[] =  {(uint16_t)28055, (uint16_t)42202, (uint16_t)1504, (uint16_t)12912, (uint16_t)57723, (uint16_t)27801, (uint16_t)14376, (uint16_t)49039, (uint16_t)47263, (uint16_t)1540, (uint16_t)21711, (uint16_t)21474, (uint16_t)37339, (uint16_t)59446, (uint16_t)46844, (uint16_t)36472, (uint16_t)13912, (uint16_t)13515, (uint16_t)21312, (uint16_t)11463, (uint16_t)21135, (uint16_t)50268, (uint16_t)22053, (uint16_t)23764, (uint16_t)11052, (uint16_t)2415, (uint16_t)5039, (uint16_t)52981, (uint16_t)15271, (uint16_t)52588, (uint16_t)65045, (uint16_t)19325, (uint16_t)49712, (uint16_t)10499, (uint16_t)43459, (uint16_t)16700, (uint16_t)25220, (uint16_t)52239, (uint16_t)36665, (uint16_t)63704, (uint16_t)20964, (uint16_t)64636, (uint16_t)27289, (uint16_t)1491, (uint16_t)61145, (uint16_t)60927, (uint16_t)47733, (uint16_t)64704, (uint16_t)43039, (uint16_t)42249, (uint16_t)29042, (uint16_t)911, (uint16_t)58329, (uint16_t)42570, (uint16_t)41032, (uint16_t)32598, (uint16_t)5829, (uint16_t)45083, (uint16_t)65464, (uint16_t)480, (uint16_t)16834, (uint16_t)38004, (uint16_t)49565, (uint16_t)26910, (uint16_t)45423, (uint16_t)17804, (uint16_t)56512, (uint16_t)62908, (uint16_t)47607, (uint16_t)33836, (uint16_t)34148, (uint16_t)44924} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)6446429035326580663L);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)49265);
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
        p0_custom_mode_SET((uint32_t)3408125283L, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_BOOT, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED), PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)34449, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)53221, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL), PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -85, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)17747, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)60399, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)39716, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)2312, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)6787, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)2311, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)24211, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2210121159848586267L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)2939771029L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_z_SET((float) -1.2099859E38F, PH.base.pack) ;
        p3_yaw_SET((float)1.0427875E38F, PH.base.pack) ;
        p3_y_SET((float)1.1768226E38F, PH.base.pack) ;
        p3_vx_SET((float)3.4970748E37F, PH.base.pack) ;
        p3_afz_SET((float) -2.5463553E38F, PH.base.pack) ;
        p3_x_SET((float) -3.0771308E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)3686, PH.base.pack) ;
        p3_vy_SET((float) -2.9346152E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)3.740571E37F, PH.base.pack) ;
        p3_vz_SET((float) -1.0083937E38F, PH.base.pack) ;
        p3_afx_SET((float) -2.5023027E38F, PH.base.pack) ;
        p3_afy_SET((float) -2.3183098E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)310102113L, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        c_TEST_Channel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)8520959309834784821L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p4_seq_SET((uint32_t)491620138L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        {
            char16_t* passkey = u"ulm";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"uxnmrvhekhsldxzhaiqfKafmuecg";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)3910606134L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t)3301, PH.base.pack) ;
        {
            char16_t* param_id = u"swmUrsYof";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_value_SET((float)2.3896311E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)11646, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)42167, PH.base.pack) ;
        {
            char16_t* param_id = u"hmuckxdkzozup";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        {
            char16_t* param_id = u"piEff";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_value_SET((float)2.6484462E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_v_acc_SET((uint32_t)156293354L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)23862, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1699839210L, &PH) ;
        p24_h_acc_SET((uint32_t)1596762773L, &PH) ;
        p24_hdg_acc_SET((uint32_t)593074329L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p24_alt_SET((int32_t)1621952165, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)1377389755253240027L, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -99271133, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)23586, PH.base.pack) ;
        p24_lon_SET((int32_t)913978713, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)56520, PH.base.pack) ;
        p24_lat_SET((int32_t) -283930369, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)62994, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)53, (uint8_t)250, (uint8_t)93, (uint8_t)198, (uint8_t)253, (uint8_t)92, (uint8_t)175, (uint8_t)36, (uint8_t)116, (uint8_t)177, (uint8_t)63, (uint8_t)66, (uint8_t)169, (uint8_t)81, (uint8_t)153, (uint8_t)203, (uint8_t)238, (uint8_t)78, (uint8_t)73, (uint8_t)86};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)95, (uint8_t)156, (uint8_t)126, (uint8_t)151, (uint8_t)215, (uint8_t)50, (uint8_t)137, (uint8_t)214, (uint8_t)36, (uint8_t)22, (uint8_t)36, (uint8_t)28, (uint8_t)110, (uint8_t)25, (uint8_t)135, (uint8_t)235, (uint8_t)233, (uint8_t)227, (uint8_t)211, (uint8_t)98};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        {
            uint8_t satellite_used[] =  {(uint8_t)238, (uint8_t)164, (uint8_t)11, (uint8_t)124, (uint8_t)29, (uint8_t)202, (uint8_t)34, (uint8_t)32, (uint8_t)30, (uint8_t)87, (uint8_t)11, (uint8_t)133, (uint8_t)58, (uint8_t)10, (uint8_t)203, (uint8_t)221, (uint8_t)25, (uint8_t)84, (uint8_t)133, (uint8_t)194};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)144, (uint8_t)148, (uint8_t)84, (uint8_t)207, (uint8_t)197, (uint8_t)155, (uint8_t)94, (uint8_t)195, (uint8_t)138, (uint8_t)94, (uint8_t)172, (uint8_t)64, (uint8_t)43, (uint8_t)217, (uint8_t)106, (uint8_t)57, (uint8_t)233, (uint8_t)125, (uint8_t)248, (uint8_t)143};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)0, (uint8_t)110, (uint8_t)4, (uint8_t)201, (uint8_t)55, (uint8_t)45, (uint8_t)42, (uint8_t)92, (uint8_t)191, (uint8_t)167, (uint8_t)108, (uint8_t)55, (uint8_t)35, (uint8_t)204, (uint8_t)222, (uint8_t)209, (uint8_t)100, (uint8_t)46, (uint8_t)175, (uint8_t)132};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_zgyro_SET((int16_t)(int16_t) -14703, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3135617430L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)21348, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)8996, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -25304, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)14915, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)23572, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)9253, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -20722, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -15768, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t) -16140, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -32714, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)17389, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)14034, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)12875, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)8785221584954152343L, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)28148, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)15890, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)8897, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -3178, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_abs_SET((int16_t)(int16_t) -5351, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)16344, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)3044431427196297846L, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)25140, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)11795, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float)8.981826E37F, PH.base.pack) ;
        p29_press_abs_SET((float)2.7227048E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -5872, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3809589523L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)2.9606886E38F, PH.base.pack) ;
        p30_yaw_SET((float)7.42736E37F, PH.base.pack) ;
        p30_pitch_SET((float)7.725193E37F, PH.base.pack) ;
        p30_yawspeed_SET((float)1.5972253E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3888653946L, PH.base.pack) ;
        p30_roll_SET((float)7.616329E37F, PH.base.pack) ;
        p30_rollspeed_SET((float)4.3112584E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -3.8435164E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1366300314L, PH.base.pack) ;
        p31_pitchspeed_SET((float)3.1715089E38F, PH.base.pack) ;
        p31_q3_SET((float) -4.7666776E37F, PH.base.pack) ;
        p31_q4_SET((float) -7.867445E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)4.2462834E36F, PH.base.pack) ;
        p31_q1_SET((float)1.1537495E38F, PH.base.pack) ;
        p31_q2_SET((float) -2.6135726E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_x_SET((float)2.9518984E38F, PH.base.pack) ;
        p32_vy_SET((float) -9.588899E37F, PH.base.pack) ;
        p32_vz_SET((float)2.287399E36F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)2741608465L, PH.base.pack) ;
        p32_z_SET((float)2.6041683E38F, PH.base.pack) ;
        p32_vx_SET((float)1.8392336E38F, PH.base.pack) ;
        p32_y_SET((float) -1.6349509E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_time_boot_ms_SET((uint32_t)2182682524L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)48278, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -30655, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -1200391901, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)8372, PH.base.pack) ;
        p33_lon_SET((int32_t) -984172246, PH.base.pack) ;
        p33_lat_SET((int32_t)1214081414, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)15458, PH.base.pack) ;
        p33_alt_SET((int32_t) -774662098, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_port_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)10833, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -22019, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)32284, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)16637, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -736, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)4109088018L, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)22209, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -15300, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -1114, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan3_raw_SET((uint16_t)(uint16_t)48541, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3647044073L, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)25038, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)8, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)51854, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)37051, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)43011, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)4608, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)27887, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo15_raw_SET((uint16_t)(uint16_t)52573, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)49220, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)44043, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)37925, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)30310, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)47833, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)52926, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)9305, &PH) ;
        p36_time_usec_SET((uint32_t)1147717692L, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)19417, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)44683, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)46010, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)42737, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)36856, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)56385, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)35655, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)45495, &PH) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)6916, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -3014, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t)10278, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)17339, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p39_param1_SET((float)3.3997101E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_param2_SET((float)5.8257997E37F, PH.base.pack) ;
        p39_z_SET((float) -2.534054E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, PH.base.pack) ;
        p39_y_SET((float) -1.4387047E38F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)6414, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p39_x_SET((float) -2.3491963E38F, PH.base.pack) ;
        p39_param4_SET((float) -9.842333E37F, PH.base.pack) ;
        p39_param3_SET((float) -2.6828186E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)24764, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)15350, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)30208, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)53744, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)43958, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t) -1027835251, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p48_altitude_SET((int32_t)1218214308, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)8012218089819038089L, &PH) ;
        p48_latitude_SET((int32_t) -2098047369, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_longitude_SET((int32_t) -1022061667, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)4598053584464042712L, &PH) ;
        p49_latitude_SET((int32_t) -1643106871, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1132883023, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_min_SET((float)2.787156E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -5061, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.1994807E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p50_param_value0_SET((float)1.4335475E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            char16_t* param_id = u"oofdlvjchykpr";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_scale_SET((float)6.727706E35F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)38600, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1y_SET((float)4.535394E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p54_p2z_SET((float) -1.631655E38F, PH.base.pack) ;
        p54_p1x_SET((float) -1.3884734E37F, PH.base.pack) ;
        p54_p2y_SET((float) -1.5402268E38F, PH.base.pack) ;
        p54_p1z_SET((float)5.6971996E37F, PH.base.pack) ;
        p54_p2x_SET((float)2.066187E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2y_SET((float)2.3703176E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p55_p1y_SET((float) -2.860838E38F, PH.base.pack) ;
        p55_p1z_SET((float)2.3220122E38F, PH.base.pack) ;
        p55_p1x_SET((float) -1.9920058E38F, PH.base.pack) ;
        p55_p2x_SET((float)2.5565703E38F, PH.base.pack) ;
        p55_p2z_SET((float)2.6287159E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float) -7.196586E36F, PH.base.pack) ;
        p61_pitchspeed_SET((float)2.3827907E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)1476416739481309040L, PH.base.pack) ;
        p61_yawspeed_SET((float) -7.383447E37F, PH.base.pack) ;
        {
            float covariance[] =  {-3.9966747E37F, 2.3834547E38F, 2.9093802E38F, -1.2407778E38F, -6.1389296E37F, 2.7123322E38F, 7.8381504E37F, -4.8769E36F, 2.4767532E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        {
            float q[] =  {1.8776978E37F, 1.2753453E38F, 5.312655E36F, -2.428011E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_wp_dist_SET((uint16_t)(uint16_t)26866, PH.base.pack) ;
        p62_alt_error_SET((float) -2.2710664E38F, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.2246706E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -1.6832221E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)22909, PH.base.pack) ;
        p62_nav_pitch_SET((float) -8.714565E37F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)25068, PH.base.pack) ;
        p62_aspd_error_SET((float)1.3008247E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lat_SET((int32_t) -937771913, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        p63_lon_SET((int32_t)1751785758, PH.base.pack) ;
        p63_alt_SET((int32_t) -1550055601, PH.base.pack) ;
        p63_vz_SET((float)2.2299232E38F, PH.base.pack) ;
        p63_vy_SET((float)3.8559538E37F, PH.base.pack) ;
        {
            float covariance[] =  {9.324785E37F, 2.6157185E38F, -7.0132247E37F, -4.7117356E37F, -9.877655E37F, -2.5082923E38F, 2.3969099E38F, 1.7351109E38F, -3.059852E38F, -1.6860039E38F, -5.168891E37F, 2.8911853E38F, 1.3700216E38F, 2.7587935E38F, -2.8177147E38F, -9.628201E37F, 1.1249308E37F, 3.0308278E37F, 3.310603E38F, -2.2104959E38F, 1.092786E38F, 2.389361E38F, -1.8456376E38F, -5.515554E37F, 2.8615659E38F, -9.113652E37F, 2.3112864E38F, -2.5115604E38F, 1.6472251E38F, 4.1204843E37F, 1.6704361E38F, 1.6287173E38F, -8.255146E37F, -1.9028194E38F, 2.3393182E38F, 4.1643453E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vx_SET((float)7.5062353E37F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)3885578181752161293L, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)699567222, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vy_SET((float)2.4942176E38F, PH.base.pack) ;
        p64_y_SET((float) -2.086658E38F, PH.base.pack) ;
        p64_ax_SET((float) -1.3484916E38F, PH.base.pack) ;
        p64_vx_SET((float) -3.0817086E37F, PH.base.pack) ;
        p64_az_SET((float)2.7949503E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p64_ay_SET((float)1.6585729E37F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)7122628453472856922L, PH.base.pack) ;
        p64_vz_SET((float) -2.9015812E38F, PH.base.pack) ;
        p64_x_SET((float) -2.2921423E38F, PH.base.pack) ;
        {
            float covariance[] =  {-7.248239E37F, 1.9247346E38F, 7.2695766E37F, 6.4051875E37F, 2.421203E38F, 2.0794496E38F, -3.1653463E38F, 2.9187935E38F, -4.772443E37F, 4.4942777E37F, -6.6966285E37F, -1.9004153E38F, 2.6027473E38F, 2.897851E38F, 3.0796484E38F, -2.8835047E38F, 5.0073776E37F, -2.911965E38F, 9.321932E37F, 1.3026383E38F, 1.1992775E37F, 2.5530343E38F, 2.7850799E38F, -1.2500509E38F, 1.4858595E38F, 3.2688362E38F, -1.3239683E38F, 6.4079723E37F, -2.6356097E38F, 1.7542402E38F, -3.1600213E38F, 2.9776768E37F, 8.2389703E37F, 3.3286375E38F, 1.1505628E38F, 6.162478E37F, 1.9231385E38F, -3.300018E38F, 1.1858987E38F, 2.7304259E38F, 1.3303357E38F, -2.6037065E38F, 7.0148083E37F, -1.0175381E38F, 1.090745E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_z_SET((float) -1.7959096E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan5_raw_SET((uint16_t)(uint16_t)44509, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)20739, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)61618, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)15710, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)34219, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)43265, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)61897, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)12282, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)15162, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2075181232L, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)27800, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)48040, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)59087, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)12148, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)37565, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)42112, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)26353, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)46297, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)17865, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)35161, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)27904, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_r_SET((int16_t)(int16_t) -28069, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)4705, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)5271, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)61418, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)20183, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan2_raw_SET((uint16_t)(uint16_t)28942, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)52566, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)25071, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)38945, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)12799, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)22455, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)59382, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)31762, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p73_y_SET((int32_t) -691584949, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)33146, PH.base.pack) ;
        p73_z_SET((float) -2.1508818E38F, PH.base.pack) ;
        p73_param3_SET((float) -1.1841184E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_GO_AROUND, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p73_x_SET((int32_t)1309228102, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_param2_SET((float)1.6653952E38F, PH.base.pack) ;
        p73_param1_SET((float)1.5238137E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p73_param4_SET((float)2.6643535E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_throttle_SET((uint16_t)(uint16_t)35027, PH.base.pack) ;
        p74_airspeed_SET((float) -9.336258E37F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)11399, PH.base.pack) ;
        p74_groundspeed_SET((float) -3.2403275E38F, PH.base.pack) ;
        p74_alt_SET((float) -1.1152988E38F, PH.base.pack) ;
        p74_climb_SET((float)3.2512216E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param1_SET((float)3.832021E37F, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_LOGGING_START, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p75_param3_SET((float) -1.9511516E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p75_z_SET((float) -1.0211023E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -1780211533, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p75_param2_SET((float)6.8993065E37F, PH.base.pack) ;
        p75_param4_SET((float) -9.780052E37F, PH.base.pack) ;
        p75_x_SET((int32_t)54124348, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE, PH.base.pack) ;
        p76_param1_SET((float) -3.4920367E37F, PH.base.pack) ;
        p76_param3_SET((float)2.357917E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p76_param4_SET((float)1.5896953E37F, PH.base.pack) ;
        p76_param2_SET((float) -1.6360408E38F, PH.base.pack) ;
        p76_param5_SET((float)1.6929635E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p76_param7_SET((float)2.7327888E38F, PH.base.pack) ;
        p76_param6_SET((float)1.6724991E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_REPOSITION, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)28, &PH) ;
        p77_result_param2_SET((int32_t) -969857554, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)180, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)243, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float) -1.0362808E38F, PH.base.pack) ;
        p81_pitch_SET((float)1.0201292E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)464306958L, PH.base.pack) ;
        p81_roll_SET((float)3.1842489E38F, PH.base.pack) ;
        p81_yaw_SET((float) -2.5799333E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_time_boot_ms_SET((uint32_t)3434475015L, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.6625863E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -2.7373874E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        {
            float q[] =  {-1.3091588E38F, 1.1677438E38F, 3.2577065E38F, 3.272849E36F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float)2.384996E38F, PH.base.pack) ;
        p82_thrust_SET((float)6.4892165E37F, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_type_mask_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p83_body_roll_rate_SET((float)1.2746782E38F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)9.710877E37F, PH.base.pack) ;
        {
            float q[] =  {7.271168E37F, -7.240945E37F, 1.2080687E38F, -3.1068636E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_thrust_SET((float) -1.1061844E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -2.6974676E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)4214411413L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vy_SET((float)1.4258091E38F, PH.base.pack) ;
        p84_vx_SET((float)2.386658E37F, PH.base.pack) ;
        p84_afx_SET((float) -2.4772392E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p84_yaw_SET((float)5.718891E37F, PH.base.pack) ;
        p84_afz_SET((float) -1.8517879E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)40769, PH.base.pack) ;
        p84_x_SET((float)2.6157173E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2315897290L, PH.base.pack) ;
        p84_vz_SET((float) -1.7253996E38F, PH.base.pack) ;
        p84_y_SET((float)4.403152E37F, PH.base.pack) ;
        p84_afy_SET((float)2.5241232E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p84_yaw_rate_SET((float)8.0418375E37F, PH.base.pack) ;
        p84_z_SET((float) -1.6515804E38F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_vy_SET((float) -1.5734852E38F, PH.base.pack) ;
        p86_afz_SET((float) -2.3566618E37F, PH.base.pack) ;
        p86_yaw_SET((float)1.4180437E36F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p86_yaw_rate_SET((float) -3.3072787E38F, PH.base.pack) ;
        p86_afy_SET((float)2.1797844E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p86_afx_SET((float) -1.2065997E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3447054801L, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -59010296, PH.base.pack) ;
        p86_vz_SET((float)1.1816582E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)10588, PH.base.pack) ;
        p86_lon_int_SET((int32_t)551422805, PH.base.pack) ;
        p86_vx_SET((float) -8.3626915E37F, PH.base.pack) ;
        p86_alt_SET((float)3.390394E37F, PH.base.pack) ;
        c_TEST_Channel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_lon_int_SET((int32_t) -372721867, PH.base.pack) ;
        p87_alt_SET((float)3.4993475E37F, PH.base.pack) ;
        p87_afy_SET((float) -8.2943915E37F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)3316, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p87_afz_SET((float) -6.49064E37F, PH.base.pack) ;
        p87_vx_SET((float)1.4241926E38F, PH.base.pack) ;
        p87_afx_SET((float)2.2615974E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.1998096E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)4280176301L, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1805121297, PH.base.pack) ;
        p87_yaw_SET((float)3.3450141E38F, PH.base.pack) ;
        p87_vy_SET((float)1.817087E38F, PH.base.pack) ;
        p87_vz_SET((float)1.5820491E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_x_SET((float)2.388116E37F, PH.base.pack) ;
        p89_z_SET((float)1.8513128E38F, PH.base.pack) ;
        p89_yaw_SET((float)1.6509772E37F, PH.base.pack) ;
        p89_pitch_SET((float) -1.5158474E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)1031710358L, PH.base.pack) ;
        p89_y_SET((float)5.6132446E37F, PH.base.pack) ;
        p89_roll_SET((float) -2.3735129E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_zacc_SET((int16_t)(int16_t) -7781, PH.base.pack) ;
        p90_pitch_SET((float)2.199528E38F, PH.base.pack) ;
        p90_alt_SET((int32_t) -611291160, PH.base.pack) ;
        p90_pitchspeed_SET((float) -5.2651274E37F, PH.base.pack) ;
        p90_roll_SET((float) -1.2982056E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -15188, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -18988, PH.base.pack) ;
        p90_yawspeed_SET((float) -2.8195632E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -13548, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -21624, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6621160391877624044L, PH.base.pack) ;
        p90_yaw_SET((float)3.3412317E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -3403, PH.base.pack) ;
        p90_lat_SET((int32_t) -458507348, PH.base.pack) ;
        p90_rollspeed_SET((float) -9.668737E37F, PH.base.pack) ;
        p90_lon_SET((int32_t) -1638797615, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_time_usec_SET((uint64_t)6892959032905017374L, PH.base.pack) ;
        p91_yaw_rudder_SET((float)2.2188548E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -8.4173607E37F, PH.base.pack) ;
        p91_aux1_SET((float)1.056376E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p91_aux3_SET((float)2.0460816E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p91_aux4_SET((float)3.363948E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)2.6149948E38F, PH.base.pack) ;
        p91_throttle_SET((float) -3.2100757E38F, PH.base.pack) ;
        p91_aux2_SET((float)8.86115E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan3_raw_SET((uint16_t)(uint16_t)35951, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)32598, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)51740, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)53813, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)63063, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)41862, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)6323, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)1716, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)26914, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)12491, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)49673, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)54620, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)8870994278679958258L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)9143718469304279823L, PH.base.pack) ;
        {
            float controls[] =  {-1.9929374E38F, 1.5941744E38F, 2.0445494E38F, -2.2081379E36F, -2.90828E38F, -1.3222085E38F, 1.5573441E38F, -1.6553245E38F, -4.226708E36F, 3.1148777E38F, -6.164555E36F, 1.2376298E38F, -1.7526821E38F, 1.772962E38F, 2.7983949E38F, 1.1335693E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)5103971221524761329L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)3041448896953357617L, PH.base.pack) ;
        p100_ground_distance_SET((float) -9.132041E36F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)1.3740337E38F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)6.966749E37F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -1.274092E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t) -17528, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)27721, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -3.1296571E38F, &PH) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_z_SET((float) -3.312887E38F, PH.base.pack) ;
        p101_y_SET((float)2.8735998E38F, PH.base.pack) ;
        p101_x_SET((float) -3.3322263E38F, PH.base.pack) ;
        p101_pitch_SET((float)1.9795944E38F, PH.base.pack) ;
        p101_roll_SET((float) -1.9811667E38F, PH.base.pack) ;
        p101_yaw_SET((float)7.110774E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)3431526208652439574L, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float) -2.9839505E38F, PH.base.pack) ;
        p102_yaw_SET((float) -1.6015866E38F, PH.base.pack) ;
        p102_pitch_SET((float)1.5387557E38F, PH.base.pack) ;
        p102_y_SET((float)2.0572022E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)3333953873928107621L, PH.base.pack) ;
        p102_z_SET((float)1.0745515E38F, PH.base.pack) ;
        p102_roll_SET((float)5.7036636E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_y_SET((float) -2.3928775E38F, PH.base.pack) ;
        p103_x_SET((float) -2.6107917E38F, PH.base.pack) ;
        p103_z_SET((float) -3.269458E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)4135466684440552753L, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_roll_SET((float)3.370568E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)9147514444017557728L, PH.base.pack) ;
        p104_yaw_SET((float) -2.4165818E38F, PH.base.pack) ;
        p104_z_SET((float) -8.4727454E37F, PH.base.pack) ;
        p104_x_SET((float)1.187243E38F, PH.base.pack) ;
        p104_y_SET((float) -2.9684824E38F, PH.base.pack) ;
        p104_pitch_SET((float) -5.666146E36F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zacc_SET((float) -1.732016E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -1.3490089E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.4954692E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -3.2566494E38F, PH.base.pack) ;
        p105_xgyro_SET((float) -4.9607366E37F, PH.base.pack) ;
        p105_yacc_SET((float) -1.5299391E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.6148027E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)1.6027243E38F, PH.base.pack) ;
        p105_temperature_SET((float) -1.0168329E38F, PH.base.pack) ;
        p105_ymag_SET((float)6.002739E37F, PH.base.pack) ;
        p105_zmag_SET((float)3.0339661E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8785828000405431647L, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)19109, PH.base.pack) ;
        p105_xacc_SET((float) -2.2828886E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -8.49169E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_x_SET((float) -5.5657275E37F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)2980944931L, PH.base.pack) ;
        p106_distance_SET((float)1.728803E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1118798630L, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p106_integrated_y_SET((float)2.0022422E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)24262, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -3.126758E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -2.1543537E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)1.2008081E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)8003489233565677399L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_time_usec_SET((uint64_t)3361173134042086987L, PH.base.pack) ;
        p107_ygyro_SET((float)2.4385166E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.7110931E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)1.6983134E38F, PH.base.pack) ;
        p107_temperature_SET((float) -1.6725603E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -1.9200455E38F, PH.base.pack) ;
        p107_yacc_SET((float) -2.747708E38F, PH.base.pack) ;
        p107_ymag_SET((float) -2.1291788E38F, PH.base.pack) ;
        p107_xacc_SET((float)1.5725522E38F, PH.base.pack) ;
        p107_xmag_SET((float) -8.3108167E37F, PH.base.pack) ;
        p107_zgyro_SET((float) -4.1061066E37F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)534919615L, PH.base.pack) ;
        p107_zmag_SET((float) -1.6249568E37F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.647673E38F, PH.base.pack) ;
        p107_zacc_SET((float)2.1877051E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_roll_SET((float)4.3616216E37F, PH.base.pack) ;
        p108_q1_SET((float) -3.936162E37F, PH.base.pack) ;
        p108_q4_SET((float)1.3679149E38F, PH.base.pack) ;
        p108_pitch_SET((float) -7.187141E37F, PH.base.pack) ;
        p108_vn_SET((float)3.0255019E38F, PH.base.pack) ;
        p108_xgyro_SET((float)2.0043899E38F, PH.base.pack) ;
        p108_q3_SET((float) -2.9554069E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.9350882E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.455119E38F, PH.base.pack) ;
        p108_lon_SET((float) -2.0154574E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -1.4873698E38F, PH.base.pack) ;
        p108_ve_SET((float)4.230914E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)3.370778E37F, PH.base.pack) ;
        p108_yacc_SET((float) -1.5963142E38F, PH.base.pack) ;
        p108_zgyro_SET((float)8.983614E36F, PH.base.pack) ;
        p108_alt_SET((float)1.3209895E38F, PH.base.pack) ;
        p108_yaw_SET((float)1.1899777E38F, PH.base.pack) ;
        p108_vd_SET((float) -2.8891988E38F, PH.base.pack) ;
        p108_zacc_SET((float)2.4667706E38F, PH.base.pack) ;
        p108_lat_SET((float) -2.5727918E38F, PH.base.pack) ;
        p108_ygyro_SET((float)9.946257E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_remnoise_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)41758, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)6606, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)195, (uint8_t)235, (uint8_t)96, (uint8_t)77, (uint8_t)31, (uint8_t)64, (uint8_t)204, (uint8_t)221, (uint8_t)151, (uint8_t)87, (uint8_t)35, (uint8_t)252, (uint8_t)135, (uint8_t)3, (uint8_t)145, (uint8_t)26, (uint8_t)222, (uint8_t)159, (uint8_t)140, (uint8_t)75, (uint8_t)132, (uint8_t)116, (uint8_t)178, (uint8_t)64, (uint8_t)194, (uint8_t)112, (uint8_t)240, (uint8_t)136, (uint8_t)62, (uint8_t)238, (uint8_t)221, (uint8_t)50, (uint8_t)150, (uint8_t)190, (uint8_t)46, (uint8_t)212, (uint8_t)3, (uint8_t)63, (uint8_t)129, (uint8_t)5, (uint8_t)176, (uint8_t)2, (uint8_t)215, (uint8_t)151, (uint8_t)248, (uint8_t)235, (uint8_t)48, (uint8_t)6, (uint8_t)145, (uint8_t)87, (uint8_t)139, (uint8_t)57, (uint8_t)117, (uint8_t)64, (uint8_t)115, (uint8_t)85, (uint8_t)207, (uint8_t)201, (uint8_t)221, (uint8_t)211, (uint8_t)6, (uint8_t)185, (uint8_t)6, (uint8_t)176, (uint8_t)83, (uint8_t)214, (uint8_t)67, (uint8_t)99, (uint8_t)225, (uint8_t)207, (uint8_t)189, (uint8_t)2, (uint8_t)52, (uint8_t)82, (uint8_t)140, (uint8_t)132, (uint8_t)17, (uint8_t)177, (uint8_t)127, (uint8_t)112, (uint8_t)201, (uint8_t)68, (uint8_t)28, (uint8_t)216, (uint8_t)162, (uint8_t)250, (uint8_t)135, (uint8_t)120, (uint8_t)173, (uint8_t)175, (uint8_t)218, (uint8_t)245, (uint8_t)92, (uint8_t)52, (uint8_t)170, (uint8_t)146, (uint8_t)129, (uint8_t)22, (uint8_t)154, (uint8_t)162, (uint8_t)251, (uint8_t)134, (uint8_t)83, (uint8_t)114, (uint8_t)234, (uint8_t)81, (uint8_t)108, (uint8_t)72, (uint8_t)1, (uint8_t)174, (uint8_t)211, (uint8_t)144, (uint8_t)183, (uint8_t)4, (uint8_t)183, (uint8_t)82, (uint8_t)204, (uint8_t)167, (uint8_t)162, (uint8_t)196, (uint8_t)94, (uint8_t)68, (uint8_t)132, (uint8_t)81, (uint8_t)35, (uint8_t)255, (uint8_t)123, (uint8_t)203, (uint8_t)228, (uint8_t)10, (uint8_t)244, (uint8_t)3, (uint8_t)206, (uint8_t)169, (uint8_t)116, (uint8_t)110, (uint8_t)51, (uint8_t)158, (uint8_t)51, (uint8_t)204, (uint8_t)174, (uint8_t)71, (uint8_t)49, (uint8_t)254, (uint8_t)47, (uint8_t)160, (uint8_t)41, (uint8_t)116, (uint8_t)38, (uint8_t)201, (uint8_t)55, (uint8_t)94, (uint8_t)111, (uint8_t)115, (uint8_t)202, (uint8_t)221, (uint8_t)46, (uint8_t)165, (uint8_t)119, (uint8_t)88, (uint8_t)151, (uint8_t)224, (uint8_t)60, (uint8_t)140, (uint8_t)143, (uint8_t)23, (uint8_t)44, (uint8_t)208, (uint8_t)17, (uint8_t)103, (uint8_t)94, (uint8_t)92, (uint8_t)241, (uint8_t)198, (uint8_t)45, (uint8_t)28, (uint8_t)25, (uint8_t)145, (uint8_t)74, (uint8_t)32, (uint8_t)102, (uint8_t)247, (uint8_t)65, (uint8_t)128, (uint8_t)160, (uint8_t)1, (uint8_t)169, (uint8_t)222, (uint8_t)243, (uint8_t)57, (uint8_t)77, (uint8_t)142, (uint8_t)6, (uint8_t)7, (uint8_t)42, (uint8_t)134, (uint8_t)251, (uint8_t)94, (uint8_t)3, (uint8_t)68, (uint8_t)247, (uint8_t)136, (uint8_t)90, (uint8_t)111, (uint8_t)78, (uint8_t)214, (uint8_t)166, (uint8_t)240, (uint8_t)36, (uint8_t)110, (uint8_t)195, (uint8_t)148, (uint8_t)72, (uint8_t)91, (uint8_t)210, (uint8_t)232, (uint8_t)99, (uint8_t)199, (uint8_t)193, (uint8_t)144, (uint8_t)95, (uint8_t)253, (uint8_t)195, (uint8_t)226, (uint8_t)125, (uint8_t)110, (uint8_t)153, (uint8_t)106, (uint8_t)159, (uint8_t)76, (uint8_t)150, (uint8_t)137, (uint8_t)62, (uint8_t)203, (uint8_t)219, (uint8_t)25, (uint8_t)166, (uint8_t)110, (uint8_t)183, (uint8_t)37, (uint8_t)226, (uint8_t)111, (uint8_t)199, (uint8_t)6, (uint8_t)5, (uint8_t)46, (uint8_t)19, (uint8_t)216, (uint8_t)155, (uint8_t)49, (uint8_t)182};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t) -2323993713934035355L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -5063514078532442807L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)1467868336L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)1041708171683274456L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_fix_type_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)42419, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)28802, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)40573, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)30529, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)22069, PH.base.pack) ;
        p113_lat_SET((int32_t)525980105, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)3852410421844451087L, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)30011, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -5979, PH.base.pack) ;
        p113_lon_SET((int32_t) -35568768, PH.base.pack) ;
        p113_alt_SET((int32_t) -329712364, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_distance_SET((float) -2.301962E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -2.1005709E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)1.7418978E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)1201527756L, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.8280124E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)12345, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -8.9400545E36F, PH.base.pack) ;
        p114_integrated_y_SET((float)1.1937839E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)3672929141L, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)7710763142997791616L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_true_airspeed_SET((uint16_t)(uint16_t)9775, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -547, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -4610, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)17666, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)19246, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)3128, PH.base.pack) ;
        p115_rollspeed_SET((float)3.2589826E38F, PH.base.pack) ;
        p115_yawspeed_SET((float)1.8527659E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -4552, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -6884, PH.base.pack) ;
        p115_lon_SET((int32_t)1244990876, PH.base.pack) ;
        p115_lat_SET((int32_t)777840675, PH.base.pack) ;
        p115_alt_SET((int32_t) -1867536338, PH.base.pack) ;
        p115_pitchspeed_SET((float) -1.9807193E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.7165245E38F, 3.0161422E38F, -3.0812862E38F, -2.0315806E36F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_time_usec_SET((uint64_t)5148363539189495964L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zmag_SET((int16_t)(int16_t)21283, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -4136, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -20186, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)9938, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -1790, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2482794757L, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)12171, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -27279, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)20954, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -26711, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)9244, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)49251, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)22166, PH.base.pack) ;
        p118_size_SET((uint32_t)158582314L, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)571211698L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)21164, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)25073, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)3285822087L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p119_ofs_SET((uint32_t)631404769L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)17449, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)3379386499L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)40597, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)160, (uint8_t)66, (uint8_t)117, (uint8_t)182, (uint8_t)218, (uint8_t)10, (uint8_t)248, (uint8_t)166, (uint8_t)106, (uint8_t)132, (uint8_t)81, (uint8_t)215, (uint8_t)208, (uint8_t)149, (uint8_t)23, (uint8_t)191, (uint8_t)37, (uint8_t)110, (uint8_t)239, (uint8_t)52, (uint8_t)82, (uint8_t)179, (uint8_t)141, (uint8_t)171, (uint8_t)49, (uint8_t)24, (uint8_t)193, (uint8_t)178, (uint8_t)130, (uint8_t)244, (uint8_t)242, (uint8_t)249, (uint8_t)74, (uint8_t)98, (uint8_t)32, (uint8_t)220, (uint8_t)243, (uint8_t)208, (uint8_t)60, (uint8_t)155, (uint8_t)93, (uint8_t)204, (uint8_t)201, (uint8_t)121, (uint8_t)196, (uint8_t)29, (uint8_t)160, (uint8_t)201, (uint8_t)238, (uint8_t)205, (uint8_t)169, (uint8_t)24, (uint8_t)92, (uint8_t)61, (uint8_t)7, (uint8_t)127, (uint8_t)65, (uint8_t)55, (uint8_t)172, (uint8_t)107, (uint8_t)128, (uint8_t)178, (uint8_t)15, (uint8_t)216, (uint8_t)124, (uint8_t)54, (uint8_t)110, (uint8_t)14, (uint8_t)28, (uint8_t)239, (uint8_t)100, (uint8_t)47, (uint8_t)19, (uint8_t)224, (uint8_t)83, (uint8_t)216, (uint8_t)58, (uint8_t)115, (uint8_t)107, (uint8_t)67, (uint8_t)198, (uint8_t)68, (uint8_t)7, (uint8_t)226, (uint8_t)50, (uint8_t)251, (uint8_t)69, (uint8_t)9, (uint8_t)236, (uint8_t)215};
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
        p121_target_component_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)50, (uint8_t)125, (uint8_t)48, (uint8_t)165, (uint8_t)21, (uint8_t)105, (uint8_t)39, (uint8_t)188, (uint8_t)36, (uint8_t)11, (uint8_t)81, (uint8_t)200, (uint8_t)177, (uint8_t)94, (uint8_t)137, (uint8_t)103, (uint8_t)70, (uint8_t)84, (uint8_t)252, (uint8_t)184, (uint8_t)209, (uint8_t)21, (uint8_t)124, (uint8_t)105, (uint8_t)15, (uint8_t)97, (uint8_t)21, (uint8_t)126, (uint8_t)91, (uint8_t)88, (uint8_t)220, (uint8_t)63, (uint8_t)25, (uint8_t)18, (uint8_t)241, (uint8_t)186, (uint8_t)252, (uint8_t)46, (uint8_t)194, (uint8_t)169, (uint8_t)129, (uint8_t)152, (uint8_t)216, (uint8_t)254, (uint8_t)224, (uint8_t)229, (uint8_t)157, (uint8_t)17, (uint8_t)194, (uint8_t)151, (uint8_t)167, (uint8_t)82, (uint8_t)236, (uint8_t)38, (uint8_t)234, (uint8_t)80, (uint8_t)20, (uint8_t)255, (uint8_t)170, (uint8_t)224, (uint8_t)248, (uint8_t)5, (uint8_t)16, (uint8_t)81, (uint8_t)146, (uint8_t)158, (uint8_t)230, (uint8_t)248, (uint8_t)125, (uint8_t)90, (uint8_t)40, (uint8_t)110, (uint8_t)142, (uint8_t)1, (uint8_t)44, (uint8_t)167, (uint8_t)95, (uint8_t)241, (uint8_t)136, (uint8_t)37, (uint8_t)49, (uint8_t)138, (uint8_t)134, (uint8_t)25, (uint8_t)128, (uint8_t)10, (uint8_t)191, (uint8_t)232, (uint8_t)160, (uint8_t)171, (uint8_t)92, (uint8_t)249, (uint8_t)83, (uint8_t)227, (uint8_t)171, (uint8_t)229, (uint8_t)66, (uint8_t)22, (uint8_t)249, (uint8_t)83, (uint8_t)216, (uint8_t)36, (uint8_t)219, (uint8_t)232, (uint8_t)54, (uint8_t)46, (uint8_t)56, (uint8_t)24, (uint8_t)53, (uint8_t)205};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p124_lat_SET((int32_t)970947804, PH.base.pack) ;
        p124_lon_SET((int32_t)1509377437, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p124_alt_SET((int32_t) -1959231154, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)50155, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)14136, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)57801, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1508189028L, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)45556, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)5047329237285931048L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)11168, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID), PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)13212, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)240174066L, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)23598, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)222, (uint8_t)55, (uint8_t)60, (uint8_t)143, (uint8_t)237, (uint8_t)64, (uint8_t)98, (uint8_t)232, (uint8_t)36, (uint8_t)236, (uint8_t)133, (uint8_t)103, (uint8_t)79, (uint8_t)84, (uint8_t)142, (uint8_t)173, (uint8_t)208, (uint8_t)199, (uint8_t)215, (uint8_t)19, (uint8_t)224, (uint8_t)86, (uint8_t)24, (uint8_t)146, (uint8_t)220, (uint8_t)219, (uint8_t)31, (uint8_t)184, (uint8_t)194, (uint8_t)201, (uint8_t)196, (uint8_t)44, (uint8_t)223, (uint8_t)146, (uint8_t)129, (uint8_t)68, (uint8_t)239, (uint8_t)14, (uint8_t)227, (uint8_t)167, (uint8_t)157, (uint8_t)212, (uint8_t)153, (uint8_t)29, (uint8_t)88, (uint8_t)176, (uint8_t)209, (uint8_t)79, (uint8_t)34, (uint8_t)74, (uint8_t)151, (uint8_t)110, (uint8_t)11, (uint8_t)41, (uint8_t)172, (uint8_t)196, (uint8_t)72, (uint8_t)120, (uint8_t)98, (uint8_t)246, (uint8_t)116, (uint8_t)174, (uint8_t)57, (uint8_t)182, (uint8_t)131, (uint8_t)132, (uint8_t)93, (uint8_t)86, (uint8_t)251, (uint8_t)253};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI), PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)249546146, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -2076951099, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)21534, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)695798664L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)4282744504L, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p127_tow_SET((uint32_t)3407032253L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1032322648, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1245591404, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_health_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t)25944539, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)26899, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4251687131L, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -726066085, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)155176704, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3967430616L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)1460436438, PH.base.pack) ;
        p128_tow_SET((uint32_t)4230919457L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_time_boot_ms_SET((uint32_t)3917002210L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)23056, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)24513, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)19913, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -26855, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -10448, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -10989, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)7510, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)23563, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)3125, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_type_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p130_size_SET((uint32_t)2844311304L, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)6884, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)48093, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)60531, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)13503, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)37, (uint8_t)172, (uint8_t)118, (uint8_t)68, (uint8_t)164, (uint8_t)177, (uint8_t)60, (uint8_t)232, (uint8_t)129, (uint8_t)20, (uint8_t)126, (uint8_t)142, (uint8_t)37, (uint8_t)162, (uint8_t)41, (uint8_t)171, (uint8_t)226, (uint8_t)253, (uint8_t)218, (uint8_t)11, (uint8_t)121, (uint8_t)157, (uint8_t)12, (uint8_t)56, (uint8_t)158, (uint8_t)111, (uint8_t)237, (uint8_t)103, (uint8_t)73, (uint8_t)186, (uint8_t)219, (uint8_t)246, (uint8_t)45, (uint8_t)41, (uint8_t)251, (uint8_t)247, (uint8_t)103, (uint8_t)240, (uint8_t)14, (uint8_t)4, (uint8_t)54, (uint8_t)76, (uint8_t)252, (uint8_t)114, (uint8_t)61, (uint8_t)192, (uint8_t)220, (uint8_t)39, (uint8_t)25, (uint8_t)220, (uint8_t)227, (uint8_t)41, (uint8_t)132, (uint8_t)123, (uint8_t)120, (uint8_t)56, (uint8_t)214, (uint8_t)203, (uint8_t)146, (uint8_t)21, (uint8_t)78, (uint8_t)3, (uint8_t)13, (uint8_t)209, (uint8_t)194, (uint8_t)139, (uint8_t)206, (uint8_t)90, (uint8_t)179, (uint8_t)185, (uint8_t)207, (uint8_t)217, (uint8_t)86, (uint8_t)5, (uint8_t)2, (uint8_t)154, (uint8_t)223, (uint8_t)62, (uint8_t)14, (uint8_t)44, (uint8_t)182, (uint8_t)248, (uint8_t)130, (uint8_t)208, (uint8_t)158, (uint8_t)102, (uint8_t)55, (uint8_t)98, (uint8_t)16, (uint8_t)16, (uint8_t)6, (uint8_t)10, (uint8_t)233, (uint8_t)179, (uint8_t)18, (uint8_t)92, (uint8_t)157, (uint8_t)48, (uint8_t)187, (uint8_t)9, (uint8_t)244, (uint8_t)66, (uint8_t)150, (uint8_t)144, (uint8_t)189, (uint8_t)73, (uint8_t)130, (uint8_t)62, (uint8_t)91, (uint8_t)189, (uint8_t)22, (uint8_t)138, (uint8_t)225, (uint8_t)242, (uint8_t)180, (uint8_t)82, (uint8_t)120, (uint8_t)126, (uint8_t)54, (uint8_t)222, (uint8_t)52, (uint8_t)227, (uint8_t)143, (uint8_t)122, (uint8_t)156, (uint8_t)122, (uint8_t)128, (uint8_t)179, (uint8_t)31, (uint8_t)64, (uint8_t)1, (uint8_t)127, (uint8_t)199, (uint8_t)93, (uint8_t)134, (uint8_t)87, (uint8_t)67, (uint8_t)253, (uint8_t)241, (uint8_t)50, (uint8_t)46, (uint8_t)124, (uint8_t)42, (uint8_t)172, (uint8_t)93, (uint8_t)166, (uint8_t)199, (uint8_t)84, (uint8_t)193, (uint8_t)22, (uint8_t)225, (uint8_t)251, (uint8_t)165, (uint8_t)6, (uint8_t)160, (uint8_t)49, (uint8_t)73, (uint8_t)209, (uint8_t)190, (uint8_t)237, (uint8_t)11, (uint8_t)8, (uint8_t)43, (uint8_t)227, (uint8_t)53, (uint8_t)69, (uint8_t)136, (uint8_t)129, (uint8_t)94, (uint8_t)8, (uint8_t)71, (uint8_t)173, (uint8_t)35, (uint8_t)45, (uint8_t)225, (uint8_t)173, (uint8_t)124, (uint8_t)115, (uint8_t)94, (uint8_t)178, (uint8_t)95, (uint8_t)156, (uint8_t)56, (uint8_t)25, (uint8_t)142, (uint8_t)230, (uint8_t)236, (uint8_t)244, (uint8_t)18, (uint8_t)116, (uint8_t)218, (uint8_t)83, (uint8_t)41, (uint8_t)79, (uint8_t)170, (uint8_t)157, (uint8_t)69, (uint8_t)248, (uint8_t)144, (uint8_t)114, (uint8_t)133, (uint8_t)35, (uint8_t)199, (uint8_t)217, (uint8_t)59, (uint8_t)185, (uint8_t)193, (uint8_t)82, (uint8_t)158, (uint8_t)98, (uint8_t)208, (uint8_t)111, (uint8_t)78, (uint8_t)32, (uint8_t)247, (uint8_t)226, (uint8_t)185, (uint8_t)215, (uint8_t)185, (uint8_t)111, (uint8_t)121, (uint8_t)30, (uint8_t)16, (uint8_t)94, (uint8_t)27, (uint8_t)22, (uint8_t)105, (uint8_t)208, (uint8_t)136, (uint8_t)84, (uint8_t)166, (uint8_t)235, (uint8_t)57, (uint8_t)79, (uint8_t)59, (uint8_t)67, (uint8_t)187, (uint8_t)214, (uint8_t)182, (uint8_t)123, (uint8_t)164, (uint8_t)70, (uint8_t)177, (uint8_t)188, (uint8_t)154, (uint8_t)87, (uint8_t)199, (uint8_t)246, (uint8_t)30, (uint8_t)45, (uint8_t)154, (uint8_t)61, (uint8_t)31};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_time_boot_ms_SET((uint32_t)3463580808L, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)22040, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)9096, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)48148, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)51678, PH.base.pack) ;
        p133_lon_SET((int32_t)1228916023, PH.base.pack) ;
        p133_mask_SET((uint64_t)5596105160408365810L, PH.base.pack) ;
        p133_lat_SET((int32_t) -353986407, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        {
            int16_t data_[] =  {(int16_t) -2254, (int16_t)32524, (int16_t)21867, (int16_t) -15860, (int16_t) -17866, (int16_t) -14735, (int16_t)29875, (int16_t) -14333, (int16_t) -1384, (int16_t)15236, (int16_t)20204, (int16_t)21868, (int16_t)26179, (int16_t)9819, (int16_t) -6454, (int16_t) -4824};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_grid_spacing_SET((uint16_t)(uint16_t)35551, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p134_lat_SET((int32_t)525840344, PH.base.pack) ;
        p134_lon_SET((int32_t)1111815485, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -2053563032, PH.base.pack) ;
        p135_lon_SET((int32_t) -1725774593, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)58949, PH.base.pack) ;
        p136_lon_SET((int32_t) -409978620, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)7248, PH.base.pack) ;
        p136_lat_SET((int32_t) -481373278, PH.base.pack) ;
        p136_current_height_SET((float)3.7730816E37F, PH.base.pack) ;
        p136_terrain_height_SET((float)6.563776E37F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)57338, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float)2.9765318E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -1.5490682E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)2360107720L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -12012, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {-2.0121323E38F, 2.763543E38F, 1.921227E38F, -1.9454709E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -2.7708603E37F, PH.base.pack) ;
        p138_y_SET((float) -2.5599212E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)48889841410749994L, PH.base.pack) ;
        p138_z_SET((float) -3.3231834E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)8187692198451616338L, PH.base.pack) ;
        {
            float controls[] =  {-1.7845634E38F, 9.547923E37F, 3.4436824E37F, 1.5197521E38F, 3.1687324E38F, -2.660771E38F, -1.90448E38F, 2.9291227E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {1.9711056E38F, -2.3320836E38F, 1.6010648E38F, 2.286581E37F, -2.8492932E38F, 2.1954207E38F, -2.0070035E37F, 1.4504018E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)7078152087248273356L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float)3.0146364E37F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.2614218E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)9192553716770657361L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -3.630095E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)6.357735E37F, PH.base.pack) ;
        p141_altitude_local_SET((float) -4.7031866E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.7667722E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)63, (uint8_t)32, (uint8_t)135, (uint8_t)177, (uint8_t)105, (uint8_t)24, (uint8_t)54, (uint8_t)8, (uint8_t)139, (uint8_t)26, (uint8_t)216, (uint8_t)222, (uint8_t)182, (uint8_t)158, (uint8_t)0, (uint8_t)249, (uint8_t)149, (uint8_t)180, (uint8_t)181, (uint8_t)171, (uint8_t)19, (uint8_t)188, (uint8_t)223, (uint8_t)191, (uint8_t)61, (uint8_t)1, (uint8_t)125, (uint8_t)84, (uint8_t)146, (uint8_t)200, (uint8_t)218, (uint8_t)39, (uint8_t)95, (uint8_t)122, (uint8_t)124, (uint8_t)150, (uint8_t)196, (uint8_t)96, (uint8_t)38, (uint8_t)78, (uint8_t)24, (uint8_t)23, (uint8_t)47, (uint8_t)255, (uint8_t)192, (uint8_t)154, (uint8_t)16, (uint8_t)69, (uint8_t)75, (uint8_t)10, (uint8_t)97, (uint8_t)88, (uint8_t)38, (uint8_t)35, (uint8_t)249, (uint8_t)155, (uint8_t)199, (uint8_t)38, (uint8_t)86, (uint8_t)59, (uint8_t)205, (uint8_t)235, (uint8_t)114, (uint8_t)152, (uint8_t)243, (uint8_t)179, (uint8_t)49, (uint8_t)110, (uint8_t)158, (uint8_t)29, (uint8_t)6, (uint8_t)75, (uint8_t)184, (uint8_t)132, (uint8_t)23, (uint8_t)217, (uint8_t)191, (uint8_t)211, (uint8_t)191, (uint8_t)200, (uint8_t)68, (uint8_t)163, (uint8_t)80, (uint8_t)34, (uint8_t)135, (uint8_t)96, (uint8_t)111, (uint8_t)44, (uint8_t)159, (uint8_t)220, (uint8_t)11, (uint8_t)172, (uint8_t)207, (uint8_t)53, (uint8_t)68, (uint8_t)223, (uint8_t)202, (uint8_t)131, (uint8_t)223, (uint8_t)117, (uint8_t)132, (uint8_t)137, (uint8_t)141, (uint8_t)24, (uint8_t)11, (uint8_t)243, (uint8_t)10, (uint8_t)164, (uint8_t)29, (uint8_t)110, (uint8_t)147, (uint8_t)106, (uint8_t)225, (uint8_t)179, (uint8_t)197, (uint8_t)114, (uint8_t)112, (uint8_t)170, (uint8_t)53, (uint8_t)247};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)200, (uint8_t)22, (uint8_t)239, (uint8_t)224, (uint8_t)112, (uint8_t)217, (uint8_t)144, (uint8_t)0, (uint8_t)196, (uint8_t)100, (uint8_t)103, (uint8_t)176, (uint8_t)160, (uint8_t)219, (uint8_t)254, (uint8_t)143, (uint8_t)77, (uint8_t)171, (uint8_t)224, (uint8_t)34, (uint8_t)218, (uint8_t)229, (uint8_t)159, (uint8_t)133, (uint8_t)140, (uint8_t)157, (uint8_t)102, (uint8_t)162, (uint8_t)81, (uint8_t)141, (uint8_t)112, (uint8_t)172, (uint8_t)170, (uint8_t)117, (uint8_t)44, (uint8_t)86, (uint8_t)71, (uint8_t)8, (uint8_t)31, (uint8_t)139, (uint8_t)61, (uint8_t)83, (uint8_t)36, (uint8_t)219, (uint8_t)181, (uint8_t)124, (uint8_t)25, (uint8_t)44, (uint8_t)163, (uint8_t)152, (uint8_t)110, (uint8_t)216, (uint8_t)141, (uint8_t)66, (uint8_t)177, (uint8_t)78, (uint8_t)68, (uint8_t)191, (uint8_t)236, (uint8_t)28, (uint8_t)8, (uint8_t)0, (uint8_t)120, (uint8_t)222, (uint8_t)202, (uint8_t)198, (uint8_t)111, (uint8_t)195, (uint8_t)103, (uint8_t)80, (uint8_t)230, (uint8_t)58, (uint8_t)225, (uint8_t)64, (uint8_t)128, (uint8_t)191, (uint8_t)143, (uint8_t)219, (uint8_t)141, (uint8_t)206, (uint8_t)82, (uint8_t)90, (uint8_t)1, (uint8_t)34, (uint8_t)74, (uint8_t)74, (uint8_t)96, (uint8_t)158, (uint8_t)215, (uint8_t)159, (uint8_t)57, (uint8_t)174, (uint8_t)228, (uint8_t)78, (uint8_t)83, (uint8_t)138, (uint8_t)22, (uint8_t)13, (uint8_t)158, (uint8_t)153, (uint8_t)139, (uint8_t)135, (uint8_t)41, (uint8_t)119, (uint8_t)77, (uint8_t)11, (uint8_t)214, (uint8_t)169, (uint8_t)159, (uint8_t)70, (uint8_t)197, (uint8_t)23, (uint8_t)70, (uint8_t)248, (uint8_t)168, (uint8_t)242, (uint8_t)58, (uint8_t)22, (uint8_t)251, (uint8_t)216};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_time_boot_ms_SET((uint32_t)2014288752L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -18086, PH.base.pack) ;
        p143_press_abs_SET((float)3.3939408E38F, PH.base.pack) ;
        p143_press_diff_SET((float)3.0538936E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_alt_SET((float)2.9031365E38F, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p144_lon_SET((int32_t)1477549743, PH.base.pack) ;
        {
            float attitude_q[] =  {-1.6011703E38F, -1.77578E38F, 2.854077E38F, -2.3945754E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-1.0855069E38F, -1.9052853E38F, -1.9823129E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)5110574854055931875L, PH.base.pack) ;
        {
            float rates[] =  {-2.8763728E38F, 1.7474837E38F, -2.6111511E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-1.855359E38F, 1.2217056E38F, 2.2981958E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)4398681343650275183L, PH.base.pack) ;
        {
            float vel[] =  {-4.88537E37F, -3.273396E37F, -1.150865E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -50041618, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_pos_SET((float) -1.820332E38F, PH.base.pack) ;
        p146_y_pos_SET((float)2.7326904E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {-1.9709093E37F, -2.410432E38F, 5.154598E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float)2.2405136E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-1.2827171E38F, -2.981915E38F, 2.2982057E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)4533545213163992157L, PH.base.pack) ;
        p146_z_vel_SET((float)2.6914687E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -4.0819784E37F, PH.base.pack) ;
        p146_x_vel_SET((float) -1.4197945E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.0289302E38F, PH.base.pack) ;
        {
            float q[] =  {1.6007995E38F, 8.93943E37F, 2.9130193E38F, -2.8801474E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_acc_SET((float)1.5866212E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -1.7829088E38F, PH.base.pack) ;
        p146_y_vel_SET((float)1.5403712E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.2805517E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -3.1555716E38F, PH.base.pack) ;
        p146_airspeed_SET((float)2.8567638E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)54, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)195197439, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -17858, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1113908176, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)14921, (uint16_t)29688, (uint16_t)19325, (uint16_t)61912, (uint16_t)62550, (uint16_t)38599, (uint16_t)14247, (uint16_t)12259, (uint16_t)15708, (uint16_t)46458};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_temperature_SET((int16_t)(int16_t)26171, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_uid_SET((uint64_t)9067564254384443685L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)852891055L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)137, (uint8_t)243, (uint8_t)151, (uint8_t)138, (uint8_t)14, (uint8_t)251, (uint8_t)117, (uint8_t)225, (uint8_t)73, (uint8_t)42, (uint8_t)15, (uint8_t)86, (uint8_t)25, (uint8_t)249, (uint8_t)48, (uint8_t)10, (uint8_t)91, (uint8_t)168};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)154, (uint8_t)192, (uint8_t)101, (uint8_t)191, (uint8_t)212, (uint8_t)248, (uint8_t)92, (uint8_t)123};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)63770, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)101, (uint8_t)106, (uint8_t)108, (uint8_t)61, (uint8_t)94, (uint8_t)132, (uint8_t)212, (uint8_t)213};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)1219949014L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)216, (uint8_t)64, (uint8_t)228, (uint8_t)87, (uint8_t)232, (uint8_t)155, (uint8_t)31, (uint8_t)66};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)852342072L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)3933482815L, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION), PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)29820, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p149_z_SET((float) -1.4770808E38F, &PH) ;
        {
            float q[] =  {1.0530255E38F, 2.7881131E38F, 2.7828659E38F, -2.020443E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_size_x_SET((float)1.9876605E38F, PH.base.pack) ;
        p149_distance_SET((float) -1.601615E38F, PH.base.pack) ;
        p149_y_SET((float)1.6411516E38F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)255, &PH) ;
        p149_angle_x_SET((float)3.2276252E38F, PH.base.pack) ;
        p149_size_y_SET((float)7.263641E37F, PH.base.pack) ;
        p149_angle_y_SET((float)1.2160274E37F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
        p149_x_SET((float)3.0193352E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)3074707530477782986L, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_SET_150(), &PH);
        p150_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p150_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_READ_REQ_151(), &PH);
        p151_data_index_SET((int16_t)(int16_t) -15074, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p151_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p151_read_req_type_SET((int16_t)(int16_t) -4815, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_152(), &PH);
        p152_func_count_SET((uint16_t)(uint16_t)28135, PH.base.pack) ;
        {
            int8_t data_[] =  {(int8_t)93, (int8_t)35, (int8_t)108, (int8_t) -58, (int8_t) -87, (int8_t)104, (int8_t) -69, (int8_t) -31, (int8_t) -91, (int8_t) -53, (int8_t) -6, (int8_t) -26, (int8_t) -123, (int8_t)100, (int8_t) -110, (int8_t) -101, (int8_t)93, (int8_t)126, (int8_t)82, (int8_t) -112, (int8_t) -59, (int8_t) -66, (int8_t) -18, (int8_t)10, (int8_t) -29, (int8_t)119, (int8_t) -113, (int8_t) -99, (int8_t)117, (int8_t) -103, (int8_t)91, (int8_t)64, (int8_t)71, (int8_t)71, (int8_t)81, (int8_t)96, (int8_t)68, (int8_t) -74, (int8_t)83, (int8_t)89, (int8_t)105, (int8_t) -111, (int8_t)79, (int8_t) -117, (int8_t)21, (int8_t)38, (int8_t)30, (int8_t) -63};
            p152_data__SET(&data_, 0, PH.base.pack) ;
        }
        p152_func_index_SET((uint16_t)(uint16_t)58171, PH.base.pack) ;
        p152_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p152_data_size_SET((uint16_t)(uint16_t)8714, PH.base.pack) ;
        p152_data_address_SET((uint16_t)(uint16_t)60302, PH.base.pack) ;
        p152_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(), &PH);
        p153_func_index_SET((uint16_t)(uint16_t)21210, PH.base.pack) ;
        p153_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p153_target_component_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p153_result_SET((uint16_t)(uint16_t)45228, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_155(), &PH);
        p155_start_index_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p155_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p155_count_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        {
            int8_t directory_data[] =  {(int8_t) -29, (int8_t)122, (int8_t)34, (int8_t) -99, (int8_t)76, (int8_t) -125, (int8_t) -124, (int8_t)97, (int8_t)56, (int8_t) -83, (int8_t) -27, (int8_t) -24, (int8_t) -121, (int8_t)122, (int8_t) -76, (int8_t) -24, (int8_t)106, (int8_t) -62, (int8_t)94, (int8_t) -52, (int8_t)49, (int8_t) -22, (int8_t) -25, (int8_t) -63, (int8_t)82, (int8_t) -29, (int8_t) -126, (int8_t) -56, (int8_t) -79, (int8_t)30, (int8_t)111, (int8_t)87, (int8_t) -25, (int8_t)119, (int8_t)33, (int8_t) -108, (int8_t)102, (int8_t) -56, (int8_t)114, (int8_t) -105, (int8_t)43, (int8_t)71, (int8_t)74, (int8_t) -41, (int8_t)4, (int8_t)93, (int8_t)79, (int8_t)62};
            p155_directory_data_SET(&directory_data, 0, PH.base.pack) ;
        }
        p155_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p155_directory_type_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_DIRECTORY_ACK_156(), &PH);
        p156_count_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p156_result_SET((uint16_t)(uint16_t)38204, PH.base.pack) ;
        p156_start_index_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p156_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p156_directory_type_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_157(), &PH);
        p157_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p157_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p157_command_type_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLEXIFUNCTION_COMMAND_ACK_158(), &PH);
        p158_command_type_SET((uint16_t)(uint16_t)17452, PH.base.pack) ;
        p158_result_SET((uint16_t)(uint16_t)40800, PH.base.pack) ;
        c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_A_170(), &PH);
        p170_sue_estimated_wind_2_SET((int16_t)(int16_t)18899, PH.base.pack) ;
        p170_sue_rmat8_SET((int16_t)(int16_t)4809, PH.base.pack) ;
        p170_sue_magFieldEarth2_SET((int16_t)(int16_t)23762, PH.base.pack) ;
        p170_sue_rmat2_SET((int16_t)(int16_t) -10012, PH.base.pack) ;
        p170_sue_estimated_wind_0_SET((int16_t)(int16_t)11191, PH.base.pack) ;
        p170_sue_magFieldEarth1_SET((int16_t)(int16_t)25105, PH.base.pack) ;
        p170_sue_rmat7_SET((int16_t)(int16_t) -16170, PH.base.pack) ;
        p170_sue_altitude_SET((int32_t)148488354, PH.base.pack) ;
        p170_sue_magFieldEarth0_SET((int16_t)(int16_t) -5745, PH.base.pack) ;
        p170_sue_rmat4_SET((int16_t)(int16_t) -8816, PH.base.pack) ;
        p170_sue_waypoint_index_SET((uint16_t)(uint16_t)49667, PH.base.pack) ;
        p170_sue_time_SET((uint32_t)3484252340L, PH.base.pack) ;
        p170_sue_latitude_SET((int32_t) -693891273, PH.base.pack) ;
        p170_sue_longitude_SET((int32_t)2069303475, PH.base.pack) ;
        p170_sue_cog_SET((uint16_t)(uint16_t)61663, PH.base.pack) ;
        p170_sue_estimated_wind_1_SET((int16_t)(int16_t)31101, PH.base.pack) ;
        p170_sue_rmat0_SET((int16_t)(int16_t)6663, PH.base.pack) ;
        p170_sue_sog_SET((int16_t)(int16_t) -31571, PH.base.pack) ;
        p170_sue_air_speed_3DIMU_SET((uint16_t)(uint16_t)43264, PH.base.pack) ;
        p170_sue_cpu_load_SET((uint16_t)(uint16_t)19197, PH.base.pack) ;
        p170_sue_svs_SET((int16_t)(int16_t) -19214, PH.base.pack) ;
        p170_sue_rmat3_SET((int16_t)(int16_t)26516, PH.base.pack) ;
        p170_sue_rmat6_SET((int16_t)(int16_t)32157, PH.base.pack) ;
        p170_sue_rmat1_SET((int16_t)(int16_t) -24245, PH.base.pack) ;
        p170_sue_rmat5_SET((int16_t)(int16_t)14992, PH.base.pack) ;
        p170_sue_status_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p170_sue_hdop_SET((int16_t)(int16_t) -11680, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F2_B_171(), &PH);
        p171_sue_waypoint_goal_z_SET((int16_t)(int16_t)18269, PH.base.pack) ;
        p171_sue_pwm_output_8_SET((int16_t)(int16_t) -974, PH.base.pack) ;
        p171_sue_pwm_input_6_SET((int16_t)(int16_t) -23743, PH.base.pack) ;
        p171_sue_pwm_input_3_SET((int16_t)(int16_t)25951, PH.base.pack) ;
        p171_sue_imu_location_z_SET((int16_t)(int16_t)6443, PH.base.pack) ;
        p171_sue_imu_location_y_SET((int16_t)(int16_t)23580, PH.base.pack) ;
        p171_sue_pwm_input_4_SET((int16_t)(int16_t)9858, PH.base.pack) ;
        p171_sue_pwm_output_12_SET((int16_t)(int16_t) -11904, PH.base.pack) ;
        p171_sue_imu_velocity_y_SET((int16_t)(int16_t) -5879, PH.base.pack) ;
        p171_sue_pwm_output_10_SET((int16_t)(int16_t)30722, PH.base.pack) ;
        p171_sue_pwm_input_7_SET((int16_t)(int16_t) -16618, PH.base.pack) ;
        p171_sue_osc_fails_SET((int16_t)(int16_t) -16411, PH.base.pack) ;
        p171_sue_time_SET((uint32_t)55687832L, PH.base.pack) ;
        p171_sue_pwm_input_1_SET((int16_t)(int16_t)16661, PH.base.pack) ;
        p171_sue_pwm_input_2_SET((int16_t)(int16_t) -4269, PH.base.pack) ;
        p171_sue_pwm_output_5_SET((int16_t)(int16_t)14746, PH.base.pack) ;
        p171_sue_pwm_input_11_SET((int16_t)(int16_t)20321, PH.base.pack) ;
        p171_sue_imu_location_x_SET((int16_t)(int16_t) -13597, PH.base.pack) ;
        p171_sue_aero_x_SET((int16_t)(int16_t)9178, PH.base.pack) ;
        p171_sue_pwm_output_9_SET((int16_t)(int16_t) -23124, PH.base.pack) ;
        p171_sue_waypoint_goal_x_SET((int16_t)(int16_t) -26670, PH.base.pack) ;
        p171_sue_pwm_input_8_SET((int16_t)(int16_t) -27934, PH.base.pack) ;
        p171_sue_aero_z_SET((int16_t)(int16_t)22265, PH.base.pack) ;
        p171_sue_pwm_input_10_SET((int16_t)(int16_t) -2733, PH.base.pack) ;
        p171_sue_pwm_input_9_SET((int16_t)(int16_t) -25839, PH.base.pack) ;
        p171_sue_desired_height_SET((int16_t)(int16_t)19161, PH.base.pack) ;
        p171_sue_pwm_input_5_SET((int16_t)(int16_t)7360, PH.base.pack) ;
        p171_sue_pwm_output_11_SET((int16_t)(int16_t)30410, PH.base.pack) ;
        p171_sue_bat_amp_SET((int16_t)(int16_t)13443, PH.base.pack) ;
        p171_sue_barom_temp_SET((int16_t)(int16_t)12287, PH.base.pack) ;
        p171_sue_location_error_earth_y_SET((int16_t)(int16_t)22502, PH.base.pack) ;
        p171_sue_pwm_output_2_SET((int16_t)(int16_t)14158, PH.base.pack) ;
        p171_sue_pwm_input_12_SET((int16_t)(int16_t)26934, PH.base.pack) ;
        p171_sue_barom_press_SET((int32_t) -846929416, PH.base.pack) ;
        p171_sue_location_error_earth_z_SET((int16_t)(int16_t) -24808, PH.base.pack) ;
        p171_sue_memory_stack_free_SET((int16_t)(int16_t) -4709, PH.base.pack) ;
        p171_sue_bat_amp_hours_SET((int16_t)(int16_t) -25578, PH.base.pack) ;
        p171_sue_location_error_earth_x_SET((int16_t)(int16_t)10397, PH.base.pack) ;
        p171_sue_waypoint_goal_y_SET((int16_t)(int16_t)23775, PH.base.pack) ;
        p171_sue_barom_alt_SET((int32_t) -2091844492, PH.base.pack) ;
        p171_sue_flags_SET((uint32_t)4185152880L, PH.base.pack) ;
        p171_sue_imu_velocity_z_SET((int16_t)(int16_t)19031, PH.base.pack) ;
        p171_sue_bat_volt_SET((int16_t)(int16_t) -2933, PH.base.pack) ;
        p171_sue_aero_y_SET((int16_t)(int16_t) -30510, PH.base.pack) ;
        p171_sue_pwm_output_4_SET((int16_t)(int16_t) -9654, PH.base.pack) ;
        p171_sue_pwm_output_3_SET((int16_t)(int16_t) -25170, PH.base.pack) ;
        p171_sue_pwm_output_7_SET((int16_t)(int16_t)3847, PH.base.pack) ;
        p171_sue_pwm_output_1_SET((int16_t)(int16_t) -1308, PH.base.pack) ;
        p171_sue_imu_velocity_x_SET((int16_t)(int16_t) -11678, PH.base.pack) ;
        p171_sue_pwm_output_6_SET((int16_t)(int16_t)28949, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F4_172(), &PH);
        p172_sue_YAW_STABILIZATION_AILERON_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p172_sue_ROLL_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p172_sue_PITCH_STABILIZATION_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p172_sue_ROLL_STABILIZATION_AILERONS_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p172_sue_RUDDER_NAVIGATION_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_STABILIZED_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p172_sue_YAW_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p172_sue_RACING_MODE_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_WAYPOINT_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p172_sue_AILERON_NAVIGATION_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F5_173(), &PH);
        p173_sue_YAWKP_AILERON_SET((float)8.450248E37F, PH.base.pack) ;
        p173_sue_YAWKD_AILERON_SET((float) -2.8088142E38F, PH.base.pack) ;
        p173_sue_ROLLKD_SET((float)7.223927E37F, PH.base.pack) ;
        p173_sue_ROLLKP_SET((float)3.037306E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F6_174(), &PH);
        p174_sue_PITCHKD_SET((float) -1.5711111E38F, PH.base.pack) ;
        p174_sue_ELEVATOR_BOOST_SET((float)6.379799E37F, PH.base.pack) ;
        p174_sue_PITCHGAIN_SET((float)2.9056334E37F, PH.base.pack) ;
        p174_sue_RUDDER_ELEV_MIX_SET((float) -2.4906174E38F, PH.base.pack) ;
        p174_sue_ROLL_ELEV_MIX_SET((float)1.8291379E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F7_175(), &PH);
        p175_sue_ROLLKD_RUDDER_SET((float)1.8181799E38F, PH.base.pack) ;
        p175_sue_YAWKD_RUDDER_SET((float)1.1987283E38F, PH.base.pack) ;
        p175_sue_RTL_PITCH_DOWN_SET((float)2.9663704E38F, PH.base.pack) ;
        p175_sue_YAWKP_RUDDER_SET((float) -1.6174207E38F, PH.base.pack) ;
        p175_sue_ROLLKP_RUDDER_SET((float)9.828309E37F, PH.base.pack) ;
        p175_sue_RUDDER_BOOST_SET((float)3.2700168E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F8_176(), &PH);
        p176_sue_ALT_HOLD_THROTTLE_MIN_SET((float) -2.6825527E38F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MAX_SET((float) -1.0712995E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_MAX_SET((float) -7.1720217E36F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_HIGH_SET((float)3.4019544E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_MIN_SET((float) -1.6597462E37F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MIN_SET((float)1.0155596E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_THROTTLE_MAX_SET((float) -7.2600043E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F13_177(), &PH);
        p177_sue_lat_origin_SET((int32_t)2139992110, PH.base.pack) ;
        p177_sue_lon_origin_SET((int32_t) -1889006497, PH.base.pack) ;
        p177_sue_alt_origin_SET((int32_t)338786242, PH.base.pack) ;
        p177_sue_week_no_SET((int16_t)(int16_t) -20405, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F14_178(), &PH);
        p178_sue_WIND_ESTIMATION_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p178_sue_FLIGHT_PLAN_TYPE_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p178_sue_AIRFRAME_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p178_sue_osc_fail_count_SET((int16_t)(int16_t) -31542, PH.base.pack) ;
        p178_sue_DR_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p178_sue_RCON_SET((int16_t)(int16_t) -4969, PH.base.pack) ;
        p178_sue_GPS_TYPE_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p178_sue_BOARD_TYPE_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p178_sue_TRAP_FLAGS_SET((int16_t)(int16_t)15274, PH.base.pack) ;
        p178_sue_CLOCK_CONFIG_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p178_sue_TRAP_SOURCE_SET((uint32_t)2317053118L, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F15_179(), &PH);
        {
            uint8_t sue_ID_VEHICLE_REGISTRATION[] =  {(uint8_t)223, (uint8_t)208, (uint8_t)27, (uint8_t)22, (uint8_t)243, (uint8_t)252, (uint8_t)191, (uint8_t)153, (uint8_t)195, (uint8_t)142, (uint8_t)27, (uint8_t)244, (uint8_t)24, (uint8_t)199, (uint8_t)37, (uint8_t)6, (uint8_t)137, (uint8_t)14, (uint8_t)128, (uint8_t)182};
            p179_sue_ID_VEHICLE_REGISTRATION_SET(&sue_ID_VEHICLE_REGISTRATION, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_VEHICLE_MODEL_NAME[] =  {(uint8_t)27, (uint8_t)30, (uint8_t)148, (uint8_t)35, (uint8_t)177, (uint8_t)149, (uint8_t)88, (uint8_t)72, (uint8_t)182, (uint8_t)34, (uint8_t)107, (uint8_t)137, (uint8_t)240, (uint8_t)117, (uint8_t)239, (uint8_t)222, (uint8_t)93, (uint8_t)236, (uint8_t)52, (uint8_t)209, (uint8_t)208, (uint8_t)94, (uint8_t)109, (uint8_t)129, (uint8_t)251, (uint8_t)236, (uint8_t)250, (uint8_t)43, (uint8_t)201, (uint8_t)98, (uint8_t)109, (uint8_t)145, (uint8_t)50, (uint8_t)111, (uint8_t)94, (uint8_t)16, (uint8_t)112, (uint8_t)177, (uint8_t)132, (uint8_t)200};
            p179_sue_ID_VEHICLE_MODEL_NAME_SET(&sue_ID_VEHICLE_MODEL_NAME, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F16_180(), &PH);
        {
            uint8_t sue_ID_LEAD_PILOT[] =  {(uint8_t)214, (uint8_t)217, (uint8_t)52, (uint8_t)143, (uint8_t)253, (uint8_t)183, (uint8_t)121, (uint8_t)28, (uint8_t)176, (uint8_t)210, (uint8_t)70, (uint8_t)36, (uint8_t)117, (uint8_t)97, (uint8_t)184, (uint8_t)53, (uint8_t)172, (uint8_t)95, (uint8_t)172, (uint8_t)91, (uint8_t)236, (uint8_t)63, (uint8_t)140, (uint8_t)126, (uint8_t)9, (uint8_t)167, (uint8_t)186, (uint8_t)38, (uint8_t)94, (uint8_t)41, (uint8_t)119, (uint8_t)189, (uint8_t)176, (uint8_t)190, (uint8_t)84, (uint8_t)119, (uint8_t)2, (uint8_t)88, (uint8_t)184, (uint8_t)98};
            p180_sue_ID_LEAD_PILOT_SET(&sue_ID_LEAD_PILOT, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_DIY_DRONES_URL[] =  {(uint8_t)115, (uint8_t)106, (uint8_t)91, (uint8_t)18, (uint8_t)109, (uint8_t)14, (uint8_t)241, (uint8_t)52, (uint8_t)23, (uint8_t)47, (uint8_t)142, (uint8_t)248, (uint8_t)175, (uint8_t)49, (uint8_t)90, (uint8_t)4, (uint8_t)109, (uint8_t)249, (uint8_t)244, (uint8_t)249, (uint8_t)227, (uint8_t)76, (uint8_t)9, (uint8_t)1, (uint8_t)82, (uint8_t)151, (uint8_t)169, (uint8_t)43, (uint8_t)162, (uint8_t)6, (uint8_t)4, (uint8_t)38, (uint8_t)123, (uint8_t)135, (uint8_t)38, (uint8_t)197, (uint8_t)31, (uint8_t)37, (uint8_t)10, (uint8_t)190, (uint8_t)97, (uint8_t)173, (uint8_t)9, (uint8_t)169, (uint8_t)203, (uint8_t)179, (uint8_t)175, (uint8_t)59, (uint8_t)204, (uint8_t)162, (uint8_t)183, (uint8_t)175, (uint8_t)251, (uint8_t)61, (uint8_t)235, (uint8_t)5, (uint8_t)91, (uint8_t)47, (uint8_t)132, (uint8_t)92, (uint8_t)109, (uint8_t)92, (uint8_t)10, (uint8_t)85, (uint8_t)51, (uint8_t)182, (uint8_t)244, (uint8_t)196, (uint8_t)45, (uint8_t)51};
            p180_sue_ID_DIY_DRONES_URL_SET(&sue_ID_DIY_DRONES_URL, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDES_181(), &PH);
        p181_alt_barometric_SET((int32_t)1868092585, PH.base.pack) ;
        p181_alt_optical_flow_SET((int32_t) -607629615, PH.base.pack) ;
        p181_time_boot_ms_SET((uint32_t)164585507L, PH.base.pack) ;
        p181_alt_extra_SET((int32_t)1310662883, PH.base.pack) ;
        p181_alt_imu_SET((int32_t)704035024, PH.base.pack) ;
        p181_alt_gps_SET((int32_t)2113711184, PH.base.pack) ;
        p181_alt_range_finder_SET((int32_t)544756421, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDES_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AIRSPEEDS_182(), &PH);
        p182_airspeed_ultrasonic_SET((int16_t)(int16_t)28330, PH.base.pack) ;
        p182_airspeed_hot_wire_SET((int16_t)(int16_t)21550, PH.base.pack) ;
        p182_airspeed_pitot_SET((int16_t)(int16_t)25121, PH.base.pack) ;
        p182_aoa_SET((int16_t)(int16_t) -10075, PH.base.pack) ;
        p182_aoy_SET((int16_t)(int16_t) -5633, PH.base.pack) ;
        p182_time_boot_ms_SET((uint32_t)1659293800L, PH.base.pack) ;
        p182_airspeed_imu_SET((int16_t)(int16_t) -21164, PH.base.pack) ;
        c_CommunicationChannel_on_AIRSPEEDS_182(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F17_183(), &PH);
        p183_sue_feed_forward_SET((float)2.7609266E38F, PH.base.pack) ;
        p183_sue_turn_rate_nav_SET((float) -1.601833E38F, PH.base.pack) ;
        p183_sue_turn_rate_fbw_SET((float)3.8468716E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F18_184(), &PH);
        p184_angle_of_attack_inverted_SET((float) -7.1742113E37F, PH.base.pack) ;
        p184_angle_of_attack_normal_SET((float) -2.1624276E38F, PH.base.pack) ;
        p184_elevator_trim_inverted_SET((float) -1.6673615E38F, PH.base.pack) ;
        p184_reference_speed_SET((float) -4.7042316E37F, PH.base.pack) ;
        p184_elevator_trim_normal_SET((float) -2.4729016E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F19_185(), &PH);
        p185_sue_throttle_reversed_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p185_sue_elevator_output_channel_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p185_sue_rudder_reversed_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p185_sue_aileron_output_channel_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p185_sue_elevator_reversed_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p185_sue_rudder_output_channel_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p185_sue_throttle_output_channel_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p185_sue_aileron_reversed_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F20_186(), &PH);
        p186_sue_trim_value_input_8_SET((int16_t)(int16_t)21664, PH.base.pack) ;
        p186_sue_trim_value_input_4_SET((int16_t)(int16_t)32475, PH.base.pack) ;
        p186_sue_trim_value_input_11_SET((int16_t)(int16_t) -9883, PH.base.pack) ;
        p186_sue_trim_value_input_2_SET((int16_t)(int16_t)5248, PH.base.pack) ;
        p186_sue_trim_value_input_12_SET((int16_t)(int16_t) -19531, PH.base.pack) ;
        p186_sue_trim_value_input_5_SET((int16_t)(int16_t) -22804, PH.base.pack) ;
        p186_sue_trim_value_input_1_SET((int16_t)(int16_t) -9748, PH.base.pack) ;
        p186_sue_number_of_inputs_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p186_sue_trim_value_input_9_SET((int16_t)(int16_t) -873, PH.base.pack) ;
        p186_sue_trim_value_input_6_SET((int16_t)(int16_t) -10146, PH.base.pack) ;
        p186_sue_trim_value_input_3_SET((int16_t)(int16_t)10041, PH.base.pack) ;
        p186_sue_trim_value_input_10_SET((int16_t)(int16_t)23091, PH.base.pack) ;
        p186_sue_trim_value_input_7_SET((int16_t)(int16_t)19365, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F21_187(), &PH);
        p187_sue_accel_x_offset_SET((int16_t)(int16_t) -31144, PH.base.pack) ;
        p187_sue_gyro_x_offset_SET((int16_t)(int16_t)511, PH.base.pack) ;
        p187_sue_gyro_y_offset_SET((int16_t)(int16_t)403, PH.base.pack) ;
        p187_sue_accel_y_offset_SET((int16_t)(int16_t) -23238, PH.base.pack) ;
        p187_sue_gyro_z_offset_SET((int16_t)(int16_t)12157, PH.base.pack) ;
        p187_sue_accel_z_offset_SET((int16_t)(int16_t)925, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SERIAL_UDB_EXTRA_F22_188(), &PH);
        p188_sue_gyro_z_at_calibration_SET((int16_t)(int16_t)22426, PH.base.pack) ;
        p188_sue_accel_x_at_calibration_SET((int16_t)(int16_t)158, PH.base.pack) ;
        p188_sue_accel_z_at_calibration_SET((int16_t)(int16_t) -3519, PH.base.pack) ;
        p188_sue_gyro_x_at_calibration_SET((int16_t)(int16_t)4745, PH.base.pack) ;
        p188_sue_accel_y_at_calibration_SET((int16_t)(int16_t) -7514, PH.base.pack) ;
        p188_sue_gyro_y_at_calibration_SET((int16_t)(int16_t)12377, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_tas_ratio_SET((float)9.271696E37F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -1.4043517E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.4211544E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -3.208412E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)1.8176619E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -2.6104972E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)6421752012664311492L, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE), PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -7.858485E36F, PH.base.pack) ;
        p230_vel_ratio_SET((float)1.9546217E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_vert_accuracy_SET((float)9.319203E37F, PH.base.pack) ;
        p231_wind_x_SET((float)3.0694056E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -3.1657223E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)1.1121925E38F, PH.base.pack) ;
        p231_wind_y_SET((float)3.8438018E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -1.7783065E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.6857309E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)4196986020672640699L, PH.base.pack) ;
        p231_var_vert_SET((float)2.1938747E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_speed_accuracy_SET((float) -2.3760269E38F, PH.base.pack) ;
        p232_alt_SET((float)3.2669133E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)3511799328468511635L, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p232_lon_SET((int32_t)1564008716, PH.base.pack) ;
        p232_hdop_SET((float) -3.350911E38F, PH.base.pack) ;
        p232_ve_SET((float)3.260631E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.921051E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -1.1659512E38F, PH.base.pack) ;
        p232_lat_SET((int32_t)905536750, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)49682, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)1472989669L, PH.base.pack) ;
        p232_vd_SET((float)1.053728E38F, PH.base.pack) ;
        p232_vn_SET((float)6.191475E37F, PH.base.pack) ;
        p232_vdop_SET((float) -1.5335549E38F, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT), PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)158, (uint8_t)178, (uint8_t)173, (uint8_t)85, (uint8_t)63, (uint8_t)223, (uint8_t)159, (uint8_t)2, (uint8_t)21, (uint8_t)72, (uint8_t)152, (uint8_t)18, (uint8_t)220, (uint8_t)232, (uint8_t)46, (uint8_t)2, (uint8_t)43, (uint8_t)149, (uint8_t)109, (uint8_t)213, (uint8_t)26, (uint8_t)158, (uint8_t)154, (uint8_t)50, (uint8_t)180, (uint8_t)85, (uint8_t)251, (uint8_t)60, (uint8_t)175, (uint8_t)134, (uint8_t)243, (uint8_t)137, (uint8_t)9, (uint8_t)23, (uint8_t)77, (uint8_t)129, (uint8_t)227, (uint8_t)15, (uint8_t)69, (uint8_t)51, (uint8_t)110, (uint8_t)15, (uint8_t)53, (uint8_t)105, (uint8_t)35, (uint8_t)71, (uint8_t)6, (uint8_t)236, (uint8_t)47, (uint8_t)206, (uint8_t)89, (uint8_t)74, (uint8_t)115, (uint8_t)128, (uint8_t)231, (uint8_t)178, (uint8_t)30, (uint8_t)16, (uint8_t)253, (uint8_t)159, (uint8_t)254, (uint8_t)41, (uint8_t)80, (uint8_t)120, (uint8_t)184, (uint8_t)145, (uint8_t)114, (uint8_t)186, (uint8_t)215, (uint8_t)222, (uint8_t)67, (uint8_t)112, (uint8_t)125, (uint8_t)247, (uint8_t)251, (uint8_t)36, (uint8_t)71, (uint8_t)145, (uint8_t)27, (uint8_t)181, (uint8_t)142, (uint8_t)150, (uint8_t)47, (uint8_t)206, (uint8_t)44, (uint8_t)186, (uint8_t)47, (uint8_t)184, (uint8_t)208, (uint8_t)169, (uint8_t)44, (uint8_t)14, (uint8_t)237, (uint8_t)233, (uint8_t)62, (uint8_t)98, (uint8_t)162, (uint8_t)84, (uint8_t)96, (uint8_t)187, (uint8_t)141, (uint8_t)211, (uint8_t)131, (uint8_t)247, (uint8_t)132, (uint8_t)218, (uint8_t)160, (uint8_t)183, (uint8_t)114, (uint8_t)175, (uint8_t)143, (uint8_t)89, (uint8_t)19, (uint8_t)9, (uint8_t)52, (uint8_t)208, (uint8_t)22, (uint8_t)18, (uint8_t)144, (uint8_t)94, (uint8_t)56, (uint8_t)84, (uint8_t)8, (uint8_t)195, (uint8_t)216, (uint8_t)63, (uint8_t)218, (uint8_t)236, (uint8_t)185, (uint8_t)15, (uint8_t)43, (uint8_t)1, (uint8_t)176, (uint8_t)108, (uint8_t)222, (uint8_t)232, (uint8_t)250, (uint8_t)70, (uint8_t)194, (uint8_t)31, (uint8_t)251, (uint8_t)198, (uint8_t)122, (uint8_t)37, (uint8_t)30, (uint8_t)145, (uint8_t)219, (uint8_t)195, (uint8_t)222, (uint8_t)10, (uint8_t)46, (uint8_t)210, (uint8_t)166, (uint8_t)43, (uint8_t)91, (uint8_t)226, (uint8_t)52, (uint8_t)223, (uint8_t)47, (uint8_t)176, (uint8_t)161, (uint8_t)242, (uint8_t)243, (uint8_t)126, (uint8_t)23, (uint8_t)25, (uint8_t)95, (uint8_t)120, (uint8_t)147, (uint8_t)188, (uint8_t)159, (uint8_t)70, (uint8_t)147, (uint8_t)186, (uint8_t)191, (uint8_t)108, (uint8_t)153, (uint8_t)173, (uint8_t)95, (uint8_t)187};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
        p234_longitude_SET((int32_t) -536270369, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)28219, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -125, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -23, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)4284106940L, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)28650, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED), PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -3704, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -15095, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)98, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)23, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)54767, PH.base.pack) ;
        p234_latitude_SET((int32_t)1651012546, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)55160, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -5259, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
        p241_clipping_2_SET((uint32_t)3589554316L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)4105791902L, PH.base.pack) ;
        p241_vibration_y_SET((float) -7.9398083E37F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)299174393081430906L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.2192096E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)2644374815L, PH.base.pack) ;
        p241_vibration_z_SET((float) -2.893432E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
        p242_altitude_SET((int32_t)463939088, PH.base.pack) ;
        p242_latitude_SET((int32_t) -719201426, PH.base.pack) ;
        {
            float q[] =  {-2.5352432E38F, -5.816463E37F, 2.6609351E38F, -3.7874018E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_x_SET((float) -1.4318507E38F, PH.base.pack) ;
        p242_approach_z_SET((float)1.4386479E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -1.2114207E37F, PH.base.pack) ;
        p242_approach_x_SET((float) -1.2967198E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)6798795451552831650L, &PH) ;
        p242_z_SET((float)2.4910103E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t)43234928, PH.base.pack) ;
        p242_y_SET((float)1.2897297E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_altitude_SET((int32_t) -1546040692, PH.base.pack) ;
        p243_latitude_SET((int32_t)2009353739, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p243_y_SET((float) -1.4421474E37F, PH.base.pack) ;
        p243_x_SET((float)1.7123708E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -2.0279775E38F, PH.base.pack) ;
        {
            float q[] =  {1.8149595E38F, -1.57721E37F, -5.2640855E36F, -3.037708E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_x_SET((float)8.530378E36F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)3181728163473809569L, &PH) ;
        p243_longitude_SET((int32_t) -577430565, PH.base.pack) ;
        p243_z_SET((float)2.7407932E38F, PH.base.pack) ;
        p243_approach_z_SET((float) -2.7425142E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1304391521, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)63861, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_SET((int32_t)397460727, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3986461236L, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)54636, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p246_lon_SET((int32_t) -507945323, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY), PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)28020, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER, PH.base.pack) ;
        p246_lat_SET((int32_t)873098128, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -16983, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        {
            char16_t* callsign = u"vmqxvxalg";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_hor_velocity_SET((uint16_t)(uint16_t)35019, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_threat_level_SET((e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE), PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)2.4008035E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -3.1390507E37F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -1.7937818E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)3578437248L, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)87, (uint8_t)100, (uint8_t)60, (uint8_t)120, (uint8_t)39, (uint8_t)21, (uint8_t)208, (uint8_t)64, (uint8_t)159, (uint8_t)173, (uint8_t)86, (uint8_t)49, (uint8_t)134, (uint8_t)215, (uint8_t)239, (uint8_t)153, (uint8_t)61, (uint8_t)114, (uint8_t)177, (uint8_t)90, (uint8_t)223, (uint8_t)152, (uint8_t)177, (uint8_t)76, (uint8_t)133, (uint8_t)135, (uint8_t)32, (uint8_t)63, (uint8_t)235, (uint8_t)152, (uint8_t)213, (uint8_t)240, (uint8_t)133, (uint8_t)38, (uint8_t)213, (uint8_t)242, (uint8_t)99, (uint8_t)87, (uint8_t)16, (uint8_t)113, (uint8_t)197, (uint8_t)181, (uint8_t)188, (uint8_t)198, (uint8_t)67, (uint8_t)181, (uint8_t)160, (uint8_t)179, (uint8_t)8, (uint8_t)122, (uint8_t)246, (uint8_t)106, (uint8_t)73, (uint8_t)95, (uint8_t)49, (uint8_t)90, (uint8_t)177, (uint8_t)201, (uint8_t)45, (uint8_t)201, (uint8_t)138, (uint8_t)73, (uint8_t)86, (uint8_t)226, (uint8_t)154, (uint8_t)53, (uint8_t)247, (uint8_t)4, (uint8_t)61, (uint8_t)202, (uint8_t)132, (uint8_t)105, (uint8_t)143, (uint8_t)220, (uint8_t)142, (uint8_t)233, (uint8_t)113, (uint8_t)175, (uint8_t)86, (uint8_t)63, (uint8_t)29, (uint8_t)63, (uint8_t)115, (uint8_t)76, (uint8_t)28, (uint8_t)5, (uint8_t)105, (uint8_t)146, (uint8_t)230, (uint8_t)180, (uint8_t)5, (uint8_t)181, (uint8_t)157, (uint8_t)101, (uint8_t)219, (uint8_t)137, (uint8_t)105, (uint8_t)75, (uint8_t)39, (uint8_t)19, (uint8_t)253, (uint8_t)248, (uint8_t)150, (uint8_t)92, (uint8_t)193, (uint8_t)102, (uint8_t)112, (uint8_t)33, (uint8_t)169, (uint8_t)218, (uint8_t)131, (uint8_t)207, (uint8_t)200, (uint8_t)195, (uint8_t)254, (uint8_t)205, (uint8_t)102, (uint8_t)50, (uint8_t)170, (uint8_t)233, (uint8_t)75, (uint8_t)254, (uint8_t)148, (uint8_t)216, (uint8_t)253, (uint8_t)206, (uint8_t)70, (uint8_t)74, (uint8_t)7, (uint8_t)136, (uint8_t)252, (uint8_t)252, (uint8_t)193, (uint8_t)174, (uint8_t)147, (uint8_t)50, (uint8_t)170, (uint8_t)146, (uint8_t)174, (uint8_t)58, (uint8_t)163, (uint8_t)150, (uint8_t)174, (uint8_t)150, (uint8_t)196, (uint8_t)6, (uint8_t)77, (uint8_t)33, (uint8_t)6, (uint8_t)123, (uint8_t)27, (uint8_t)31, (uint8_t)223, (uint8_t)109, (uint8_t)116, (uint8_t)221, (uint8_t)21, (uint8_t)168, (uint8_t)36, (uint8_t)102, (uint8_t)56, (uint8_t)29, (uint8_t)107, (uint8_t)237, (uint8_t)89, (uint8_t)31, (uint8_t)234, (uint8_t)34, (uint8_t)254, (uint8_t)227, (uint8_t)83, (uint8_t)17, (uint8_t)3, (uint8_t)142, (uint8_t)158, (uint8_t)70, (uint8_t)150, (uint8_t)86, (uint8_t)161, (uint8_t)81, (uint8_t)201, (uint8_t)74, (uint8_t)112, (uint8_t)151, (uint8_t)165, (uint8_t)42, (uint8_t)115, (uint8_t)84, (uint8_t)203, (uint8_t)120, (uint8_t)110, (uint8_t)32, (uint8_t)159, (uint8_t)90, (uint8_t)37, (uint8_t)30, (uint8_t)210, (uint8_t)15, (uint8_t)115, (uint8_t)168, (uint8_t)17, (uint8_t)131, (uint8_t)88, (uint8_t)216, (uint8_t)196, (uint8_t)18, (uint8_t)196, (uint8_t)231, (uint8_t)17, (uint8_t)32, (uint8_t)47, (uint8_t)114, (uint8_t)204, (uint8_t)73, (uint8_t)164, (uint8_t)85, (uint8_t)168, (uint8_t)12, (uint8_t)222, (uint8_t)193, (uint8_t)92, (uint8_t)159, (uint8_t)67, (uint8_t)25, (uint8_t)35, (uint8_t)164, (uint8_t)238, (uint8_t)139, (uint8_t)71, (uint8_t)172, (uint8_t)253, (uint8_t)86, (uint8_t)19, (uint8_t)54, (uint8_t)50, (uint8_t)174, (uint8_t)85, (uint8_t)207, (uint8_t)203, (uint8_t)17, (uint8_t)93, (uint8_t)198, (uint8_t)9, (uint8_t)23, (uint8_t)68, (uint8_t)220, (uint8_t)184, (uint8_t)245, (uint8_t)1};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_message_type_SET((uint16_t)(uint16_t)19953, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)48000, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)87, (int8_t) -105, (int8_t)105, (int8_t)99, (int8_t)127, (int8_t)34, (int8_t)13, (int8_t) -35, (int8_t) -30, (int8_t)120, (int8_t) -10, (int8_t) -13, (int8_t) -59, (int8_t)126, (int8_t)108, (int8_t)25, (int8_t)0, (int8_t) -75, (int8_t) -20, (int8_t)34, (int8_t)22, (int8_t)51, (int8_t)93, (int8_t) -86, (int8_t)113, (int8_t)39, (int8_t)1, (int8_t) -24, (int8_t)27, (int8_t)92, (int8_t) -66, (int8_t)69};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float)3.2018215E38F, PH.base.pack) ;
        p250_y_SET((float)8.510319E37F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)8168709550551217930L, PH.base.pack) ;
        {
            char16_t* name = u"ssm";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float)6.7169626E37F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)2032895809L, PH.base.pack) ;
        {
            char16_t* name = u"h";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -2.5963955E38F, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)1763485774, PH.base.pack) ;
        {
            char16_t* name = u"dpRuf";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)3144508605L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"Grbpme";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_time_boot_ms_SET((uint32_t)4010610800L, PH.base.pack) ;
        p254_value_SET((float) -3.49838E37F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)228, (uint8_t)28, (uint8_t)211, (uint8_t)66, (uint8_t)129, (uint8_t)171, (uint8_t)26, (uint8_t)120, (uint8_t)137, (uint8_t)159, (uint8_t)229, (uint8_t)102, (uint8_t)63, (uint8_t)97, (uint8_t)217, (uint8_t)223, (uint8_t)204, (uint8_t)171, (uint8_t)214, (uint8_t)31, (uint8_t)187, (uint8_t)206, (uint8_t)239, (uint8_t)140, (uint8_t)112, (uint8_t)247, (uint8_t)99, (uint8_t)199, (uint8_t)4, (uint8_t)225, (uint8_t)237, (uint8_t)225};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)3575242703178803148L, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)3414827530L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)678527081L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        {
            char16_t* tune = u"dfvmctqzJbtbqXsoyfwynbfnymxu";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_v_SET((uint16_t)(uint16_t)15781, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)3464399766L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"leTmlyjoiTdghnsfuehHzqotrznarzxpxpirbsvzwzdreiphdqdmzgecsxxgqauzk";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_sensor_size_h_SET((float)7.5836584E37F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)47768, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -1.6800413E38F, PH.base.pack) ;
        p259_focal_length_SET((float) -1.8936035E38F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)200, (uint8_t)47, (uint8_t)184, (uint8_t)248, (uint8_t)155, (uint8_t)235, (uint8_t)211, (uint8_t)30, (uint8_t)238, (uint8_t)179, (uint8_t)174, (uint8_t)126, (uint8_t)6, (uint8_t)203, (uint8_t)232, (uint8_t)178, (uint8_t)56, (uint8_t)185, (uint8_t)167, (uint8_t)127, (uint8_t)58, (uint8_t)83, (uint8_t)135, (uint8_t)43, (uint8_t)170, (uint8_t)165, (uint8_t)56, (uint8_t)225, (uint8_t)177, (uint8_t)165, (uint8_t)250, (uint8_t)63};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE), PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)112, (uint8_t)253, (uint8_t)56, (uint8_t)157, (uint8_t)124, (uint8_t)141, (uint8_t)71, (uint8_t)95, (uint8_t)32, (uint8_t)193, (uint8_t)48, (uint8_t)162, (uint8_t)36, (uint8_t)152, (uint8_t)51, (uint8_t)196, (uint8_t)245, (uint8_t)88, (uint8_t)174, (uint8_t)3, (uint8_t)230, (uint8_t)146, (uint8_t)207, (uint8_t)249, (uint8_t)31, (uint8_t)53, (uint8_t)1, (uint8_t)42, (uint8_t)128, (uint8_t)10, (uint8_t)102, (uint8_t)190};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)21589, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)1921863614L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET((e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY), PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3026728645L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_time_boot_ms_SET((uint32_t)1591058872L, PH.base.pack) ;
        p261_used_capacity_SET((float)1.4094225E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p261_available_capacity_SET((float)7.8911336E37F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p261_write_speed_SET((float) -8.4147494E37F, PH.base.pack) ;
        p261_total_capacity_SET((float) -8.4666125E37F, PH.base.pack) ;
        p261_read_speed_SET((float)5.862127E37F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float)9.340211E37F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p262_available_capacity_SET((float)5.325937E37F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)1323995604L, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)349962624L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_camera_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p263_image_index_SET((int32_t)347978819, PH.base.pack) ;
        p263_lon_SET((int32_t)763489430, PH.base.pack) ;
        {
            char16_t* file_url = u"nqCfurpbafiSpqwUl";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_utc_SET((uint64_t)2573210771851268416L, PH.base.pack) ;
        p263_alt_SET((int32_t)587152532, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)1040048299, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)801565525L, PH.base.pack) ;
        {
            float q[] =  {-3.1414722E38F, 6.3983604E37F, 1.6361245E38F, 2.0770108E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_capture_result_SET((int8_t)(int8_t)93, PH.base.pack) ;
        p263_lat_SET((int32_t) -1491043280, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)856793907L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)3719202905706038695L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)6087857316651890716L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)2845270483614826946L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)1549602044L, PH.base.pack) ;
        p265_pitch_SET((float) -2.4678358E38F, PH.base.pack) ;
        p265_yaw_SET((float)1.1062974E38F, PH.base.pack) ;
        p265_roll_SET((float)1.455532E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)129, (uint8_t)33, (uint8_t)226, (uint8_t)129, (uint8_t)216, (uint8_t)13, (uint8_t)232, (uint8_t)13, (uint8_t)109, (uint8_t)65, (uint8_t)171, (uint8_t)169, (uint8_t)13, (uint8_t)247, (uint8_t)39, (uint8_t)237, (uint8_t)153, (uint8_t)192, (uint8_t)68, (uint8_t)159, (uint8_t)193, (uint8_t)0, (uint8_t)106, (uint8_t)185, (uint8_t)175, (uint8_t)88, (uint8_t)149, (uint8_t)206, (uint8_t)151, (uint8_t)72, (uint8_t)178, (uint8_t)103, (uint8_t)237, (uint8_t)10, (uint8_t)134, (uint8_t)23, (uint8_t)160, (uint8_t)59, (uint8_t)9, (uint8_t)67, (uint8_t)10, (uint8_t)110, (uint8_t)60, (uint8_t)120, (uint8_t)162, (uint8_t)13, (uint8_t)111, (uint8_t)137, (uint8_t)182, (uint8_t)252, (uint8_t)155, (uint8_t)185, (uint8_t)68, (uint8_t)247, (uint8_t)177, (uint8_t)6, (uint8_t)140, (uint8_t)144, (uint8_t)185, (uint8_t)109, (uint8_t)244, (uint8_t)153, (uint8_t)69, (uint8_t)134, (uint8_t)216, (uint8_t)186, (uint8_t)87, (uint8_t)163, (uint8_t)146, (uint8_t)37, (uint8_t)49, (uint8_t)107, (uint8_t)4, (uint8_t)93, (uint8_t)174, (uint8_t)36, (uint8_t)242, (uint8_t)165, (uint8_t)8, (uint8_t)202, (uint8_t)143, (uint8_t)25, (uint8_t)154, (uint8_t)156, (uint8_t)115, (uint8_t)47, (uint8_t)208, (uint8_t)231, (uint8_t)3, (uint8_t)83, (uint8_t)5, (uint8_t)130, (uint8_t)226, (uint8_t)2, (uint8_t)238, (uint8_t)41, (uint8_t)20, (uint8_t)46, (uint8_t)100, (uint8_t)84, (uint8_t)35, (uint8_t)153, (uint8_t)207, (uint8_t)82, (uint8_t)105, (uint8_t)59, (uint8_t)114, (uint8_t)213, (uint8_t)146, (uint8_t)20, (uint8_t)240, (uint8_t)45, (uint8_t)220, (uint8_t)106, (uint8_t)120, (uint8_t)90, (uint8_t)195, (uint8_t)38, (uint8_t)168, (uint8_t)169, (uint8_t)109, (uint8_t)47, (uint8_t)135, (uint8_t)248, (uint8_t)230, (uint8_t)233, (uint8_t)198, (uint8_t)175, (uint8_t)90, (uint8_t)180, (uint8_t)86, (uint8_t)103, (uint8_t)102, (uint8_t)222, (uint8_t)126, (uint8_t)2, (uint8_t)45, (uint8_t)148, (uint8_t)100, (uint8_t)213, (uint8_t)5, (uint8_t)238, (uint8_t)32, (uint8_t)171, (uint8_t)138, (uint8_t)65, (uint8_t)191, (uint8_t)220, (uint8_t)201, (uint8_t)55, (uint8_t)66, (uint8_t)103, (uint8_t)158, (uint8_t)249, (uint8_t)110, (uint8_t)167, (uint8_t)55, (uint8_t)227, (uint8_t)150, (uint8_t)15, (uint8_t)96, (uint8_t)200, (uint8_t)61, (uint8_t)129, (uint8_t)237, (uint8_t)29, (uint8_t)195, (uint8_t)79, (uint8_t)249, (uint8_t)132, (uint8_t)195, (uint8_t)200, (uint8_t)9, (uint8_t)63, (uint8_t)74, (uint8_t)196, (uint8_t)77, (uint8_t)6, (uint8_t)255, (uint8_t)223, (uint8_t)124, (uint8_t)122, (uint8_t)189, (uint8_t)110, (uint8_t)41, (uint8_t)188, (uint8_t)197, (uint8_t)79, (uint8_t)2, (uint8_t)73, (uint8_t)234, (uint8_t)34, (uint8_t)203, (uint8_t)147, (uint8_t)31, (uint8_t)157, (uint8_t)92, (uint8_t)121, (uint8_t)44, (uint8_t)232, (uint8_t)36, (uint8_t)217, (uint8_t)138, (uint8_t)240, (uint8_t)27, (uint8_t)114, (uint8_t)83, (uint8_t)90, (uint8_t)233, (uint8_t)171, (uint8_t)95, (uint8_t)194, (uint8_t)151, (uint8_t)198, (uint8_t)139, (uint8_t)128, (uint8_t)120, (uint8_t)85, (uint8_t)56, (uint8_t)71, (uint8_t)1, (uint8_t)175, (uint8_t)214, (uint8_t)107, (uint8_t)131, (uint8_t)5, (uint8_t)192, (uint8_t)187, (uint8_t)165, (uint8_t)207, (uint8_t)170, (uint8_t)84, (uint8_t)51, (uint8_t)200, (uint8_t)195, (uint8_t)229, (uint8_t)225, (uint8_t)123, (uint8_t)61, (uint8_t)189, (uint8_t)77, (uint8_t)114, (uint8_t)108, (uint8_t)42, (uint8_t)245, (uint8_t)129, (uint8_t)125, (uint8_t)117, (uint8_t)51};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)22749, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)8, (uint8_t)178, (uint8_t)249, (uint8_t)103, (uint8_t)64, (uint8_t)36, (uint8_t)157, (uint8_t)89, (uint8_t)245, (uint8_t)251, (uint8_t)0, (uint8_t)8, (uint8_t)219, (uint8_t)94, (uint8_t)119, (uint8_t)18, (uint8_t)229, (uint8_t)227, (uint8_t)176, (uint8_t)133, (uint8_t)77, (uint8_t)98, (uint8_t)54, (uint8_t)37, (uint8_t)138, (uint8_t)36, (uint8_t)181, (uint8_t)13, (uint8_t)115, (uint8_t)7, (uint8_t)26, (uint8_t)60, (uint8_t)83, (uint8_t)36, (uint8_t)3, (uint8_t)206, (uint8_t)93, (uint8_t)67, (uint8_t)174, (uint8_t)60, (uint8_t)180, (uint8_t)186, (uint8_t)70, (uint8_t)86, (uint8_t)22, (uint8_t)38, (uint8_t)170, (uint8_t)226, (uint8_t)143, (uint8_t)78, (uint8_t)186, (uint8_t)22, (uint8_t)135, (uint8_t)207, (uint8_t)187, (uint8_t)245, (uint8_t)201, (uint8_t)251, (uint8_t)142, (uint8_t)210, (uint8_t)70, (uint8_t)156, (uint8_t)81, (uint8_t)16, (uint8_t)17, (uint8_t)90, (uint8_t)148, (uint8_t)179, (uint8_t)147, (uint8_t)245, (uint8_t)164, (uint8_t)46, (uint8_t)7, (uint8_t)102, (uint8_t)23, (uint8_t)158, (uint8_t)56, (uint8_t)121, (uint8_t)109, (uint8_t)68, (uint8_t)112, (uint8_t)171, (uint8_t)124, (uint8_t)150, (uint8_t)149, (uint8_t)172, (uint8_t)52, (uint8_t)17, (uint8_t)189, (uint8_t)37, (uint8_t)196, (uint8_t)198, (uint8_t)240, (uint8_t)253, (uint8_t)132, (uint8_t)119, (uint8_t)48, (uint8_t)100, (uint8_t)56, (uint8_t)214, (uint8_t)76, (uint8_t)15, (uint8_t)135, (uint8_t)174, (uint8_t)230, (uint8_t)18, (uint8_t)44, (uint8_t)7, (uint8_t)169, (uint8_t)142, (uint8_t)151, (uint8_t)10, (uint8_t)135, (uint8_t)93, (uint8_t)101, (uint8_t)218, (uint8_t)67, (uint8_t)91, (uint8_t)100, (uint8_t)111, (uint8_t)169, (uint8_t)114, (uint8_t)139, (uint8_t)85, (uint8_t)192, (uint8_t)147, (uint8_t)36, (uint8_t)187, (uint8_t)60, (uint8_t)227, (uint8_t)139, (uint8_t)232, (uint8_t)220, (uint8_t)245, (uint8_t)113, (uint8_t)141, (uint8_t)245, (uint8_t)114, (uint8_t)1, (uint8_t)181, (uint8_t)233, (uint8_t)188, (uint8_t)35, (uint8_t)110, (uint8_t)37, (uint8_t)99, (uint8_t)14, (uint8_t)237, (uint8_t)180, (uint8_t)112, (uint8_t)65, (uint8_t)94, (uint8_t)189, (uint8_t)169, (uint8_t)127, (uint8_t)187, (uint8_t)172, (uint8_t)50, (uint8_t)45, (uint8_t)155, (uint8_t)180, (uint8_t)2, (uint8_t)144, (uint8_t)85, (uint8_t)217, (uint8_t)214, (uint8_t)250, (uint8_t)209, (uint8_t)35, (uint8_t)82, (uint8_t)214, (uint8_t)171, (uint8_t)215, (uint8_t)67, (uint8_t)136, (uint8_t)201, (uint8_t)168, (uint8_t)53, (uint8_t)72, (uint8_t)101, (uint8_t)153, (uint8_t)223, (uint8_t)254, (uint8_t)133, (uint8_t)228, (uint8_t)155, (uint8_t)174, (uint8_t)95, (uint8_t)208, (uint8_t)106, (uint8_t)162, (uint8_t)196, (uint8_t)117, (uint8_t)96, (uint8_t)10, (uint8_t)210, (uint8_t)64, (uint8_t)175, (uint8_t)210, (uint8_t)217, (uint8_t)248, (uint8_t)159, (uint8_t)35, (uint8_t)140, (uint8_t)3, (uint8_t)177, (uint8_t)1, (uint8_t)2, (uint8_t)93, (uint8_t)201, (uint8_t)94, (uint8_t)248, (uint8_t)1, (uint8_t)16, (uint8_t)152, (uint8_t)142, (uint8_t)164, (uint8_t)146, (uint8_t)185, (uint8_t)212, (uint8_t)169, (uint8_t)85, (uint8_t)221, (uint8_t)126, (uint8_t)2, (uint8_t)29, (uint8_t)175, (uint8_t)189, (uint8_t)173, (uint8_t)247, (uint8_t)137, (uint8_t)184, (uint8_t)41, (uint8_t)230, (uint8_t)207, (uint8_t)89, (uint8_t)42, (uint8_t)116, (uint8_t)144, (uint8_t)219, (uint8_t)59, (uint8_t)83, (uint8_t)227, (uint8_t)215, (uint8_t)61, (uint8_t)143, (uint8_t)17, (uint8_t)182, (uint8_t)122};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_length_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)56728, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)9052, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_rotation_SET((uint16_t)(uint16_t)62567, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)58392, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)56799, PH.base.pack) ;
        p269_framerate_SET((float)1.6167486E38F, PH.base.pack) ;
        {
            char16_t* uri = u"zajlrhpsxSwbdknhvpyvxnzfuwvyqbjrNqknbMdBjSdrqkhnnekfIYkwueUownuuvwkkNrxxjdgkOxzxqpnddaopdEktqegzhAwyauzVfmzroslqRTjjhgcHvcYwzvdlsvuuuUebdlzhxhddlpoqnrzhazareaqmdxErhthqtzlrjslythjgjisdmbail";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_camera_id_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)342645355L, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_target_component_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)59881, PH.base.pack) ;
        p270_framerate_SET((float)6.7623196E37F, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)33842, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)3167113745L, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)26669, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        {
            char16_t* uri = u"hipWNuysdtzj";
            p270_uri_SET_(uri, &PH) ;
        }
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"kdVvmjwsoiivlxGHzmwablO";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"cihpflhwrvdSvbmcssfpagdfGcrpnfcxjy";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)5085, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)228, (uint8_t)90, (uint8_t)165, (uint8_t)30, (uint8_t)7, (uint8_t)158, (uint8_t)147, (uint8_t)227};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)10288, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)0, (uint8_t)186, (uint8_t)148, (uint8_t)60, (uint8_t)144, (uint8_t)235, (uint8_t)164, (uint8_t)163};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)37967, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)420361618L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)5947519773673961516L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)49115, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)6554167624591943630L, PH.base.pack) ;
        {
            char16_t* name = u"ehgmrysslhfVwrjtsidbLmdbmffukcxwsDddmZmxcduMfdGhzkgylwtliXPeiUwelovuogdambc";
            p311_name_SET_(name, &PH) ;
        }
        {
            uint8_t hw_unique_id[] =  {(uint8_t)135, (uint8_t)27, (uint8_t)239, (uint8_t)70, (uint8_t)253, (uint8_t)248, (uint8_t)190, (uint8_t)127, (uint8_t)189, (uint8_t)222, (uint8_t)234, (uint8_t)225, (uint8_t)243, (uint8_t)156, (uint8_t)168, (uint8_t)197};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_hw_version_major_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2930772623L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)1805612457L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"ojtozFkyZvc";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)24475, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"sqvmlaDj";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)60662, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)42307, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        {
            char16_t* param_value = u"ienwnj";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        {
            char16_t* param_id = u"EFqKlibjrbt";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        {
            char16_t* param_value = u"gmakoxtmabfhDvqiqbrovkupyssMerzrzubttppuDbkejlXsippnaeMtdMiphhgcykhymxudlsEnUtxyemtpjvqtjAbilkboirfykjcvcytaYxvqp";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"odlptsq";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"izbbemuoclEwg";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)19932, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)28055, (uint16_t)42202, (uint16_t)1504, (uint16_t)12912, (uint16_t)57723, (uint16_t)27801, (uint16_t)14376, (uint16_t)49039, (uint16_t)47263, (uint16_t)1540, (uint16_t)21711, (uint16_t)21474, (uint16_t)37339, (uint16_t)59446, (uint16_t)46844, (uint16_t)36472, (uint16_t)13912, (uint16_t)13515, (uint16_t)21312, (uint16_t)11463, (uint16_t)21135, (uint16_t)50268, (uint16_t)22053, (uint16_t)23764, (uint16_t)11052, (uint16_t)2415, (uint16_t)5039, (uint16_t)52981, (uint16_t)15271, (uint16_t)52588, (uint16_t)65045, (uint16_t)19325, (uint16_t)49712, (uint16_t)10499, (uint16_t)43459, (uint16_t)16700, (uint16_t)25220, (uint16_t)52239, (uint16_t)36665, (uint16_t)63704, (uint16_t)20964, (uint16_t)64636, (uint16_t)27289, (uint16_t)1491, (uint16_t)61145, (uint16_t)60927, (uint16_t)47733, (uint16_t)64704, (uint16_t)43039, (uint16_t)42249, (uint16_t)29042, (uint16_t)911, (uint16_t)58329, (uint16_t)42570, (uint16_t)41032, (uint16_t)32598, (uint16_t)5829, (uint16_t)45083, (uint16_t)65464, (uint16_t)480, (uint16_t)16834, (uint16_t)38004, (uint16_t)49565, (uint16_t)26910, (uint16_t)45423, (uint16_t)17804, (uint16_t)56512, (uint16_t)62908, (uint16_t)47607, (uint16_t)33836, (uint16_t)34148, (uint16_t)44924};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)6446429035326580663L, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)49265, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


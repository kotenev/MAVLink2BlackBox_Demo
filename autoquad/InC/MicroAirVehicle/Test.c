
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
    return  _en__X(get_bits(data, 40, 4));
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
    return  _en__G(get_bits(data, 276, 8));
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
    return  _en__G(get_bits(data, 276, 8));
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
INLINER void p140_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	 this field to difference between instances*/
INLINER void p140_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	 mixer to repurpose them as generic outputs*/
INLINER void p140_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  9, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
Pack * c_TEST_Channel_new_ACTUATOR_CONTROL_TARGET_140()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 140));
};
INLINER void p141_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
*	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
*	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
*	 time. This altitude will also drift and vary between flights*/
INLINER void p141_altitude_monotonic_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
/**
*This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
*	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
*	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
*	 by default and not the WGS84 altitude*/
INLINER void p141_altitude_amsl_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
/**
*This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
*	 to the coordinate origin (0, 0, 0). It is up-positive*/
INLINER void p141_altitude_local_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p141_altitude_relative_SET(float  src, Pack * dst)//This is the altitude above the home position. It resets on each change of the current home position
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
/**
*This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
*	 than -1000 should be interpreted as unknown*/
INLINER void p141_altitude_terrain_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
/**
*This is not the altitude, but the clear space below the system according to the fused clearance estimate.
*	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
*	 target. A negative value indicates no measurement available*/
INLINER void p141_bottom_clearance_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
Pack * c_TEST_Channel_new_ALTITUDE_141()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 141));
};
INLINER void p142_request_id_SET(uint8_t  src, Pack * dst)//Request ID. This ID should be re-used when sending back URI contents
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p142_uri_type_SET(uint8_t  src, Pack * dst)//The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
/**
*The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
*	 on the URI type enum*/
INLINER void p142_uri_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p142_transfer_type_SET(uint8_t  src, Pack * dst)//The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  122);
}
/**
*The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
*	 has a storage associated (e.g. MAVLink FTP)*/
INLINER void p142_storage_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  123, src_max = pos + 120; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_RESOURCE_REQUEST_142()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 142));
};
INLINER void p143_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp (milliseconds since system boot)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p143_press_abs_SET(float  src, Pack * dst)//Absolute pressure (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p143_press_diff_SET(float  src, Pack * dst)//Differential pressure 1 (hectopascal)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p143_temperature_SET(int16_t  src, Pack * dst)//Temperature measurement (0.01 degrees celsius)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
Pack * c_TEST_Channel_new_SCALED_PRESSURE3_143()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 143));
};
INLINER void p144_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp in milliseconds since system boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p144_custom_state_SET(uint64_t  src, Pack * dst)//button states or switches of a tracker device
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p144_est_capabilities_SET(uint8_t  src, Pack * dst)//bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
INLINER void p144_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  17);
}
INLINER void p144_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  21);
}
INLINER void p144_alt_SET(float  src, Pack * dst)//AMSL, in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER void p144_vel_SET(float*  src, int32_t pos, Pack * dst) //target velocity (0,0,0) for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p144_acc_SET(float*  src, int32_t pos, Pack * dst) //linear target acceleration (0,0,0) for unknown
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  41, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p144_attitude_q_SET(float*  src, int32_t pos, Pack * dst) //(1 0 0 0 for unknown)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  53, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p144_rates_SET(float*  src, int32_t pos, Pack * dst) //(0 0 0 for unknown)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  69, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p144_position_cov_SET(float*  src, int32_t pos, Pack * dst) //eph epv
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  81, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
Pack * c_TEST_Channel_new_FOLLOW_TARGET_144()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 144));
};
INLINER void p146_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p146_x_acc_SET(float  src, Pack * dst)//X acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p146_y_acc_SET(float  src, Pack * dst)//Y acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p146_z_acc_SET(float  src, Pack * dst)//Z acceleration in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p146_x_vel_SET(float  src, Pack * dst)//X velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p146_y_vel_SET(float  src, Pack * dst)//Y velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p146_z_vel_SET(float  src, Pack * dst)//Z velocity in body frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p146_x_pos_SET(float  src, Pack * dst)//X position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p146_y_pos_SET(float  src, Pack * dst)//Y position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p146_z_pos_SET(float  src, Pack * dst)//Z position in local frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p146_airspeed_SET(float  src, Pack * dst)//Airspeed, set to -1 if unknown
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p146_vel_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance of body velocity estimate
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  48, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p146_pos_variance_SET(float*  src, int32_t pos, Pack * dst) //Variance in local position
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  60, src_max = pos + 3; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p146_q_SET(float*  src, int32_t pos, Pack * dst) //The attitude, represented as Quaternion
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  72, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p146_roll_rate_SET(float  src, Pack * dst)//Angular rate in roll axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  88);
}
INLINER void p146_pitch_rate_SET(float  src, Pack * dst)//Angular rate in pitch axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  92);
}
INLINER void p146_yaw_rate_SET(float  src, Pack * dst)//Angular rate in yaw axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  96);
}
Pack * c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 146));
};/**
*Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
*	 should have the UINT16_MAX value*/
INLINER void p147_voltages_SET(uint16_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 10; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p147_id_SET(uint8_t  src, Pack * dst)//Battery ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  20);
}
INLINER void p147_temperature_SET(int16_t  src, Pack * dst)//Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  21);
}
INLINER void p147_current_battery_SET(int16_t  src, Pack * dst)//Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER void p147_current_consumed_SET(int32_t  src, Pack * dst)//Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  25);
}
/**
*Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
*	 energy consumption estimat*/
INLINER void p147_energy_consumed_SET(int32_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  29);
}
INLINER void p147_battery_remaining_SET(int8_t  src, Pack * dst)//Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER void p147_battery_function_SET(e_MAV_BATTERY_FUNCTION  src, Pack * dst)//Function of the battery
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 272);
}
INLINER void p147_type_SET(e_MAV_BATTERY_TYPE  src, Pack * dst)//Type (chemistry) of the battery
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 275);
}
Pack * c_TEST_Channel_new_BATTERY_STATUS_147()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 147));
};
INLINER void p148_vendor_id_SET(uint16_t  src, Pack * dst)//ID of the board vendor
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p148_product_id_SET(uint16_t  src, Pack * dst)//ID of the product
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p148_flight_sw_version_SET(uint32_t  src, Pack * dst)//Firmware version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p148_middleware_sw_version_SET(uint32_t  src, Pack * dst)//Middleware version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER void p148_os_sw_version_SET(uint32_t  src, Pack * dst)//Operating system version number
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  12);
}
INLINER void p148_board_version_SET(uint32_t  src, Pack * dst)//HW / board version (last 8 bytes should be silicon ID, if any)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  16);
}
INLINER void p148_uid_SET(uint64_t  src, Pack * dst)//UID if provided by hardware (see uid2)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  20);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_flight_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_middleware_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	 should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_os_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY  src, Pack * dst)//bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 17, data, 416);
}
/**
*UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
*	 use uid*/
INLINER void p148_uid2_SET(uint8_t*  src, int32_t pos, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 433)insert_field(dst, 433, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 18; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_AUTOPILOT_VERSION_148()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 148));
};
INLINER void p149_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p149_target_num_SET(uint8_t  src, Pack * dst)//The ID of the target if multiple targets are present
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p149_angle_x_SET(float  src, Pack * dst)//X-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER void p149_angle_y_SET(float  src, Pack * dst)//Y-axis angular offset (in radians) of the target from the center of the image
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER void p149_distance_SET(float  src, Pack * dst)//Distance to the target from the vehicle in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER void p149_size_x_SET(float  src, Pack * dst)//Size in radians of target along x-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER void p149_size_y_SET(float  src, Pack * dst)//Size in radians of target along y-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER void p149_frame_SET(e_MAV_FRAME  src, Pack * dst)//MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 232);
}
INLINER void p149_type_SET(e_LANDING_TARGET_TYPE  src, Pack * dst)//LANDING_TARGET_TYPE enum specifying the type of landing target
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 236);
}
INLINER void p149_x_SET(float  src, Bounds_Inside * dst)//X Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 238)insert_field(dst, 238, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_y_SET(float  src, Bounds_Inside * dst)//Y Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_z_SET(float  src, Bounds_Inside * dst)//Z Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_q_SET(float*  src, int32_t pos, Bounds_Inside * dst) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    if(dst->base.field_bit != 241)insert_field(dst, 241, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	 the landing targe*/
INLINER void p149_position_valid_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 242)insert_field(dst, 242, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 1, data,  dst->BYTE);
}
Pack * c_TEST_Channel_new_LANDING_TARGET_149()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 149));
};
INLINER void p150_Index_SET(uint16_t  src, Pack * dst)//Index of message
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p150_value1_SET(float  src, Pack * dst)//value1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p150_value2_SET(float  src, Pack * dst)//value2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p150_value3_SET(float  src, Pack * dst)//value3
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p150_value4_SET(float  src, Pack * dst)//value4
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p150_value5_SET(float  src, Pack * dst)//value5
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p150_value6_SET(float  src, Pack * dst)//value6
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p150_value7_SET(float  src, Pack * dst)//value7
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p150_value8_SET(float  src, Pack * dst)//value8
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p150_value9_SET(float  src, Pack * dst)//value9
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p150_value10_SET(float  src, Pack * dst)//value10
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER void p150_value11_SET(float  src, Pack * dst)//value11
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
INLINER void p150_value12_SET(float  src, Pack * dst)//value12
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  46);
}
INLINER void p150_value13_SET(float  src, Pack * dst)//value13
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  50);
}
INLINER void p150_value14_SET(float  src, Pack * dst)//value14
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  54);
}
INLINER void p150_value15_SET(float  src, Pack * dst)//value15
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  58);
}
INLINER void p150_value16_SET(float  src, Pack * dst)//value16
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  62);
}
INLINER void p150_value17_SET(float  src, Pack * dst)//value17
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  66);
}
INLINER void p150_value18_SET(float  src, Pack * dst)//value18
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  70);
}
INLINER void p150_value19_SET(float  src, Pack * dst)//value19
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  74);
}
INLINER void p150_value20_SET(float  src, Pack * dst)//value20
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  78);
}
Pack * c_TEST_Channel_new_AQ_TELEMETRY_F_150()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 150));
};
INLINER void p152_status_age_SET(uint16_t*  src, int32_t pos, Pack * dst) //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p152_time_boot_ms_SET(uint32_t  src, Pack * dst)//Timestamp of the component clock since boot time in ms.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER void p152_data0_SET(uint32_t*  src, int32_t pos, Pack * dst) //Data bits 1-32 for each ESC.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  12, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p152_data1_SET(uint32_t*  src, int32_t pos, Pack * dst) //Data bits 33-64 for each ESC.
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p152_seq_SET(uint8_t  src, Pack * dst)//Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  44);
}
INLINER void p152_num_motors_SET(uint8_t  src, Pack * dst)//Total number of active ESCs/motors on the system.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  45);
}
INLINER void p152_num_in_seq_SET(uint8_t  src, Pack * dst)//Number of active ESCs in this sequence (1 through this many array members will be populated with data
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  46);
}
INLINER void p152_escid_SET(uint8_t*  src, int32_t pos, Pack * dst) //ESC/Motor ID
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  47, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p152_data_version_SET(uint8_t*  src, int32_t pos, Pack * dst) //Version of data structure (determines contents).
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  51, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_AQ_ESC_TELEMETRY_152()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 152));
};
INLINER void p230_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p230_vel_ratio_SET(float  src, Pack * dst)//Velocity innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p230_pos_horiz_ratio_SET(float  src, Pack * dst)//Horizontal position innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p230_pos_vert_ratio_SET(float  src, Pack * dst)//Vertical position innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p230_mag_ratio_SET(float  src, Pack * dst)//Magnetometer innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p230_hagl_ratio_SET(float  src, Pack * dst)//Height above terrain innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p230_tas_ratio_SET(float  src, Pack * dst)//True airspeed innovation test ratio
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p230_pos_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p230_pos_vert_accuracy_SET(float  src, Pack * dst)//Vertical position 1-STD accuracy relative to the EKF local origin (m)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS  src, Pack * dst)//Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 11, data, 320);
}
Pack * c_TEST_Channel_new_ESTIMATOR_STATUS_230()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 230));
};
INLINER void p231_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p231_wind_x_SET(float  src, Pack * dst)//Wind in X (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p231_wind_y_SET(float  src, Pack * dst)//Wind in Y (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p231_wind_z_SET(float  src, Pack * dst)//Wind in Z (NED) direction in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p231_var_horiz_SET(float  src, Pack * dst)//Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p231_var_vert_SET(float  src, Pack * dst)//Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p231_wind_alt_SET(float  src, Pack * dst)//AMSL altitude (m) this measurement was taken at
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p231_horiz_accuracy_SET(float  src, Pack * dst)//Horizontal speed 1-STD accuracy
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p231_vert_accuracy_SET(float  src, Pack * dst)//Vertical speed 1-STD accuracy
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
Pack * c_TEST_Channel_new_WIND_COV_231()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 231));
};
INLINER void p232_time_week_SET(uint16_t  src, Pack * dst)//GPS week number
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p232_time_week_ms_SET(uint32_t  src, Pack * dst)//GPS time (milliseconds from start of GPS week)
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p232_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER void p232_gps_id_SET(uint8_t  src, Pack * dst)//ID of the GPS for multiple GPS inputs
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER void p232_fix_type_SET(uint8_t  src, Pack * dst)//0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
INLINER void p232_lat_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  16);
}
INLINER void p232_lon_SET(int32_t  src, Pack * dst)//Longitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  20);
}
INLINER void p232_alt_SET(float  src, Pack * dst)//Altitude (AMSL, not WGS84), in m (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p232_hdop_SET(float  src, Pack * dst)//GPS HDOP horizontal dilution of position in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p232_vdop_SET(float  src, Pack * dst)//GPS VDOP vertical dilution of position in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p232_vn_SET(float  src, Pack * dst)//GPS velocity in m/s in NORTH direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p232_ve_SET(float  src, Pack * dst)//GPS velocity in m/s in EAST direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p232_vd_SET(float  src, Pack * dst)//GPS velocity in m/s in DOWN direction in earth-fixed NED frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p232_speed_accuracy_SET(float  src, Pack * dst)//GPS speed accuracy in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER void p232_horiz_accuracy_SET(float  src, Pack * dst)//GPS horizontal accuracy in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER void p232_vert_accuracy_SET(float  src, Pack * dst)//GPS vertical accuracy in m
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER void p232_satellites_visible_SET(uint8_t  src, Pack * dst)//Number of satellites visible.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  60);
}
INLINER void p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS  src, Pack * dst)//Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 8, data, 488);
}
Pack * c_TEST_Channel_new_GPS_INPUT_232()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 232));
};/**
*LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
*	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
INLINER void p233_flags_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p233_len_SET(uint8_t  src, Pack * dst)//data length
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p233_data__SET(uint8_t*  src, int32_t pos, Pack * dst) //RTCM message (may be fragmented)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 180; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_GPS_RTCM_DATA_233()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 233));
};
INLINER void p234_heading_SET(uint16_t  src, Pack * dst)//heading (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p234_wp_distance_SET(uint16_t  src, Pack * dst)//distance to target (meters)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p234_custom_mode_SET(uint32_t  src, Pack * dst)//A bitfield for use for autopilot-specific flags.
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p234_roll_SET(int16_t  src, Pack * dst)//roll (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p234_pitch_SET(int16_t  src, Pack * dst)//pitch (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  10);
}
INLINER void p234_throttle_SET(int8_t  src, Pack * dst)//throttle (percentage)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  12);
}
INLINER void p234_heading_sp_SET(int16_t  src, Pack * dst)//heading setpoint (centidegrees)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  13);
}
INLINER void p234_latitude_SET(int32_t  src, Pack * dst)//Latitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  15);
}
INLINER void p234_longitude_SET(int32_t  src, Pack * dst)//Longitude, expressed as degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  19);
}
INLINER void p234_altitude_amsl_SET(int16_t  src, Pack * dst)//Altitude above mean sea level (meters)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  23);
}
INLINER void p234_altitude_sp_SET(int16_t  src, Pack * dst)//Altitude setpoint relative to the home position (meters)
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  25);
}
INLINER void p234_airspeed_SET(uint8_t  src, Pack * dst)//airspeed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  27);
}
INLINER void p234_airspeed_sp_SET(uint8_t  src, Pack * dst)//airspeed setpoint (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  28);
}
INLINER void p234_groundspeed_SET(uint8_t  src, Pack * dst)//groundspeed (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  29);
}
INLINER void p234_climb_rate_SET(int8_t  src, Pack * dst)//climb rate (m/s)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  30);
}
INLINER void p234_gps_nsat_SET(uint8_t  src, Pack * dst)//Number of satellites visible. If unknown, set to 255
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
INLINER void p234_battery_remaining_SET(uint8_t  src, Pack * dst)//Remaining battery (percentage)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
INLINER void p234_temperature_SET(int8_t  src, Pack * dst)//Autopilot temperature (degrees C)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  33);
}
INLINER void p234_temperature_air_SET(int8_t  src, Pack * dst)//Air temperature (degrees C) from airspeed sensor
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  34);
}
/**
*failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
*	 bit3:GCS, bit4:fence*/
INLINER void p234_failsafe_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  35);
}
INLINER void p234_wp_num_SET(uint8_t  src, Pack * dst)//current waypoint number
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  36);
}
INLINER void p234_base_mode_SET(e_MAV_MODE_FLAG  src, Pack * dst)//System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
{
    uint8_t * data = dst->data;
    set_bits(- 1 +   src, 8, data, 296);
}
INLINER void p234_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 304);
}
INLINER void p234_gps_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 307);
}
Pack * c_TEST_Channel_new_HIGH_LATENCY_234()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 234));
};
INLINER void p241_clipping_0_SET(uint32_t  src, Pack * dst)//first accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p241_clipping_1_SET(uint32_t  src, Pack * dst)//second accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p241_clipping_2_SET(uint32_t  src, Pack * dst)//third accelerometer clipping count
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  8);
}
INLINER void p241_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  12);
}
INLINER void p241_vibration_x_SET(float  src, Pack * dst)//Vibration levels on X-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p241_vibration_y_SET(float  src, Pack * dst)//Vibration levels on Y-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p241_vibration_z_SET(float  src, Pack * dst)//Vibration levels on Z-axis
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
Pack * c_TEST_Channel_new_VIBRATION_241()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 241));
};
INLINER void p242_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  0);
}
INLINER void p242_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  4);
}
INLINER void p242_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  8);
}
INLINER void p242_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p242_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p242_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	 and slope of the groun*/
INLINER void p242_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p242_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p242_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p242_approach_z_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER void p242_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 416)insert_field(dst, 416, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
Pack * c_TEST_Channel_new_HOME_POSITION_242()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 242));
};
INLINER void p243_target_system_SET(uint8_t  src, Pack * dst)//System ID.
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p243_latitude_SET(int32_t  src, Pack * dst)//Latitude (WGS84), in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  1);
}
INLINER void p243_longitude_SET(int32_t  src, Pack * dst)//Longitude (WGS84, in degrees * 1E7
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  5);
}
INLINER void p243_altitude_SET(int32_t  src, Pack * dst)//Altitude (AMSL), in meters * 1000 (positive for up)
{
    uint8_t * data = dst->data;
    set_bytes((uint32_t)(src), 4, data,  9);
}
INLINER void p243_x_SET(float  src, Pack * dst)//Local X position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER void p243_y_SET(float  src, Pack * dst)//Local Y position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER void p243_z_SET(float  src, Pack * dst)//Local Z position of this position in the local coordinate frame
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
/**
*World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
*	 and slope of the groun*/
INLINER void p243_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p243_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p243_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	 from the threshold / touchdown zone*/
INLINER void p243_approach_z_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  49);
}
INLINER void p243_time_usec_SET(uint64_t  src, Bounds_Inside * dst)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
{
    if(dst->base.field_bit != 424)insert_field(dst, 424, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes((src), 8, data,  dst->BYTE);
}
Pack * c_TEST_Channel_new_SET_HOME_POSITION_243()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 243));
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
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED));
    assert(p0_custom_mode_GET(pack) == (uint32_t)3987814143L);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CRITICAL);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -51);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)40795);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)34646);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)16898);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)18270);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)52738);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG));
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -12968);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)37769);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)44729);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)39756);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)271560254L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)7149049800920340969L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vy_GET(pack) == (float)9.359445E37F);
    assert(p3_yaw_rate_GET(pack) == (float)4.842398E37F);
    assert(p3_afx_GET(pack) == (float) -4.3526837E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)4056374365L);
    assert(p3_vz_GET(pack) == (float) -1.1321557E38F);
    assert(p3_yaw_GET(pack) == (float)4.8177487E37F);
    assert(p3_afy_GET(pack) == (float)2.8962591E38F);
    assert(p3_vx_GET(pack) == (float)3.3343386E37F);
    assert(p3_y_GET(pack) == (float)2.5373217E38F);
    assert(p3_x_GET(pack) == (float)1.6999316E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)370);
    assert(p3_z_GET(pack) == (float) -2.0352122E38F);
    assert(p3_afz_GET(pack) == (float)2.025005E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)7454558731081332350L);
    assert(p4_seq_GET(pack) == (uint32_t)4241381451L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)162);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p5_passkey_LEN(ph) == 20);
    {
        char16_t * exemplary = u"hyjkchhawvjsiwHytomv";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)85);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)210);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 13);
    {
        char16_t * exemplary = u"XPqaHaDodIQgo";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)4232978769L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -6913);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p20_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"veixYmgsybn";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p22_param_value_GET(pack) == (float) -1.4223395E37F);
    assert(p22_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"tbcuJ";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)25632);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)7274);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p23_param_value_GET(pack) == (float) -1.7118003E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p23_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"NtfSxh";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)16941);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3115386608L);
    assert(p24_alt_GET(pack) == (int32_t)1613165542);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)11051);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)30608);
    assert(p24_time_usec_GET(pack) == (uint64_t)4883894581440238095L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1783650429L);
    assert(p24_lon_GET(pack) == (int32_t) -944980178);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)3554);
    assert(p24_lat_GET(pack) == (int32_t) -237063330);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p24_v_acc_TRY(ph) == (uint32_t)72307326L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2417298517L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -661520520);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)192, (uint8_t)67, (uint8_t)126, (uint8_t)68, (uint8_t)146, (uint8_t)83, (uint8_t)171, (uint8_t)69, (uint8_t)195, (uint8_t)244, (uint8_t)190, (uint8_t)174, (uint8_t)166, (uint8_t)88, (uint8_t)63, (uint8_t)152, (uint8_t)11, (uint8_t)35, (uint8_t)169, (uint8_t)124} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)170, (uint8_t)50, (uint8_t)35, (uint8_t)131, (uint8_t)227, (uint8_t)128, (uint8_t)169, (uint8_t)87, (uint8_t)247, (uint8_t)172, (uint8_t)219, (uint8_t)254, (uint8_t)28, (uint8_t)239, (uint8_t)81, (uint8_t)116, (uint8_t)234, (uint8_t)76, (uint8_t)181, (uint8_t)175} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)12, (uint8_t)134, (uint8_t)85, (uint8_t)188, (uint8_t)195, (uint8_t)239, (uint8_t)173, (uint8_t)54, (uint8_t)0, (uint8_t)49, (uint8_t)207, (uint8_t)133, (uint8_t)18, (uint8_t)116, (uint8_t)123, (uint8_t)172, (uint8_t)131, (uint8_t)39, (uint8_t)209, (uint8_t)110} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)178, (uint8_t)167, (uint8_t)127, (uint8_t)155, (uint8_t)170, (uint8_t)73, (uint8_t)238, (uint8_t)68, (uint8_t)249, (uint8_t)169, (uint8_t)8, (uint8_t)218, (uint8_t)80, (uint8_t)204, (uint8_t)254, (uint8_t)76, (uint8_t)222, (uint8_t)15, (uint8_t)143, (uint8_t)166} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)132, (uint8_t)129, (uint8_t)120, (uint8_t)54, (uint8_t)80, (uint8_t)166, (uint8_t)71, (uint8_t)96, (uint8_t)62, (uint8_t)60, (uint8_t)52, (uint8_t)196, (uint8_t)114, (uint8_t)253, (uint8_t)38, (uint8_t)116, (uint8_t)247, (uint8_t)15, (uint8_t)153, (uint8_t)189} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)166);
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)9708);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)18130);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -20548);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)20975);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)9190);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -17989);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -5323);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3949195841L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)4470);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -19728);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)2207);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)18098);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -21018);
    assert(p27_time_usec_GET(pack) == (uint64_t)5206282137199934679L);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -31699);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -21620);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -205);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)18720);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)1310);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)748);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)1688610272083548404L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)18);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -23042);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)6919);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -29528);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_diff_GET(pack) == (float) -1.9821772E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)4259);
    assert(p29_press_abs_GET(pack) == (float)2.9046435E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3276141460L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitch_GET(pack) == (float)7.528729E37F);
    assert(p30_pitchspeed_GET(pack) == (float)2.3555457E38F);
    assert(p30_rollspeed_GET(pack) == (float)1.6823054E38F);
    assert(p30_yawspeed_GET(pack) == (float)1.6192204E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)1568594698L);
    assert(p30_roll_GET(pack) == (float) -8.611308E37F);
    assert(p30_yaw_GET(pack) == (float)2.2750381E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q2_GET(pack) == (float) -1.9485037E38F);
    assert(p31_yawspeed_GET(pack) == (float) -1.290549E38F);
    assert(p31_rollspeed_GET(pack) == (float) -2.0625866E38F);
    assert(p31_pitchspeed_GET(pack) == (float)2.4810038E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1343082906L);
    assert(p31_q1_GET(pack) == (float)2.4568673E38F);
    assert(p31_q3_GET(pack) == (float)9.720614E37F);
    assert(p31_q4_GET(pack) == (float)3.2381484E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)527139452L);
    assert(p32_vy_GET(pack) == (float) -1.8647647E38F);
    assert(p32_vx_GET(pack) == (float) -3.0495612E37F);
    assert(p32_z_GET(pack) == (float) -7.559855E37F);
    assert(p32_vz_GET(pack) == (float)2.4358691E38F);
    assert(p32_y_GET(pack) == (float)2.8301415E38F);
    assert(p32_x_GET(pack) == (float)6.562412E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lon_GET(pack) == (int32_t)1751368219);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)24003);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)26421);
    assert(p33_alt_GET(pack) == (int32_t)200097094);
    assert(p33_lat_GET(pack) == (int32_t)1024000856);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)46195);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)6149);
    assert(p33_relative_alt_GET(pack) == (int32_t) -1103683449);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)3064660787L);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)19413);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -12663);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -118);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -23621);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)19532);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1892351263L);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -29657);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -20225);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)12310);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)14506);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)31148);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)2529802364L);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)31007);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)19562);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)55273);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)12399);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)53200);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)32452);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)51078);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)13672);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)2150);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)5029);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)37096);
    assert(p36_time_usec_GET(pack) == (uint32_t)1657687733L);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)52381);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)39252);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)33257);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)65317);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)3475);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)10299);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)59410);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)45049);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)27635);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)12444);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)18972);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -26756);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -15390);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)32);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -30083);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)31489);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_param4_GET(pack) == (float) -3.3149053E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p39_param2_GET(pack) == (float)1.1960267E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)46798);
    assert(p39_param3_GET(pack) == (float) -1.8722364E37F);
    assert(p39_y_GET(pack) == (float)1.7853997E38F);
    assert(p39_param1_GET(pack) == (float)3.1294554E37F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE);
    assert(p39_x_GET(pack) == (float) -3.259754E38F);
    assert(p39_z_GET(pack) == (float) -8.411506E37F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)49233);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)15);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)59259);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)207);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)19146);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)27);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)32058);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)24715);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ERROR);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p48_altitude_GET(pack) == (int32_t)666249786);
    assert(p48_latitude_GET(pack) == (int32_t)873751517);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1923219053396279582L);
    assert(p48_longitude_GET(pack) == (int32_t)2059712);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_altitude_GET(pack) == (int32_t) -1237381391);
    assert(p49_time_usec_TRY(ph) == (uint64_t)5142321724795212631L);
    assert(p49_longitude_GET(pack) == (int32_t)1654399152);
    assert(p49_latitude_GET(pack) == (int32_t)1567629641);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p50_param_value_max_GET(pack) == (float) -2.7567283E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -15012);
    assert(p50_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"whtJpcyvdayr";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p50_param_value_min_GET(pack) == (float) -1.1958294E38F);
    assert(p50_scale_GET(pack) == (float)9.039135E37F);
    assert(p50_param_value0_GET(pack) == (float)1.3165329E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)29370);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p54_p1z_GET(pack) == (float)3.3498724E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p54_p1x_GET(pack) == (float) -2.0251313E38F);
    assert(p54_p2z_GET(pack) == (float) -6.105977E37F);
    assert(p54_p2y_GET(pack) == (float)2.8604491E38F);
    assert(p54_p2x_GET(pack) == (float)1.5915134E38F);
    assert(p54_p1y_GET(pack) == (float)1.6024705E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2x_GET(pack) == (float)1.4166262E38F);
    assert(p55_p1x_GET(pack) == (float)2.088433E38F);
    assert(p55_p1z_GET(pack) == (float)1.6748643E38F);
    assert(p55_p2z_GET(pack) == (float) -1.0958896E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p55_p2y_GET(pack) == (float) -2.8086065E38F);
    assert(p55_p1y_GET(pack) == (float)1.8708155E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_time_usec_GET(pack) == (uint64_t)7299278394145823672L);
    {
        float exemplary[] =  {-1.0997585E37F, 1.6336377E38F, -1.0179133E38F, 2.2624322E38F, 2.19328E38F, -1.8191796E38F, 2.6074496E38F, -2.5314877E38F, -1.315054E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.985172E38F, -1.2907591E38F, 1.2891452E38F, -8.851783E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float) -2.0942626E38F);
    assert(p61_rollspeed_GET(pack) == (float)3.0408636E38F);
    assert(p61_yawspeed_GET(pack) == (float)1.779212E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_pitch_GET(pack) == (float)1.9230278E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)63575);
    assert(p62_aspd_error_GET(pack) == (float)1.572086E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -3.0081886E38F);
    assert(p62_alt_error_GET(pack) == (float)1.1575848E37F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -15946);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)3163);
    assert(p62_nav_roll_GET(pack) == (float) -2.5619959E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t)197955571);
    assert(p63_lat_GET(pack) == (int32_t)2005155596);
    assert(p63_alt_GET(pack) == (int32_t) -1343901399);
    assert(p63_vz_GET(pack) == (float)1.874497E38F);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_time_usec_GET(pack) == (uint64_t)134086187806832567L);
    assert(p63_relative_alt_GET(pack) == (int32_t)1649695389);
    {
        float exemplary[] =  {8.992675E37F, -1.6390304E38F, 3.153418E38F, 2.1353313E38F, 3.3615877E38F, -1.8219922E38F, 1.4986667E38F, 3.3786647E38F, -1.942408E38F, -9.066253E37F, 4.392285E37F, -9.609272E37F, 2.4104802E38F, -2.4555911E38F, 2.3230322E38F, 1.9258785E38F, -3.1872565E38F, -1.6188197E38F, -1.836981E38F, 3.1984192E38F, -3.0499329E38F, 2.7278794E38F, 2.0470752E38F, 1.8526503E38F, 1.3472153E38F, 9.692387E37F, 1.3123091E38F, -1.5454715E38F, -2.3559465E38F, 1.1614236E38F, 2.2435225E38F, -2.2071724E38F, 2.3686717E38F, 1.2336006E38F, 1.4333819E38F, 1.4278382E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vy_GET(pack) == (float)2.7001655E37F);
    assert(p63_vx_GET(pack) == (float)2.5787534E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_z_GET(pack) == (float) -1.6698122E38F);
    {
        float exemplary[] =  {-2.4369427E38F, -3.3208042E38F, -1.3389458E38F, -9.430279E37F, -1.8597765E38F, 3.3384883E38F, -4.4910934E37F, 1.8883625E38F, 2.129482E38F, -2.0884361E38F, 3.0508573E38F, 1.93677E38F, -1.7797244E38F, 2.1108119E38F, 3.3125826E38F, 2.899527E38F, -3.787093E37F, -1.3114915E38F, 2.440276E38F, 1.9818362E38F, 1.113373E38F, -2.6944157E38F, -9.090668E37F, -2.9472838E38F, 2.612785E38F, 2.6178303E38F, 1.6223046E38F, 1.8090905E38F, -2.535822E38F, 2.0534129E38F, -3.4489964E37F, 4.8771307E36F, 1.2909491E37F, -1.5376475E38F, 1.4806603E38F, -9.528364E37F, 1.4519774E38F, 1.9520022E38F, 2.5179975E38F, -2.6344394E38F, -2.7592072E38F, -2.4105396E38F, 3.5974768E37F, 2.239268E38F, -1.2152455E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vy_GET(pack) == (float)3.2233795E38F);
    assert(p64_ax_GET(pack) == (float)2.673099E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_ay_GET(pack) == (float) -2.3661966E38F);
    assert(p64_vz_GET(pack) == (float)1.9142197E38F);
    assert(p64_az_GET(pack) == (float)1.3892256E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)7008051352561144337L);
    assert(p64_y_GET(pack) == (float)2.5544973E38F);
    assert(p64_vx_GET(pack) == (float)2.328527E38F);
    assert(p64_x_GET(pack) == (float)1.6033444E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)60204);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)21752);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)17483);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)20493);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3740241577L);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)43);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)8073);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)30313);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)10440);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)59555);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)61282);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)1363);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)38146);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)47613);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)37563);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11308);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)13460);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)64818);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)10739);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)30826);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)24066);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)191);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -4472);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)299);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -17020);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)18700);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)31066);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)259);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)10152);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)6402);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)15019);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)58918);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)34591);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)13956);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)35951);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p73_y_GET(pack) == (int32_t)825533370);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p73_x_GET(pack) == (int32_t) -367111639);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_param3_GET(pack) == (float)2.4616499E38F);
    assert(p73_param2_GET(pack) == (float)1.0252338E38F);
    assert(p73_param4_GET(pack) == (float) -1.0674067E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)30545);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p73_param1_GET(pack) == (float) -2.3333663E38F);
    assert(p73_z_GET(pack) == (float) -2.1168045E38F);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float) -2.9335299E38F);
    assert(p74_alt_GET(pack) == (float)2.8218693E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)47260);
    assert(p74_climb_GET(pack) == (float) -4.1114863E37F);
    assert(p74_airspeed_GET(pack) == (float)9.069406E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)4356);
};


void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p75_z_GET(pack) == (float) -1.301891E38F);
    assert(p75_param3_GET(pack) == (float) -1.5840722E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p75_param2_GET(pack) == (float)2.923574E38F);
    assert(p75_param1_GET(pack) == (float) -1.781822E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p75_y_GET(pack) == (int32_t) -1031454205);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL);
    assert(p75_param4_GET(pack) == (float)1.9604083E38F);
    assert(p75_x_GET(pack) == (int32_t) -1804366421);
};


void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS);
    assert(p76_param3_GET(pack) == (float)2.7163288E38F);
    assert(p76_param6_GET(pack) == (float)1.2219337E38F);
    assert(p76_param1_GET(pack) == (float)1.885954E38F);
    assert(p76_param5_GET(pack) == (float)1.6715292E38F);
    assert(p76_param7_GET(pack) == (float) -2.5346562E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p76_param4_GET(pack) == (float) -3.3606957E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p76_param2_GET(pack) == (float) -2.1814985E38F);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)186);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)158);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_UNSUPPORTED);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1641692374);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)53);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p81_pitch_GET(pack) == (float)6.9463795E37F);
    assert(p81_roll_GET(pack) == (float)1.3877806E38F);
    assert(p81_thrust_GET(pack) == (float) -5.0912803E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3801605090L);
    assert(p81_yaw_GET(pack) == (float) -4.784882E36F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.006726E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    {
        float exemplary[] =  {1.7283897E38F, -5.048295E37F, -9.709133E37F, 3.280587E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p82_body_roll_rate_GET(pack) == (float) -3.3070327E38F);
    assert(p82_thrust_GET(pack) == (float) -2.249642E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float) -2.9375065E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2651332951L);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float) -1.3850263E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)1973478928L);
    {
        float exemplary[] =  {-6.871629E36F, 2.6017701E38F, -8.345181E37F, -2.9318444E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_thrust_GET(pack) == (float)2.0417495E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p83_body_yaw_rate_GET(pack) == (float) -8.44872E37F);
    assert(p83_body_roll_rate_GET(pack) == (float)2.8643188E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float) -3.0810463E38F);
    assert(p84_z_GET(pack) == (float) -3.8447377E36F);
    assert(p84_vz_GET(pack) == (float)7.9023843E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p84_vy_GET(pack) == (float)3.2036352E38F);
    assert(p84_afz_GET(pack) == (float)1.7047067E38F);
    assert(p84_y_GET(pack) == (float)6.43701E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)685642074L);
    assert(p84_yaw_GET(pack) == (float) -1.1869339E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)6833);
    assert(p84_x_GET(pack) == (float) -7.861003E36F);
    assert(p84_afx_GET(pack) == (float)3.0762434E38F);
    assert(p84_yaw_rate_GET(pack) == (float)1.0530615E38F);
    assert(p84_vx_GET(pack) == (float)1.9501916E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_alt_GET(pack) == (float)1.0759623E38F);
    assert(p86_yaw_rate_GET(pack) == (float)3.3033407E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p86_afy_GET(pack) == (float)1.189635E38F);
    assert(p86_vy_GET(pack) == (float)1.4527239E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3139229058L);
    assert(p86_vz_GET(pack) == (float) -1.8338479E37F);
    assert(p86_afz_GET(pack) == (float)2.0050444E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)269272145);
    assert(p86_afx_GET(pack) == (float)1.1961693E38F);
    assert(p86_yaw_GET(pack) == (float)2.8374817E38F);
    assert(p86_vx_GET(pack) == (float)3.3075128E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p86_lon_int_GET(pack) == (int32_t)1233219105);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)30320);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_GET(pack) == (float)1.6378949E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -1344863223);
    assert(p87_vz_GET(pack) == (float) -1.5103252E38F);
    assert(p87_vx_GET(pack) == (float)5.409404E37F);
    assert(p87_lat_int_GET(pack) == (int32_t) -1237434558);
    assert(p87_afz_GET(pack) == (float) -6.318017E37F);
    assert(p87_afy_GET(pack) == (float)2.6227259E38F);
    assert(p87_alt_GET(pack) == (float) -3.3465511E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2749554933L);
    assert(p87_afx_GET(pack) == (float) -2.7659584E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -1.1799289E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)58709);
    assert(p87_vy_GET(pack) == (float)1.6826324E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float)2.1871656E37F);
    assert(p89_roll_GET(pack) == (float)1.2011131E38F);
    assert(p89_pitch_GET(pack) == (float) -3.0061614E37F);
    assert(p89_yaw_GET(pack) == (float)2.1783194E38F);
    assert(p89_x_GET(pack) == (float)3.025476E38F);
    assert(p89_z_GET(pack) == (float) -6.225537E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2581991961L);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_rollspeed_GET(pack) == (float)2.8651346E38F);
    assert(p90_pitchspeed_GET(pack) == (float)2.4272454E38F);
    assert(p90_yaw_GET(pack) == (float)1.5356899E38F);
    assert(p90_roll_GET(pack) == (float)2.511215E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)26843);
    assert(p90_yawspeed_GET(pack) == (float) -2.0266133E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -30622);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)13665);
    assert(p90_time_usec_GET(pack) == (uint64_t)2781226436743815201L);
    assert(p90_alt_GET(pack) == (int32_t) -1201546970);
    assert(p90_pitch_GET(pack) == (float)2.1765747E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -30850);
    assert(p90_lon_GET(pack) == (int32_t) -951764448);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -25501);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)14331);
    assert(p90_lat_GET(pack) == (int32_t)989732767);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_roll_ailerons_GET(pack) == (float) -1.9758152E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -1.8279102E38F);
    assert(p91_aux4_GET(pack) == (float) -3.1848476E38F);
    assert(p91_aux2_GET(pack) == (float)3.0482997E38F);
    assert(p91_aux1_GET(pack) == (float)3.331718E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)3.3393044E38F);
    assert(p91_throttle_GET(pack) == (float)1.3298584E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p91_aux3_GET(pack) == (float) -5.6478734E36F);
    assert(p91_time_usec_GET(pack) == (uint64_t)8316745193885672204L);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)36011);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)20803);
    assert(p92_time_usec_GET(pack) == (uint64_t)3663797172986814734L);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)2079);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)54161);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)8973);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)1525);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)48405);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)30406);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)30286);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)25623);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)6333);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)59112);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)221181000520005436L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_DISARMED);
    {
        float exemplary[] =  {-8.0268924E37F, -1.2366151E38F, 1.7417728E38F, -2.3602995E38F, -1.6827568E38F, -2.9575138E38F, 3.0938272E38F, -1.1506472E38F, -2.0907603E38F, -1.6707755E38F, 1.0218559E38F, -2.7035932E37F, 1.2965905E38F, 2.1756868E38F, 1.9776948E38F, -3.354939E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)6526834267480741586L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_rate_x_TRY(ph) == (float)1.4115999E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p100_ground_distance_GET(pack) == (float)9.683519E36F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.8206489E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)17050);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)24702);
    assert(p100_flow_comp_m_y_GET(pack) == (float)1.1704209E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)7.1048115E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)412618383370504699L);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_pitch_GET(pack) == (float)1.3967989E38F);
    assert(p101_usec_GET(pack) == (uint64_t)8118770812204925674L);
    assert(p101_roll_GET(pack) == (float) -7.450939E37F);
    assert(p101_z_GET(pack) == (float) -2.3277199E38F);
    assert(p101_y_GET(pack) == (float) -2.6963281E38F);
    assert(p101_x_GET(pack) == (float)1.4915651E38F);
    assert(p101_yaw_GET(pack) == (float)2.7876129E37F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_x_GET(pack) == (float) -3.067035E38F);
    assert(p102_roll_GET(pack) == (float)1.0264344E37F);
    assert(p102_usec_GET(pack) == (uint64_t)6703553654120414539L);
    assert(p102_z_GET(pack) == (float)4.9711375E37F);
    assert(p102_pitch_GET(pack) == (float) -1.4892465E38F);
    assert(p102_y_GET(pack) == (float) -3.0788213E38F);
    assert(p102_yaw_GET(pack) == (float) -1.4256395E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float)6.5736E37F);
    assert(p103_usec_GET(pack) == (uint64_t)8065180117579933809L);
    assert(p103_x_GET(pack) == (float) -2.4832367E38F);
    assert(p103_y_GET(pack) == (float) -3.0625459E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float) -2.9551025E38F);
    assert(p104_y_GET(pack) == (float)2.8818974E38F);
    assert(p104_z_GET(pack) == (float) -1.1747345E38F);
    assert(p104_usec_GET(pack) == (uint64_t)1206017022636779189L);
    assert(p104_roll_GET(pack) == (float)6.020918E36F);
    assert(p104_pitch_GET(pack) == (float) -8.917477E36F);
    assert(p104_x_GET(pack) == (float)9.377127E37F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_ymag_GET(pack) == (float)1.5565715E38F);
    assert(p105_ygyro_GET(pack) == (float) -6.1699587E37F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)36403);
    assert(p105_diff_pressure_GET(pack) == (float)1.7656933E38F);
    assert(p105_zgyro_GET(pack) == (float) -1.908982E38F);
    assert(p105_zacc_GET(pack) == (float)2.8598092E38F);
    assert(p105_yacc_GET(pack) == (float)1.9135076E37F);
    assert(p105_temperature_GET(pack) == (float)2.959566E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -1.0170709E38F);
    assert(p105_xgyro_GET(pack) == (float) -1.6830517E37F);
    assert(p105_xmag_GET(pack) == (float)2.2934097E38F);
    assert(p105_xacc_GET(pack) == (float)8.671136E37F);
    assert(p105_abs_pressure_GET(pack) == (float)1.2186993E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)2531961716453451517L);
    assert(p105_zmag_GET(pack) == (float)3.0253713E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_distance_GET(pack) == (float) -6.7511136E37F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)30848);
    assert(p106_integrated_xgyro_GET(pack) == (float)1.0314596E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)2862142505220433122L);
    assert(p106_integrated_x_GET(pack) == (float)1.3031659E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3015893999L);
    assert(p106_integrated_y_GET(pack) == (float) -1.1055013E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)748522867L);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p106_integrated_zgyro_GET(pack) == (float)3.6370214E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p106_integrated_ygyro_GET(pack) == (float)2.1265676E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float) -1.6180555E38F);
    assert(p107_temperature_GET(pack) == (float)2.7398995E37F);
    assert(p107_yacc_GET(pack) == (float)2.968856E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)3235133965418775373L);
    assert(p107_fields_updated_GET(pack) == (uint32_t)3478981554L);
    assert(p107_xacc_GET(pack) == (float)7.867336E37F);
    assert(p107_abs_pressure_GET(pack) == (float) -3.3884674E38F);
    assert(p107_ygyro_GET(pack) == (float)1.7916848E38F);
    assert(p107_ymag_GET(pack) == (float)7.734063E37F);
    assert(p107_zgyro_GET(pack) == (float)1.9269397E38F);
    assert(p107_xgyro_GET(pack) == (float) -1.8690873E38F);
    assert(p107_pressure_alt_GET(pack) == (float)3.290046E38F);
    assert(p107_diff_pressure_GET(pack) == (float)9.421345E37F);
    assert(p107_zacc_GET(pack) == (float) -3.2575795E38F);
    assert(p107_xmag_GET(pack) == (float)1.8548197E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_pitch_GET(pack) == (float)1.7841411E38F);
    assert(p108_ygyro_GET(pack) == (float)3.1993307E38F);
    assert(p108_alt_GET(pack) == (float) -1.7902543E37F);
    assert(p108_yaw_GET(pack) == (float)1.0419518E38F);
    assert(p108_lon_GET(pack) == (float)6.6733123E37F);
    assert(p108_xgyro_GET(pack) == (float)1.5496138E38F);
    assert(p108_zacc_GET(pack) == (float) -2.1743911E38F);
    assert(p108_zgyro_GET(pack) == (float)1.9061439E38F);
    assert(p108_q3_GET(pack) == (float) -2.8979748E38F);
    assert(p108_xacc_GET(pack) == (float) -1.9483755E38F);
    assert(p108_vd_GET(pack) == (float)1.1004289E38F);
    assert(p108_roll_GET(pack) == (float)1.4552023E38F);
    assert(p108_vn_GET(pack) == (float)2.0313687E38F);
    assert(p108_lat_GET(pack) == (float) -5.468513E37F);
    assert(p108_yacc_GET(pack) == (float)1.4546914E38F);
    assert(p108_q2_GET(pack) == (float) -2.053697E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)1.8342624E38F);
    assert(p108_ve_GET(pack) == (float)1.858377E38F);
    assert(p108_q1_GET(pack) == (float) -1.7923815E38F);
    assert(p108_q4_GET(pack) == (float) -1.5925506E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -9.790762E37F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)20199);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)59872);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)143, (uint8_t)168, (uint8_t)25, (uint8_t)55, (uint8_t)48, (uint8_t)78, (uint8_t)201, (uint8_t)125, (uint8_t)249, (uint8_t)206, (uint8_t)83, (uint8_t)94, (uint8_t)238, (uint8_t)122, (uint8_t)125, (uint8_t)57, (uint8_t)196, (uint8_t)204, (uint8_t)203, (uint8_t)19, (uint8_t)172, (uint8_t)199, (uint8_t)221, (uint8_t)67, (uint8_t)14, (uint8_t)93, (uint8_t)160, (uint8_t)4, (uint8_t)53, (uint8_t)188, (uint8_t)120, (uint8_t)46, (uint8_t)162, (uint8_t)118, (uint8_t)147, (uint8_t)143, (uint8_t)175, (uint8_t)137, (uint8_t)28, (uint8_t)140, (uint8_t)48, (uint8_t)182, (uint8_t)51, (uint8_t)231, (uint8_t)231, (uint8_t)188, (uint8_t)93, (uint8_t)63, (uint8_t)212, (uint8_t)61, (uint8_t)165, (uint8_t)224, (uint8_t)154, (uint8_t)166, (uint8_t)174, (uint8_t)9, (uint8_t)231, (uint8_t)186, (uint8_t)185, (uint8_t)227, (uint8_t)137, (uint8_t)149, (uint8_t)140, (uint8_t)109, (uint8_t)5, (uint8_t)235, (uint8_t)8, (uint8_t)224, (uint8_t)252, (uint8_t)195, (uint8_t)17, (uint8_t)109, (uint8_t)64, (uint8_t)171, (uint8_t)30, (uint8_t)181, (uint8_t)153, (uint8_t)95, (uint8_t)15, (uint8_t)8, (uint8_t)211, (uint8_t)2, (uint8_t)232, (uint8_t)220, (uint8_t)76, (uint8_t)82, (uint8_t)148, (uint8_t)37, (uint8_t)112, (uint8_t)217, (uint8_t)230, (uint8_t)201, (uint8_t)18, (uint8_t)23, (uint8_t)178, (uint8_t)215, (uint8_t)194, (uint8_t)205, (uint8_t)104, (uint8_t)77, (uint8_t)5, (uint8_t)207, (uint8_t)199, (uint8_t)220, (uint8_t)82, (uint8_t)230, (uint8_t)226, (uint8_t)147, (uint8_t)208, (uint8_t)164, (uint8_t)238, (uint8_t)219, (uint8_t)83, (uint8_t)54, (uint8_t)238, (uint8_t)214, (uint8_t)12, (uint8_t)78, (uint8_t)28, (uint8_t)52, (uint8_t)116, (uint8_t)231, (uint8_t)23, (uint8_t)246, (uint8_t)8, (uint8_t)86, (uint8_t)186, (uint8_t)99, (uint8_t)80, (uint8_t)163, (uint8_t)207, (uint8_t)72, (uint8_t)54, (uint8_t)63, (uint8_t)78, (uint8_t)17, (uint8_t)177, (uint8_t)126, (uint8_t)52, (uint8_t)13, (uint8_t)149, (uint8_t)79, (uint8_t)231, (uint8_t)176, (uint8_t)127, (uint8_t)134, (uint8_t)213, (uint8_t)20, (uint8_t)104, (uint8_t)199, (uint8_t)106, (uint8_t)25, (uint8_t)106, (uint8_t)210, (uint8_t)157, (uint8_t)111, (uint8_t)4, (uint8_t)66, (uint8_t)217, (uint8_t)188, (uint8_t)24, (uint8_t)47, (uint8_t)117, (uint8_t)30, (uint8_t)65, (uint8_t)191, (uint8_t)76, (uint8_t)64, (uint8_t)75, (uint8_t)227, (uint8_t)21, (uint8_t)161, (uint8_t)203, (uint8_t)27, (uint8_t)203, (uint8_t)104, (uint8_t)27, (uint8_t)149, (uint8_t)7, (uint8_t)241, (uint8_t)166, (uint8_t)139, (uint8_t)1, (uint8_t)89, (uint8_t)165, (uint8_t)26, (uint8_t)231, (uint8_t)116, (uint8_t)103, (uint8_t)159, (uint8_t)8, (uint8_t)27, (uint8_t)107, (uint8_t)163, (uint8_t)221, (uint8_t)204, (uint8_t)240, (uint8_t)15, (uint8_t)146, (uint8_t)58, (uint8_t)204, (uint8_t)12, (uint8_t)6, (uint8_t)228, (uint8_t)195, (uint8_t)131, (uint8_t)41, (uint8_t)249, (uint8_t)138, (uint8_t)226, (uint8_t)220, (uint8_t)248, (uint8_t)6, (uint8_t)167, (uint8_t)224, (uint8_t)133, (uint8_t)150, (uint8_t)113, (uint8_t)66, (uint8_t)190, (uint8_t)223, (uint8_t)198, (uint8_t)240, (uint8_t)121, (uint8_t)13, (uint8_t)111, (uint8_t)224, (uint8_t)97, (uint8_t)49, (uint8_t)78, (uint8_t)197, (uint8_t)152, (uint8_t)165, (uint8_t)201, (uint8_t)2, (uint8_t)107, (uint8_t)234, (uint8_t)183, (uint8_t)62, (uint8_t)174, (uint8_t)155, (uint8_t)224, (uint8_t)3, (uint8_t)52, (uint8_t)9, (uint8_t)59, (uint8_t)24, (uint8_t)175, (uint8_t)221, (uint8_t)28, (uint8_t)203} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)77);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -2689485646187033610L);
    assert(p111_ts1_GET(pack) == (int64_t)4349254374565009158L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)1835393123970923715L);
    assert(p112_seq_GET(pack) == (uint32_t)3765137406L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -24025);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -9661);
    assert(p113_lat_GET(pack) == (int32_t) -557786525);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)39670);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)49939);
    assert(p113_alt_GET(pack) == (int32_t) -267639183);
    assert(p113_lon_GET(pack) == (int32_t)989301673);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)45056);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -26274);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)41664);
    assert(p113_time_usec_GET(pack) == (uint64_t)9091569222351114604L);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)4461);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p114_integrated_y_GET(pack) == (float)4.0098007E37F);
    assert(p114_integrated_zgyro_GET(pack) == (float)1.7435268E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p114_integrated_xgyro_GET(pack) == (float) -3.272776E38F);
    assert(p114_integrated_x_GET(pack) == (float) -1.6651567E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)4278723362360207712L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)694269199L);
    assert(p114_integrated_ygyro_GET(pack) == (float) -2.8288003E37F);
    assert(p114_distance_GET(pack) == (float)3.1374845E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3370393311L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.49382E38F, -1.7509638E38F, 1.703202E37F, -1.5994473E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_time_usec_GET(pack) == (uint64_t)4260791009304458455L);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)40919);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)19583);
    assert(p115_lon_GET(pack) == (int32_t)1293429311);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -19730);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)22067);
    assert(p115_pitchspeed_GET(pack) == (float)2.230368E38F);
    assert(p115_yawspeed_GET(pack) == (float) -2.1178937E38F);
    assert(p115_rollspeed_GET(pack) == (float) -1.6793678E38F);
    assert(p115_lat_GET(pack) == (int32_t)1205960289);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)11849);
    assert(p115_alt_GET(pack) == (int32_t)1719213762);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)30767);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)28828);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -23728);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)588);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)27000);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -6612);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)16005);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)2621);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)1446029072L);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -15495);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)31482);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)6685);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -17197);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)17095);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)10959);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)16);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)64804);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)48833);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)16607);
    assert(p118_time_utc_GET(pack) == (uint32_t)1947518875L);
    assert(p118_size_GET(pack) == (uint32_t)378694524L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)42429);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p119_count_GET(pack) == (uint32_t)299600496L);
    assert(p119_ofs_GET(pack) == (uint32_t)134805848L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)11605);
    {
        uint8_t exemplary[] =  {(uint8_t)94, (uint8_t)53, (uint8_t)105, (uint8_t)101, (uint8_t)127, (uint8_t)66, (uint8_t)204, (uint8_t)15, (uint8_t)122, (uint8_t)190, (uint8_t)28, (uint8_t)250, (uint8_t)174, (uint8_t)211, (uint8_t)171, (uint8_t)35, (uint8_t)176, (uint8_t)7, (uint8_t)192, (uint8_t)251, (uint8_t)22, (uint8_t)42, (uint8_t)43, (uint8_t)243, (uint8_t)10, (uint8_t)205, (uint8_t)143, (uint8_t)172, (uint8_t)88, (uint8_t)231, (uint8_t)75, (uint8_t)112, (uint8_t)168, (uint8_t)74, (uint8_t)153, (uint8_t)189, (uint8_t)155, (uint8_t)131, (uint8_t)197, (uint8_t)24, (uint8_t)15, (uint8_t)41, (uint8_t)50, (uint8_t)206, (uint8_t)117, (uint8_t)161, (uint8_t)107, (uint8_t)190, (uint8_t)49, (uint8_t)55, (uint8_t)124, (uint8_t)145, (uint8_t)18, (uint8_t)107, (uint8_t)214, (uint8_t)117, (uint8_t)190, (uint8_t)36, (uint8_t)74, (uint8_t)220, (uint8_t)52, (uint8_t)204, (uint8_t)92, (uint8_t)203, (uint8_t)230, (uint8_t)49, (uint8_t)82, (uint8_t)95, (uint8_t)246, (uint8_t)15, (uint8_t)162, (uint8_t)198, (uint8_t)141, (uint8_t)85, (uint8_t)4, (uint8_t)167, (uint8_t)195, (uint8_t)184, (uint8_t)203, (uint8_t)232, (uint8_t)242, (uint8_t)188, (uint8_t)213, (uint8_t)52, (uint8_t)142, (uint8_t)240, (uint8_t)75, (uint8_t)18, (uint8_t)202, (uint8_t)19} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)701919745L);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)107);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)255);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)133);
    {
        uint8_t exemplary[] =  {(uint8_t)205, (uint8_t)70, (uint8_t)162, (uint8_t)154, (uint8_t)251, (uint8_t)60, (uint8_t)171, (uint8_t)243, (uint8_t)189, (uint8_t)232, (uint8_t)20, (uint8_t)45, (uint8_t)191, (uint8_t)146, (uint8_t)55, (uint8_t)121, (uint8_t)124, (uint8_t)85, (uint8_t)164, (uint8_t)86, (uint8_t)140, (uint8_t)166, (uint8_t)87, (uint8_t)209, (uint8_t)212, (uint8_t)249, (uint8_t)92, (uint8_t)90, (uint8_t)172, (uint8_t)39, (uint8_t)193, (uint8_t)98, (uint8_t)192, (uint8_t)45, (uint8_t)113, (uint8_t)88, (uint8_t)164, (uint8_t)119, (uint8_t)232, (uint8_t)250, (uint8_t)184, (uint8_t)85, (uint8_t)20, (uint8_t)156, (uint8_t)123, (uint8_t)94, (uint8_t)244, (uint8_t)47, (uint8_t)60, (uint8_t)41, (uint8_t)197, (uint8_t)237, (uint8_t)209, (uint8_t)152, (uint8_t)169, (uint8_t)27, (uint8_t)122, (uint8_t)125, (uint8_t)161, (uint8_t)233, (uint8_t)122, (uint8_t)225, (uint8_t)173, (uint8_t)57, (uint8_t)1, (uint8_t)202, (uint8_t)46, (uint8_t)139, (uint8_t)246, (uint8_t)30, (uint8_t)91, (uint8_t)37, (uint8_t)138, (uint8_t)71, (uint8_t)72, (uint8_t)192, (uint8_t)44, (uint8_t)251, (uint8_t)111, (uint8_t)105, (uint8_t)229, (uint8_t)138, (uint8_t)231, (uint8_t)46, (uint8_t)36, (uint8_t)207, (uint8_t)104, (uint8_t)215, (uint8_t)6, (uint8_t)44, (uint8_t)147, (uint8_t)6, (uint8_t)175, (uint8_t)224, (uint8_t)15, (uint8_t)204, (uint8_t)33, (uint8_t)128, (uint8_t)46, (uint8_t)81, (uint8_t)160, (uint8_t)42, (uint8_t)1, (uint8_t)65, (uint8_t)118, (uint8_t)176, (uint8_t)70, (uint8_t)132, (uint8_t)55, (uint8_t)172} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lat_GET(pack) == (int32_t)359316989);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p124_lon_GET(pack) == (int32_t)881997607);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)57955);
    assert(p124_time_usec_GET(pack) == (uint64_t)6791614626726841459L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)8247);
    assert(p124_dgps_age_GET(pack) == (uint32_t)201436132L);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)30833);
    assert(p124_alt_GET(pack) == (int32_t) -1391247789);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)11607);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)107);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)12567);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)83, (uint8_t)56, (uint8_t)145, (uint8_t)160, (uint8_t)78, (uint8_t)224, (uint8_t)239, (uint8_t)140, (uint8_t)43, (uint8_t)137, (uint8_t)184, (uint8_t)224, (uint8_t)42, (uint8_t)196, (uint8_t)228, (uint8_t)206, (uint8_t)48, (uint8_t)166, (uint8_t)161, (uint8_t)208, (uint8_t)26, (uint8_t)107, (uint8_t)155, (uint8_t)55, (uint8_t)50, (uint8_t)238, (uint8_t)242, (uint8_t)18, (uint8_t)60, (uint8_t)244, (uint8_t)224, (uint8_t)30, (uint8_t)177, (uint8_t)239, (uint8_t)110, (uint8_t)105, (uint8_t)211, (uint8_t)193, (uint8_t)112, (uint8_t)241, (uint8_t)46, (uint8_t)23, (uint8_t)88, (uint8_t)245, (uint8_t)251, (uint8_t)34, (uint8_t)239, (uint8_t)130, (uint8_t)150, (uint8_t)200, (uint8_t)189, (uint8_t)180, (uint8_t)170, (uint8_t)2, (uint8_t)241, (uint8_t)10, (uint8_t)166, (uint8_t)27, (uint8_t)229, (uint8_t)154, (uint8_t)202, (uint8_t)199, (uint8_t)24, (uint8_t)50, (uint8_t)33, (uint8_t)33, (uint8_t)193, (uint8_t)128, (uint8_t)233, (uint8_t)48} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_baudrate_GET(pack) == (uint32_t)977200459L);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI));
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)41955);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -853782139);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)532127456L);
    assert(p127_accuracy_GET(pack) == (uint32_t)1356473552L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -457147882);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)14848);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)967647187);
    assert(p127_tow_GET(pack) == (uint32_t)642253106L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)1753853744);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)1499956976);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)324646647);
    assert(p128_accuracy_GET(pack) == (uint32_t)694948962L);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1379115072);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)506120324);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)56818);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p128_tow_GET(pack) == (uint32_t)3562588082L);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)3042815528L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -21749);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)405);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)3035);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -14576);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -18511);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)383);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -25543);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)20816);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)352802680L);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -11094);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)24552);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)53907);
    assert(p130_size_GET(pack) == (uint32_t)1617041137L);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)18680);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)35913);
    {
        uint8_t exemplary[] =  {(uint8_t)187, (uint8_t)82, (uint8_t)168, (uint8_t)199, (uint8_t)61, (uint8_t)5, (uint8_t)24, (uint8_t)164, (uint8_t)208, (uint8_t)167, (uint8_t)124, (uint8_t)130, (uint8_t)1, (uint8_t)181, (uint8_t)4, (uint8_t)197, (uint8_t)100, (uint8_t)73, (uint8_t)4, (uint8_t)126, (uint8_t)218, (uint8_t)234, (uint8_t)166, (uint8_t)171, (uint8_t)57, (uint8_t)233, (uint8_t)80, (uint8_t)72, (uint8_t)129, (uint8_t)56, (uint8_t)148, (uint8_t)95, (uint8_t)96, (uint8_t)76, (uint8_t)70, (uint8_t)96, (uint8_t)224, (uint8_t)107, (uint8_t)23, (uint8_t)202, (uint8_t)142, (uint8_t)45, (uint8_t)241, (uint8_t)219, (uint8_t)78, (uint8_t)241, (uint8_t)97, (uint8_t)136, (uint8_t)101, (uint8_t)38, (uint8_t)178, (uint8_t)82, (uint8_t)182, (uint8_t)167, (uint8_t)112, (uint8_t)181, (uint8_t)129, (uint8_t)200, (uint8_t)241, (uint8_t)18, (uint8_t)247, (uint8_t)228, (uint8_t)189, (uint8_t)71, (uint8_t)176, (uint8_t)162, (uint8_t)97, (uint8_t)216, (uint8_t)29, (uint8_t)74, (uint8_t)108, (uint8_t)59, (uint8_t)234, (uint8_t)46, (uint8_t)168, (uint8_t)253, (uint8_t)171, (uint8_t)82, (uint8_t)144, (uint8_t)214, (uint8_t)123, (uint8_t)127, (uint8_t)127, (uint8_t)61, (uint8_t)253, (uint8_t)91, (uint8_t)119, (uint8_t)191, (uint8_t)174, (uint8_t)4, (uint8_t)79, (uint8_t)84, (uint8_t)231, (uint8_t)55, (uint8_t)136, (uint8_t)169, (uint8_t)42, (uint8_t)212, (uint8_t)199, (uint8_t)75, (uint8_t)57, (uint8_t)108, (uint8_t)116, (uint8_t)203, (uint8_t)164, (uint8_t)189, (uint8_t)124, (uint8_t)13, (uint8_t)101, (uint8_t)204, (uint8_t)194, (uint8_t)19, (uint8_t)82, (uint8_t)62, (uint8_t)9, (uint8_t)41, (uint8_t)201, (uint8_t)251, (uint8_t)169, (uint8_t)77, (uint8_t)255, (uint8_t)70, (uint8_t)69, (uint8_t)96, (uint8_t)242, (uint8_t)105, (uint8_t)106, (uint8_t)232, (uint8_t)212, (uint8_t)103, (uint8_t)111, (uint8_t)178, (uint8_t)148, (uint8_t)86, (uint8_t)76, (uint8_t)191, (uint8_t)109, (uint8_t)60, (uint8_t)20, (uint8_t)157, (uint8_t)235, (uint8_t)125, (uint8_t)8, (uint8_t)127, (uint8_t)161, (uint8_t)144, (uint8_t)99, (uint8_t)15, (uint8_t)255, (uint8_t)213, (uint8_t)179, (uint8_t)142, (uint8_t)91, (uint8_t)77, (uint8_t)186, (uint8_t)37, (uint8_t)228, (uint8_t)119, (uint8_t)195, (uint8_t)224, (uint8_t)15, (uint8_t)220, (uint8_t)225, (uint8_t)79, (uint8_t)85, (uint8_t)111, (uint8_t)66, (uint8_t)229, (uint8_t)152, (uint8_t)155, (uint8_t)71, (uint8_t)116, (uint8_t)67, (uint8_t)98, (uint8_t)11, (uint8_t)146, (uint8_t)180, (uint8_t)205, (uint8_t)73, (uint8_t)159, (uint8_t)127, (uint8_t)189, (uint8_t)166, (uint8_t)161, (uint8_t)3, (uint8_t)244, (uint8_t)126, (uint8_t)163, (uint8_t)57, (uint8_t)236, (uint8_t)2, (uint8_t)161, (uint8_t)253, (uint8_t)86, (uint8_t)163, (uint8_t)237, (uint8_t)4, (uint8_t)101, (uint8_t)219, (uint8_t)73, (uint8_t)53, (uint8_t)161, (uint8_t)233, (uint8_t)242, (uint8_t)120, (uint8_t)40, (uint8_t)241, (uint8_t)138, (uint8_t)101, (uint8_t)193, (uint8_t)224, (uint8_t)122, (uint8_t)20, (uint8_t)228, (uint8_t)79, (uint8_t)117, (uint8_t)128, (uint8_t)20, (uint8_t)17, (uint8_t)53, (uint8_t)192, (uint8_t)41, (uint8_t)107, (uint8_t)27, (uint8_t)151, (uint8_t)175, (uint8_t)61, (uint8_t)100, (uint8_t)18, (uint8_t)20, (uint8_t)79, (uint8_t)123, (uint8_t)55, (uint8_t)250, (uint8_t)89, (uint8_t)23, (uint8_t)110, (uint8_t)78, (uint8_t)221, (uint8_t)221, (uint8_t)61, (uint8_t)134, (uint8_t)103, (uint8_t)156, (uint8_t)17, (uint8_t)139, (uint8_t)22, (uint8_t)241, (uint8_t)183, (uint8_t)33, (uint8_t)232, (uint8_t)74, (uint8_t)138} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)20846);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)37953);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)7750);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2329771615L);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lon_GET(pack) == (int32_t) -1381466486);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)42856);
    assert(p133_mask_GET(pack) == (uint64_t)5226407373882906714L);
    assert(p133_lat_GET(pack) == (int32_t) -498464944);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)52221);
    assert(p134_lat_GET(pack) == (int32_t) -1695693672);
    assert(p134_lon_GET(pack) == (int32_t)77044834);
    {
        int16_t exemplary[] =  {(int16_t) -14931, (int16_t) -27367, (int16_t)8130, (int16_t) -9211, (int16_t)11696, (int16_t)18148, (int16_t)8081, (int16_t)30033, (int16_t) -31835, (int16_t)11275, (int16_t)12801, (int16_t)32025, (int16_t)13956, (int16_t)31123, (int16_t) -24388, (int16_t)20705} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)69);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)282975167);
    assert(p135_lon_GET(pack) == (int32_t) -1376015770);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lat_GET(pack) == (int32_t) -280934409);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)41801);
    assert(p136_lon_GET(pack) == (int32_t)1391397828);
    assert(p136_terrain_height_GET(pack) == (float)2.2932576E38F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)26507);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)35925);
    assert(p136_current_height_GET(pack) == (float)1.6126564E38F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3618817596L);
    assert(p137_press_abs_GET(pack) == (float) -7.300449E37F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -22334);
    assert(p137_press_diff_GET(pack) == (float)8.607557E37F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_z_GET(pack) == (float)1.8867152E38F);
    assert(p138_x_GET(pack) == (float) -3.2552685E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)8385775580460573572L);
    {
        float exemplary[] =  {-1.0086597E38F, 7.033651E36F, 1.6166746E38F, 7.4238766E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_y_GET(pack) == (float) -2.1917882E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)163);
    {
        float exemplary[] =  {-5.660196E37F, 1.763918E38F, 2.425814E38F, -2.7227023E37F, 2.499352E38F, 1.1184288E38F, 1.3564305E38F, -1.9668671E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p139_time_usec_GET(pack) == (uint64_t)1911272738448978091L);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)5634389353038876433L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)155);
    {
        float exemplary[] =  {-3.3307655E38F, 2.4665438E37F, -2.545521E38F, 1.5849667E38F, -2.5727949E38F, 1.375263E38F, -1.5249613E38F, -1.934323E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_terrain_GET(pack) == (float) -2.3590436E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)7329697944692647208L);
    assert(p141_altitude_relative_GET(pack) == (float)9.621488E36F);
    assert(p141_altitude_local_GET(pack) == (float)3.1059265E37F);
    assert(p141_altitude_amsl_GET(pack) == (float)3.1155523E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -2.1392024E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.499503E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)222);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)64, (uint8_t)107, (uint8_t)121, (uint8_t)15, (uint8_t)232, (uint8_t)129, (uint8_t)225, (uint8_t)252, (uint8_t)6, (uint8_t)135, (uint8_t)204, (uint8_t)39, (uint8_t)41, (uint8_t)141, (uint8_t)116, (uint8_t)185, (uint8_t)170, (uint8_t)221, (uint8_t)177, (uint8_t)238, (uint8_t)94, (uint8_t)233, (uint8_t)24, (uint8_t)2, (uint8_t)46, (uint8_t)64, (uint8_t)52, (uint8_t)218, (uint8_t)85, (uint8_t)54, (uint8_t)243, (uint8_t)71, (uint8_t)119, (uint8_t)247, (uint8_t)41, (uint8_t)195, (uint8_t)215, (uint8_t)113, (uint8_t)190, (uint8_t)12, (uint8_t)17, (uint8_t)161, (uint8_t)162, (uint8_t)145, (uint8_t)56, (uint8_t)50, (uint8_t)6, (uint8_t)95, (uint8_t)199, (uint8_t)66, (uint8_t)163, (uint8_t)116, (uint8_t)26, (uint8_t)38, (uint8_t)157, (uint8_t)63, (uint8_t)67, (uint8_t)142, (uint8_t)170, (uint8_t)245, (uint8_t)129, (uint8_t)2, (uint8_t)147, (uint8_t)82, (uint8_t)142, (uint8_t)202, (uint8_t)219, (uint8_t)129, (uint8_t)226, (uint8_t)90, (uint8_t)204, (uint8_t)121, (uint8_t)196, (uint8_t)23, (uint8_t)134, (uint8_t)59, (uint8_t)192, (uint8_t)152, (uint8_t)116, (uint8_t)205, (uint8_t)32, (uint8_t)154, (uint8_t)239, (uint8_t)204, (uint8_t)29, (uint8_t)247, (uint8_t)124, (uint8_t)111, (uint8_t)185, (uint8_t)143, (uint8_t)173, (uint8_t)146, (uint8_t)215, (uint8_t)7, (uint8_t)109, (uint8_t)173, (uint8_t)105, (uint8_t)150, (uint8_t)90, (uint8_t)1, (uint8_t)191, (uint8_t)19, (uint8_t)178, (uint8_t)34, (uint8_t)246, (uint8_t)190, (uint8_t)210, (uint8_t)230, (uint8_t)156, (uint8_t)94, (uint8_t)101, (uint8_t)97, (uint8_t)67, (uint8_t)130, (uint8_t)238, (uint8_t)85, (uint8_t)73, (uint8_t)176, (uint8_t)79} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)83);
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)108, (uint8_t)230, (uint8_t)66, (uint8_t)91, (uint8_t)90, (uint8_t)141, (uint8_t)195, (uint8_t)38, (uint8_t)241, (uint8_t)20, (uint8_t)86, (uint8_t)252, (uint8_t)233, (uint8_t)179, (uint8_t)76, (uint8_t)34, (uint8_t)97, (uint8_t)132, (uint8_t)117, (uint8_t)99, (uint8_t)96, (uint8_t)214, (uint8_t)208, (uint8_t)242, (uint8_t)240, (uint8_t)240, (uint8_t)89, (uint8_t)29, (uint8_t)209, (uint8_t)116, (uint8_t)30, (uint8_t)141, (uint8_t)207, (uint8_t)176, (uint8_t)76, (uint8_t)249, (uint8_t)115, (uint8_t)80, (uint8_t)234, (uint8_t)138, (uint8_t)98, (uint8_t)78, (uint8_t)133, (uint8_t)109, (uint8_t)165, (uint8_t)253, (uint8_t)246, (uint8_t)205, (uint8_t)126, (uint8_t)185, (uint8_t)156, (uint8_t)201, (uint8_t)137, (uint8_t)124, (uint8_t)224, (uint8_t)226, (uint8_t)252, (uint8_t)35, (uint8_t)232, (uint8_t)62, (uint8_t)73, (uint8_t)42, (uint8_t)244, (uint8_t)111, (uint8_t)80, (uint8_t)179, (uint8_t)200, (uint8_t)210, (uint8_t)222, (uint8_t)21, (uint8_t)160, (uint8_t)247, (uint8_t)176, (uint8_t)17, (uint8_t)24, (uint8_t)214, (uint8_t)111, (uint8_t)78, (uint8_t)34, (uint8_t)210, (uint8_t)148, (uint8_t)241, (uint8_t)10, (uint8_t)110, (uint8_t)47, (uint8_t)180, (uint8_t)254, (uint8_t)158, (uint8_t)36, (uint8_t)141, (uint8_t)3, (uint8_t)183, (uint8_t)165, (uint8_t)100, (uint8_t)64, (uint8_t)83, (uint8_t)239, (uint8_t)12, (uint8_t)203, (uint8_t)128, (uint8_t)1, (uint8_t)55, (uint8_t)41, (uint8_t)91, (uint8_t)104, (uint8_t)35, (uint8_t)239, (uint8_t)195, (uint8_t)235, (uint8_t)171, (uint8_t)60, (uint8_t)15, (uint8_t)133, (uint8_t)189, (uint8_t)145, (uint8_t)221, (uint8_t)26, (uint8_t)145, (uint8_t)82} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float) -8.958351E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -18627);
    assert(p143_press_diff_GET(pack) == (float) -3.1260177E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)431280461L);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_lon_GET(pack) == (int32_t) -139403957);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)213);
    {
        float exemplary[] =  {2.983449E38F, -2.983097E38F, 2.932498E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.3192758E37F, 5.5901617E37F, 1.0300591E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)5308689589440451670L);
    {
        float exemplary[] =  {9.587057E37F, -1.906359E38F, 1.8313742E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)1.3598978E38F);
    {
        float exemplary[] =  {1.0222001E36F, -1.5232758E38F, 1.4080457E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -73795928);
    {
        float exemplary[] =  {-1.4574601E38F, 1.6794376E38F, -1.4702749E38F, 4.9766583E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)1650628327616744111L);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_y_vel_GET(pack) == (float) -6.220076E37F);
    assert(p146_z_vel_GET(pack) == (float) -2.95608E38F);
    assert(p146_y_acc_GET(pack) == (float) -3.0109687E38F);
    {
        float exemplary[] =  {2.4377953E38F, -8.828426E37F, -1.3793106E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_pos_GET(pack) == (float) -2.2457412E38F);
    assert(p146_roll_rate_GET(pack) == (float) -2.561504E38F);
    assert(p146_y_pos_GET(pack) == (float)4.8843186E37F);
    assert(p146_x_vel_GET(pack) == (float)1.8530441E38F);
    {
        float exemplary[] =  {1.2292276E38F, -1.8710002E38F, 8.649498E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_acc_GET(pack) == (float)2.9966203E37F);
    {
        float exemplary[] =  {2.2482755E38F, -1.7719528E38F, 5.704137E37F, 2.8615768E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_pos_GET(pack) == (float)7.652396E37F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.1000786E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)1619038403660305502L);
    assert(p146_z_acc_GET(pack) == (float)1.0283785E38F);
    assert(p146_airspeed_GET(pack) == (float)5.119441E37F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.8314758E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p147_energy_consumed_GET(pack) == (int32_t)2085033280);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)32078);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)1);
    assert(p147_current_consumed_GET(pack) == (int32_t)691693029);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD);
    {
        uint16_t exemplary[] =  {(uint16_t)47917, (uint16_t)53509, (uint16_t)19400, (uint16_t)50274, (uint16_t)57778, (uint16_t)49310, (uint16_t)55932, (uint16_t)13781, (uint16_t)20578, (uint16_t)53645} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -11409);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)27350);
    {
        uint8_t exemplary[] =  {(uint8_t)176, (uint8_t)20, (uint8_t)14, (uint8_t)212, (uint8_t)21, (uint8_t)103, (uint8_t)190, (uint8_t)2} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)226, (uint8_t)130, (uint8_t)149, (uint8_t)155, (uint8_t)246, (uint8_t)137, (uint8_t)145, (uint8_t)255} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)241, (uint8_t)245, (uint8_t)30, (uint8_t)184, (uint8_t)236, (uint8_t)161, (uint8_t)108, (uint8_t)192, (uint8_t)165, (uint8_t)234, (uint8_t)214, (uint8_t)241, (uint8_t)79, (uint8_t)116, (uint8_t)18, (uint8_t)167, (uint8_t)71, (uint8_t)171} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)39, (uint8_t)219, (uint8_t)200, (uint8_t)137, (uint8_t)2, (uint8_t)62, (uint8_t)243} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)3844912591L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1883292021L);
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT));
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)778016811L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1543635465L);
    assert(p148_uid_GET(pack) == (uint64_t)6995396170291095570L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)11647);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)86);
    assert(p149_size_x_GET(pack) == (float) -2.1226349E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)4407418101831889848L);
    assert(p149_angle_x_GET(pack) == (float)3.1746248E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    assert(p149_x_TRY(ph) == (float)1.0808698E37F);
    {
        float exemplary[] =  {-2.684405E38F, 1.8285028E38F, -1.3970531E38F, -2.0990876E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p149_angle_y_GET(pack) == (float)2.370314E38F);
    assert(p149_distance_GET(pack) == (float)6.4034995E37F);
    assert(p149_z_TRY(ph) == (float)5.327722E37F);
    assert(p149_size_y_GET(pack) == (float)2.9565153E38F);
    assert(p149_y_TRY(ph) == (float)1.4066739E38F);
};


void c_CommunicationChannel_on_AQ_TELEMETRY_F_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_value17_GET(pack) == (float)6.2901817E37F);
    assert(p150_value19_GET(pack) == (float) -2.7692994E38F);
    assert(p150_Index_GET(pack) == (uint16_t)(uint16_t)17866);
    assert(p150_value2_GET(pack) == (float)1.3191691E38F);
    assert(p150_value16_GET(pack) == (float)2.8570613E38F);
    assert(p150_value4_GET(pack) == (float)1.0884099E38F);
    assert(p150_value8_GET(pack) == (float)7.606547E37F);
    assert(p150_value7_GET(pack) == (float) -3.2967233E38F);
    assert(p150_value9_GET(pack) == (float)5.4435244E37F);
    assert(p150_value12_GET(pack) == (float)1.2104812E38F);
    assert(p150_value3_GET(pack) == (float) -3.3682697E38F);
    assert(p150_value15_GET(pack) == (float) -3.0737434E38F);
    assert(p150_value10_GET(pack) == (float)3.919579E37F);
    assert(p150_value11_GET(pack) == (float)1.2369319E38F);
    assert(p150_value14_GET(pack) == (float)2.4136445E38F);
    assert(p150_value13_GET(pack) == (float) -5.1004263E35F);
    assert(p150_value5_GET(pack) == (float)2.6295747E38F);
    assert(p150_value18_GET(pack) == (float)1.4266858E38F);
    assert(p150_value20_GET(pack) == (float)2.0093114E38F);
    assert(p150_value1_GET(pack) == (float) -3.2472858E37F);
    assert(p150_value6_GET(pack) == (float) -1.3713405E37F);
};


void c_CommunicationChannel_on_AQ_ESC_TELEMETRY_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_seq_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p152_num_in_seq_GET(pack) == (uint8_t)(uint8_t)227);
    {
        uint8_t exemplary[] =  {(uint8_t)198, (uint8_t)250, (uint8_t)169, (uint8_t)37} ;
        uint8_t*  sample = p152_data_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {3767355010L, 451454289L, 2184363697L, 2946792672L} ;
        uint32_t*  sample = p152_data1_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)201, (uint8_t)81, (uint8_t)229, (uint8_t)56} ;
        uint8_t*  sample = p152_escid_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {1157178701L, 3132872100L, 412329233L, 1176023367L} ;
        uint32_t*  sample = p152_data0_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_num_motors_GET(pack) == (uint8_t)(uint8_t)213);
    {
        uint16_t exemplary[] =  {(uint16_t)47998, (uint16_t)19648, (uint16_t)64121, (uint16_t)2713} ;
        uint16_t*  sample = p152_status_age_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_time_boot_ms_GET(pack) == (uint32_t)3454567170L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE));
    assert(p230_hagl_ratio_GET(pack) == (float)1.5459157E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)7550178073896723329L);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.955588E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.6114018E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.0657504E38F);
    assert(p230_tas_ratio_GET(pack) == (float)5.174929E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.0069607E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.0243843E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.3820996E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_horiz_accuracy_GET(pack) == (float)3.2007431E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)3206402163156425535L);
    assert(p231_var_vert_GET(pack) == (float) -5.448522E37F);
    assert(p231_wind_z_GET(pack) == (float)1.1534208E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)1.6657603E38F);
    assert(p231_wind_x_GET(pack) == (float) -1.0704133E38F);
    assert(p231_var_horiz_GET(pack) == (float) -1.3587766E38F);
    assert(p231_wind_alt_GET(pack) == (float) -2.7393917E38F);
    assert(p231_wind_y_GET(pack) == (float) -6.880156E36F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)55606);
    assert(p232_ve_GET(pack) == (float)2.4692582E38F);
    assert(p232_vd_GET(pack) == (float)2.1040686E38F);
    assert(p232_vn_GET(pack) == (float)3.1321713E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p232_hdop_GET(pack) == (float) -8.515767E37F);
    assert(p232_speed_accuracy_GET(pack) == (float) -2.1962381E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)1.0971364E38F);
    assert(p232_lat_GET(pack) == (int32_t)1225144050);
    assert(p232_lon_GET(pack) == (int32_t)191165136);
    assert(p232_time_usec_GET(pack) == (uint64_t)1729014026324559933L);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p232_alt_GET(pack) == (float) -2.988146E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2489352422L);
    assert(p232_vdop_GET(pack) == (float) -3.127378E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -3.0493994E37F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)69, (uint8_t)26, (uint8_t)139, (uint8_t)217, (uint8_t)60, (uint8_t)146, (uint8_t)196, (uint8_t)55, (uint8_t)186, (uint8_t)209, (uint8_t)250, (uint8_t)70, (uint8_t)22, (uint8_t)203, (uint8_t)86, (uint8_t)52, (uint8_t)42, (uint8_t)45, (uint8_t)65, (uint8_t)46, (uint8_t)125, (uint8_t)86, (uint8_t)118, (uint8_t)23, (uint8_t)114, (uint8_t)12, (uint8_t)41, (uint8_t)160, (uint8_t)118, (uint8_t)173, (uint8_t)160, (uint8_t)35, (uint8_t)239, (uint8_t)250, (uint8_t)6, (uint8_t)27, (uint8_t)184, (uint8_t)37, (uint8_t)154, (uint8_t)104, (uint8_t)228, (uint8_t)115, (uint8_t)101, (uint8_t)58, (uint8_t)155, (uint8_t)235, (uint8_t)11, (uint8_t)170, (uint8_t)185, (uint8_t)60, (uint8_t)86, (uint8_t)97, (uint8_t)89, (uint8_t)118, (uint8_t)83, (uint8_t)86, (uint8_t)27, (uint8_t)12, (uint8_t)76, (uint8_t)190, (uint8_t)254, (uint8_t)209, (uint8_t)62, (uint8_t)115, (uint8_t)185, (uint8_t)68, (uint8_t)245, (uint8_t)36, (uint8_t)34, (uint8_t)44, (uint8_t)210, (uint8_t)168, (uint8_t)67, (uint8_t)248, (uint8_t)134, (uint8_t)239, (uint8_t)188, (uint8_t)35, (uint8_t)88, (uint8_t)216, (uint8_t)184, (uint8_t)131, (uint8_t)210, (uint8_t)249, (uint8_t)21, (uint8_t)165, (uint8_t)4, (uint8_t)246, (uint8_t)45, (uint8_t)98, (uint8_t)91, (uint8_t)122, (uint8_t)49, (uint8_t)27, (uint8_t)75, (uint8_t)51, (uint8_t)200, (uint8_t)120, (uint8_t)58, (uint8_t)160, (uint8_t)169, (uint8_t)209, (uint8_t)67, (uint8_t)94, (uint8_t)131, (uint8_t)166, (uint8_t)86, (uint8_t)52, (uint8_t)125, (uint8_t)79, (uint8_t)223, (uint8_t)175, (uint8_t)156, (uint8_t)116, (uint8_t)66, (uint8_t)82, (uint8_t)87, (uint8_t)243, (uint8_t)120, (uint8_t)12, (uint8_t)107, (uint8_t)175, (uint8_t)224, (uint8_t)190, (uint8_t)8, (uint8_t)187, (uint8_t)122, (uint8_t)158, (uint8_t)155, (uint8_t)123, (uint8_t)19, (uint8_t)22, (uint8_t)45, (uint8_t)149, (uint8_t)10, (uint8_t)126, (uint8_t)72, (uint8_t)210, (uint8_t)195, (uint8_t)106, (uint8_t)194, (uint8_t)197, (uint8_t)95, (uint8_t)238, (uint8_t)8, (uint8_t)139, (uint8_t)232, (uint8_t)201, (uint8_t)198, (uint8_t)25, (uint8_t)37, (uint8_t)204, (uint8_t)207, (uint8_t)76, (uint8_t)214, (uint8_t)244, (uint8_t)254, (uint8_t)177, (uint8_t)79, (uint8_t)82, (uint8_t)147, (uint8_t)0, (uint8_t)238, (uint8_t)150, (uint8_t)14, (uint8_t)159, (uint8_t)13, (uint8_t)21, (uint8_t)147, (uint8_t)155, (uint8_t)65, (uint8_t)115, (uint8_t)103, (uint8_t)245, (uint8_t)251, (uint8_t)189, (uint8_t)26, (uint8_t)225, (uint8_t)157, (uint8_t)115} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)0);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -113);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)26800);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -928);
    assert(p234_custom_mode_GET(pack) == (uint32_t)2473184599L);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -31653);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED));
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_longitude_GET(pack) == (int32_t) -451132756);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)9458);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)91);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)62031);
    assert(p234_latitude_GET(pack) == (int32_t) -983246324);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)97);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -40);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -3627);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -26729);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_z_GET(pack) == (float) -1.2845564E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)7351982156701305515L);
    assert(p241_vibration_x_GET(pack) == (float) -1.9914823E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)254089256L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)153961006L);
    assert(p241_vibration_y_GET(pack) == (float)7.6471864E37F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)385573319L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_longitude_GET(pack) == (int32_t)1157017793);
    assert(p242_approach_z_GET(pack) == (float) -1.8481646E38F);
    assert(p242_z_GET(pack) == (float)3.3999993E38F);
    assert(p242_approach_y_GET(pack) == (float) -6.116728E37F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)4757372918215412978L);
    assert(p242_y_GET(pack) == (float) -2.868408E38F);
    {
        float exemplary[] =  {-1.7888846E38F, 1.3765528E37F, -3.0786883E38F, -1.7288158E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_x_GET(pack) == (float)2.6417262E38F);
    assert(p242_latitude_GET(pack) == (int32_t)214673482);
    assert(p242_altitude_GET(pack) == (int32_t)2029599948);
    assert(p242_x_GET(pack) == (float) -1.4312479E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_y_GET(pack) == (float) -6.7840457E37F);
    assert(p243_latitude_GET(pack) == (int32_t)2114027292);
    assert(p243_longitude_GET(pack) == (int32_t) -1096200481);
    assert(p243_approach_x_GET(pack) == (float) -2.84697E38F);
    {
        float exemplary[] =  {-2.416283E38F, 2.214733E38F, -6.241874E37F, -5.0301157E37F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_z_GET(pack) == (float)3.2539167E38F);
    assert(p243_approach_z_GET(pack) == (float) -2.7871371E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p243_x_GET(pack) == (float)2.5696517E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)1474354256405988074L);
    assert(p243_y_GET(pack) == (float) -2.5398546E38F);
    assert(p243_altitude_GET(pack) == (int32_t)1322495407);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)9934);
    assert(p244_interval_us_GET(pack) == (int32_t) -523513121);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)59246);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)64531);
    assert(p246_callsign_LEN(ph) == 9);
    {
        char16_t * exemplary = u"suedllnxb";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lat_GET(pack) == (int32_t)116460501);
    assert(p246_altitude_GET(pack) == (int32_t)1473517028);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)33697);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED));
    assert(p246_lon_GET(pack) == (int32_t) -1046003890);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -26402);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)440944949L);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -2.1499948E38F);
    assert(p247_threat_level_GET(pack) == (e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH |
                                           e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW));
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.3830025E38F);
    assert(p247_id_GET(pack) == (uint32_t)3243930578L);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)2.483384E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)34, (uint8_t)119, (uint8_t)232, (uint8_t)195, (uint8_t)117, (uint8_t)137, (uint8_t)237, (uint8_t)35, (uint8_t)184, (uint8_t)81, (uint8_t)202, (uint8_t)182, (uint8_t)175, (uint8_t)185, (uint8_t)196, (uint8_t)21, (uint8_t)241, (uint8_t)68, (uint8_t)29, (uint8_t)42, (uint8_t)89, (uint8_t)18, (uint8_t)59, (uint8_t)220, (uint8_t)113, (uint8_t)243, (uint8_t)254, (uint8_t)106, (uint8_t)49, (uint8_t)144, (uint8_t)119, (uint8_t)16, (uint8_t)209, (uint8_t)231, (uint8_t)145, (uint8_t)114, (uint8_t)157, (uint8_t)57, (uint8_t)248, (uint8_t)127, (uint8_t)2, (uint8_t)247, (uint8_t)81, (uint8_t)154, (uint8_t)65, (uint8_t)144, (uint8_t)138, (uint8_t)51, (uint8_t)34, (uint8_t)51, (uint8_t)171, (uint8_t)121, (uint8_t)159, (uint8_t)67, (uint8_t)104, (uint8_t)251, (uint8_t)1, (uint8_t)74, (uint8_t)161, (uint8_t)131, (uint8_t)145, (uint8_t)99, (uint8_t)10, (uint8_t)160, (uint8_t)101, (uint8_t)185, (uint8_t)59, (uint8_t)42, (uint8_t)107, (uint8_t)231, (uint8_t)125, (uint8_t)2, (uint8_t)208, (uint8_t)73, (uint8_t)35, (uint8_t)192, (uint8_t)151, (uint8_t)135, (uint8_t)157, (uint8_t)165, (uint8_t)46, (uint8_t)164, (uint8_t)150, (uint8_t)240, (uint8_t)0, (uint8_t)142, (uint8_t)5, (uint8_t)144, (uint8_t)200, (uint8_t)110, (uint8_t)2, (uint8_t)19, (uint8_t)99, (uint8_t)23, (uint8_t)240, (uint8_t)198, (uint8_t)236, (uint8_t)103, (uint8_t)204, (uint8_t)0, (uint8_t)219, (uint8_t)203, (uint8_t)143, (uint8_t)117, (uint8_t)208, (uint8_t)205, (uint8_t)29, (uint8_t)220, (uint8_t)14, (uint8_t)142, (uint8_t)74, (uint8_t)53, (uint8_t)206, (uint8_t)183, (uint8_t)197, (uint8_t)166, (uint8_t)109, (uint8_t)4, (uint8_t)138, (uint8_t)28, (uint8_t)213, (uint8_t)26, (uint8_t)107, (uint8_t)68, (uint8_t)196, (uint8_t)53, (uint8_t)151, (uint8_t)91, (uint8_t)152, (uint8_t)183, (uint8_t)160, (uint8_t)170, (uint8_t)7, (uint8_t)224, (uint8_t)27, (uint8_t)90, (uint8_t)36, (uint8_t)132, (uint8_t)246, (uint8_t)131, (uint8_t)129, (uint8_t)11, (uint8_t)85, (uint8_t)50, (uint8_t)234, (uint8_t)223, (uint8_t)199, (uint8_t)125, (uint8_t)246, (uint8_t)110, (uint8_t)26, (uint8_t)157, (uint8_t)28, (uint8_t)110, (uint8_t)180, (uint8_t)36, (uint8_t)249, (uint8_t)11, (uint8_t)75, (uint8_t)76, (uint8_t)127, (uint8_t)14, (uint8_t)59, (uint8_t)40, (uint8_t)182, (uint8_t)192, (uint8_t)134, (uint8_t)185, (uint8_t)198, (uint8_t)156, (uint8_t)199, (uint8_t)42, (uint8_t)123, (uint8_t)168, (uint8_t)117, (uint8_t)32, (uint8_t)131, (uint8_t)231, (uint8_t)103, (uint8_t)242, (uint8_t)223, (uint8_t)181, (uint8_t)152, (uint8_t)243, (uint8_t)166, (uint8_t)82, (uint8_t)238, (uint8_t)21, (uint8_t)246, (uint8_t)224, (uint8_t)177, (uint8_t)19, (uint8_t)242, (uint8_t)52, (uint8_t)40, (uint8_t)123, (uint8_t)99, (uint8_t)88, (uint8_t)75, (uint8_t)44, (uint8_t)63, (uint8_t)3, (uint8_t)0, (uint8_t)79, (uint8_t)77, (uint8_t)165, (uint8_t)131, (uint8_t)72, (uint8_t)110, (uint8_t)160, (uint8_t)67, (uint8_t)227, (uint8_t)141, (uint8_t)48, (uint8_t)66, (uint8_t)108, (uint8_t)50, (uint8_t)13, (uint8_t)243, (uint8_t)154, (uint8_t)236, (uint8_t)130, (uint8_t)88, (uint8_t)84, (uint8_t)61, (uint8_t)171, (uint8_t)23, (uint8_t)169, (uint8_t)181, (uint8_t)22, (uint8_t)218, (uint8_t)42, (uint8_t)139, (uint8_t)9, (uint8_t)64, (uint8_t)131, (uint8_t)24, (uint8_t)203, (uint8_t)191, (uint8_t)165, (uint8_t)130, (uint8_t)183, (uint8_t)169, (uint8_t)47, (uint8_t)83, (uint8_t)108, (uint8_t)228, (uint8_t)39} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)40838);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)46834);
    {
        int8_t exemplary[] =  {(int8_t) -7, (int8_t)10, (int8_t)51, (int8_t)125, (int8_t)2, (int8_t) -127, (int8_t) -69, (int8_t)120, (int8_t) -63, (int8_t)125, (int8_t)61, (int8_t)48, (int8_t)70, (int8_t) -54, (int8_t) -9, (int8_t) -20, (int8_t) -122, (int8_t)125, (int8_t) -85, (int8_t)28, (int8_t) -113, (int8_t)114, (int8_t) -98, (int8_t) -66, (int8_t) -34, (int8_t) -103, (int8_t)9, (int8_t) -34, (int8_t)121, (int8_t) -6, (int8_t) -6, (int8_t)5} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float)2.0670163E38F);
    assert(p250_x_GET(pack) == (float) -2.477569E38F);
    assert(p250_z_GET(pack) == (float)6.922714E37F);
    assert(p250_time_usec_GET(pack) == (uint64_t)8124758947596593065L);
    assert(p250_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"osd";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -4.284225E37F);
    assert(p251_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"ncsrwmsvqc";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)949870768L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t)1540569665);
    assert(p252_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"Qgz";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1556522035L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 35);
    {
        char16_t * exemplary = u"hegUimxpyqbustvtHaxqraidyqtzqhfgVyr";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3935043085L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p254_value_GET(pack) == (float) -3.8866246E37F);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    {
        uint8_t exemplary[] =  {(uint8_t)147, (uint8_t)126, (uint8_t)229, (uint8_t)190, (uint8_t)73, (uint8_t)88, (uint8_t)85, (uint8_t)160, (uint8_t)10, (uint8_t)154, (uint8_t)11, (uint8_t)109, (uint8_t)34, (uint8_t)99, (uint8_t)228, (uint8_t)57, (uint8_t)90, (uint8_t)201, (uint8_t)136, (uint8_t)247, (uint8_t)231, (uint8_t)207, (uint8_t)234, (uint8_t)82, (uint8_t)45, (uint8_t)203, (uint8_t)209, (uint8_t)180, (uint8_t)142, (uint8_t)247, (uint8_t)34, (uint8_t)246} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)1082966711951180144L);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)407775262L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1172484084L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p258_tune_LEN(ph) == 23);
    {
        char16_t * exemplary = u"gqjhnotjjuwptaggokljczv";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 46);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_v_GET(pack) == (float)1.0827285E38F);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)168);
    {
        uint8_t exemplary[] =  {(uint8_t)114, (uint8_t)2, (uint8_t)117, (uint8_t)192, (uint8_t)134, (uint8_t)159, (uint8_t)18, (uint8_t)84, (uint8_t)66, (uint8_t)223, (uint8_t)36, (uint8_t)5, (uint8_t)236, (uint8_t)8, (uint8_t)247, (uint8_t)171, (uint8_t)191, (uint8_t)3, (uint8_t)53, (uint8_t)81, (uint8_t)99, (uint8_t)153, (uint8_t)66, (uint8_t)174, (uint8_t)108, (uint8_t)209, (uint8_t)43, (uint8_t)49, (uint8_t)197, (uint8_t)131, (uint8_t)170, (uint8_t)223} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)331889379L);
    {
        uint8_t exemplary[] =  {(uint8_t)106, (uint8_t)178, (uint8_t)81, (uint8_t)57, (uint8_t)31, (uint8_t)102, (uint8_t)157, (uint8_t)124, (uint8_t)57, (uint8_t)69, (uint8_t)102, (uint8_t)242, (uint8_t)38, (uint8_t)40, (uint8_t)17, (uint8_t)66, (uint8_t)22, (uint8_t)10, (uint8_t)173, (uint8_t)9, (uint8_t)94, (uint8_t)104, (uint8_t)6, (uint8_t)54, (uint8_t)228, (uint8_t)75, (uint8_t)224, (uint8_t)26, (uint8_t)119, (uint8_t)60, (uint8_t)246, (uint8_t)202} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float)4.610976E37F);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
    assert(p259_firmware_version_GET(pack) == (uint32_t)2171741535L);
    assert(p259_cam_definition_uri_LEN(ph) == 13);
    {
        char16_t * exemplary = u"ypDqqavolzvcL";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)18773);
    assert(p259_sensor_size_h_GET(pack) == (float) -2.3514785E38F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)13234);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)60732);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3505527786L);
    assert(p260_mode_id_GET(pack) == (e_CAMERA_MODE_CAMERA_MODE_IMAGE));
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p261_available_capacity_GET(pack) == (float)5.9327037E37F);
    assert(p261_write_speed_GET(pack) == (float)1.3225602E38F);
    assert(p261_total_capacity_GET(pack) == (float)2.3873246E38F);
    assert(p261_used_capacity_GET(pack) == (float) -1.3079482E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2631759373L);
    assert(p261_read_speed_GET(pack) == (float) -1.0862719E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_available_capacity_GET(pack) == (float)9.443459E37F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)226072064L);
    assert(p262_image_interval_GET(pack) == (float)1.1590006E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3519907365L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -26);
    assert(p263_time_utc_GET(pack) == (uint64_t)1953461888702783785L);
    assert(p263_relative_alt_GET(pack) == (int32_t) -1047988233);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)3049128121L);
    assert(p263_alt_GET(pack) == (int32_t) -151725461);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p263_lon_GET(pack) == (int32_t)1025196);
    assert(p263_file_url_LEN(ph) == 203);
    {
        char16_t * exemplary = u"fshkowultvKtnzjtconwTppgexrwkaNpdYpakqtssicvnsavUjwvddwbuzrigqimgfjnsipvgmujobVnxbaqryxvcwpunQrkGhsykrfoYvrNocvakRtIvsbzbraiuyiipzYbrdzdAolwhwzKlvqexnddjmvdhupjzhlvwgNzcvpvrcvibwjkgxwzyveUsemutiziddrupwy";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 406);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_lat_GET(pack) == (int32_t)239973729);
    assert(p263_image_index_GET(pack) == (int32_t)1104771512);
    {
        float exemplary[] =  {1.139643E38F, 1.5610668E38F, 1.2220242E38F, 7.7502114E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)8957923858965941624L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)5909017516484199504L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)3217478618740819508L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)336319954L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)2.56425E38F);
    assert(p265_pitch_GET(pack) == (float)4.2880675E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2088930417L);
    assert(p265_roll_GET(pack) == (float) -1.1833299E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)87, (uint8_t)20, (uint8_t)78, (uint8_t)177, (uint8_t)131, (uint8_t)220, (uint8_t)162, (uint8_t)29, (uint8_t)114, (uint8_t)124, (uint8_t)237, (uint8_t)3, (uint8_t)95, (uint8_t)109, (uint8_t)142, (uint8_t)140, (uint8_t)128, (uint8_t)176, (uint8_t)147, (uint8_t)30, (uint8_t)195, (uint8_t)202, (uint8_t)232, (uint8_t)153, (uint8_t)182, (uint8_t)81, (uint8_t)14, (uint8_t)174, (uint8_t)248, (uint8_t)175, (uint8_t)83, (uint8_t)197, (uint8_t)71, (uint8_t)20, (uint8_t)29, (uint8_t)187, (uint8_t)21, (uint8_t)133, (uint8_t)172, (uint8_t)219, (uint8_t)200, (uint8_t)225, (uint8_t)20, (uint8_t)39, (uint8_t)164, (uint8_t)86, (uint8_t)135, (uint8_t)25, (uint8_t)195, (uint8_t)4, (uint8_t)51, (uint8_t)220, (uint8_t)141, (uint8_t)214, (uint8_t)245, (uint8_t)128, (uint8_t)50, (uint8_t)57, (uint8_t)80, (uint8_t)138, (uint8_t)184, (uint8_t)144, (uint8_t)126, (uint8_t)117, (uint8_t)95, (uint8_t)228, (uint8_t)80, (uint8_t)136, (uint8_t)98, (uint8_t)134, (uint8_t)111, (uint8_t)227, (uint8_t)63, (uint8_t)184, (uint8_t)40, (uint8_t)29, (uint8_t)76, (uint8_t)17, (uint8_t)214, (uint8_t)200, (uint8_t)92, (uint8_t)142, (uint8_t)11, (uint8_t)27, (uint8_t)216, (uint8_t)88, (uint8_t)138, (uint8_t)8, (uint8_t)233, (uint8_t)54, (uint8_t)88, (uint8_t)239, (uint8_t)247, (uint8_t)138, (uint8_t)152, (uint8_t)226, (uint8_t)228, (uint8_t)11, (uint8_t)230, (uint8_t)151, (uint8_t)245, (uint8_t)89, (uint8_t)81, (uint8_t)188, (uint8_t)92, (uint8_t)232, (uint8_t)118, (uint8_t)11, (uint8_t)51, (uint8_t)209, (uint8_t)205, (uint8_t)89, (uint8_t)150, (uint8_t)253, (uint8_t)66, (uint8_t)238, (uint8_t)81, (uint8_t)5, (uint8_t)180, (uint8_t)229, (uint8_t)215, (uint8_t)66, (uint8_t)131, (uint8_t)1, (uint8_t)129, (uint8_t)204, (uint8_t)109, (uint8_t)17, (uint8_t)232, (uint8_t)181, (uint8_t)19, (uint8_t)65, (uint8_t)211, (uint8_t)67, (uint8_t)46, (uint8_t)248, (uint8_t)125, (uint8_t)154, (uint8_t)10, (uint8_t)230, (uint8_t)130, (uint8_t)168, (uint8_t)56, (uint8_t)189, (uint8_t)93, (uint8_t)42, (uint8_t)232, (uint8_t)234, (uint8_t)179, (uint8_t)58, (uint8_t)136, (uint8_t)29, (uint8_t)204, (uint8_t)57, (uint8_t)220, (uint8_t)95, (uint8_t)241, (uint8_t)118, (uint8_t)157, (uint8_t)156, (uint8_t)123, (uint8_t)40, (uint8_t)109, (uint8_t)179, (uint8_t)172, (uint8_t)38, (uint8_t)253, (uint8_t)35, (uint8_t)40, (uint8_t)65, (uint8_t)210, (uint8_t)200, (uint8_t)43, (uint8_t)176, (uint8_t)234, (uint8_t)7, (uint8_t)185, (uint8_t)0, (uint8_t)207, (uint8_t)206, (uint8_t)111, (uint8_t)103, (uint8_t)93, (uint8_t)243, (uint8_t)83, (uint8_t)115, (uint8_t)77, (uint8_t)10, (uint8_t)108, (uint8_t)228, (uint8_t)179, (uint8_t)242, (uint8_t)14, (uint8_t)254, (uint8_t)120, (uint8_t)243, (uint8_t)78, (uint8_t)1, (uint8_t)98, (uint8_t)69, (uint8_t)9, (uint8_t)147, (uint8_t)143, (uint8_t)115, (uint8_t)70, (uint8_t)43, (uint8_t)246, (uint8_t)41, (uint8_t)216, (uint8_t)68, (uint8_t)174, (uint8_t)133, (uint8_t)254, (uint8_t)135, (uint8_t)173, (uint8_t)100, (uint8_t)98, (uint8_t)239, (uint8_t)1, (uint8_t)23, (uint8_t)13, (uint8_t)72, (uint8_t)91, (uint8_t)53, (uint8_t)48, (uint8_t)188, (uint8_t)200, (uint8_t)76, (uint8_t)245, (uint8_t)234, (uint8_t)244, (uint8_t)253, (uint8_t)242, (uint8_t)252, (uint8_t)93, (uint8_t)86, (uint8_t)224, (uint8_t)211, (uint8_t)71, (uint8_t)33, (uint8_t)113, (uint8_t)92, (uint8_t)229, (uint8_t)43, (uint8_t)84, (uint8_t)127, (uint8_t)48, (uint8_t)105, (uint8_t)224} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)10595);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)27283);
    {
        uint8_t exemplary[] =  {(uint8_t)245, (uint8_t)159, (uint8_t)61, (uint8_t)197, (uint8_t)197, (uint8_t)228, (uint8_t)25, (uint8_t)101, (uint8_t)82, (uint8_t)158, (uint8_t)251, (uint8_t)179, (uint8_t)189, (uint8_t)73, (uint8_t)16, (uint8_t)181, (uint8_t)3, (uint8_t)56, (uint8_t)130, (uint8_t)237, (uint8_t)138, (uint8_t)66, (uint8_t)69, (uint8_t)238, (uint8_t)122, (uint8_t)21, (uint8_t)115, (uint8_t)227, (uint8_t)125, (uint8_t)14, (uint8_t)69, (uint8_t)182, (uint8_t)106, (uint8_t)98, (uint8_t)51, (uint8_t)68, (uint8_t)91, (uint8_t)164, (uint8_t)45, (uint8_t)229, (uint8_t)209, (uint8_t)6, (uint8_t)225, (uint8_t)101, (uint8_t)60, (uint8_t)113, (uint8_t)141, (uint8_t)129, (uint8_t)32, (uint8_t)237, (uint8_t)63, (uint8_t)178, (uint8_t)9, (uint8_t)140, (uint8_t)163, (uint8_t)235, (uint8_t)108, (uint8_t)252, (uint8_t)146, (uint8_t)188, (uint8_t)80, (uint8_t)238, (uint8_t)93, (uint8_t)146, (uint8_t)35, (uint8_t)209, (uint8_t)5, (uint8_t)96, (uint8_t)55, (uint8_t)103, (uint8_t)19, (uint8_t)157, (uint8_t)31, (uint8_t)142, (uint8_t)130, (uint8_t)41, (uint8_t)13, (uint8_t)107, (uint8_t)88, (uint8_t)100, (uint8_t)68, (uint8_t)11, (uint8_t)53, (uint8_t)121, (uint8_t)167, (uint8_t)56, (uint8_t)113, (uint8_t)220, (uint8_t)181, (uint8_t)204, (uint8_t)131, (uint8_t)255, (uint8_t)203, (uint8_t)147, (uint8_t)101, (uint8_t)141, (uint8_t)140, (uint8_t)205, (uint8_t)218, (uint8_t)55, (uint8_t)93, (uint8_t)107, (uint8_t)224, (uint8_t)168, (uint8_t)246, (uint8_t)83, (uint8_t)79, (uint8_t)17, (uint8_t)171, (uint8_t)33, (uint8_t)177, (uint8_t)79, (uint8_t)103, (uint8_t)192, (uint8_t)187, (uint8_t)136, (uint8_t)49, (uint8_t)143, (uint8_t)90, (uint8_t)5, (uint8_t)102, (uint8_t)168, (uint8_t)203, (uint8_t)178, (uint8_t)213, (uint8_t)40, (uint8_t)62, (uint8_t)252, (uint8_t)130, (uint8_t)10, (uint8_t)115, (uint8_t)93, (uint8_t)6, (uint8_t)193, (uint8_t)7, (uint8_t)109, (uint8_t)73, (uint8_t)228, (uint8_t)102, (uint8_t)219, (uint8_t)245, (uint8_t)95, (uint8_t)148, (uint8_t)86, (uint8_t)27, (uint8_t)251, (uint8_t)79, (uint8_t)186, (uint8_t)48, (uint8_t)95, (uint8_t)91, (uint8_t)23, (uint8_t)248, (uint8_t)129, (uint8_t)23, (uint8_t)70, (uint8_t)227, (uint8_t)221, (uint8_t)201, (uint8_t)4, (uint8_t)85, (uint8_t)195, (uint8_t)11, (uint8_t)251, (uint8_t)19, (uint8_t)128, (uint8_t)136, (uint8_t)244, (uint8_t)235, (uint8_t)62, (uint8_t)117, (uint8_t)56, (uint8_t)107, (uint8_t)98, (uint8_t)190, (uint8_t)36, (uint8_t)156, (uint8_t)84, (uint8_t)180, (uint8_t)46, (uint8_t)214, (uint8_t)192, (uint8_t)199, (uint8_t)65, (uint8_t)100, (uint8_t)56, (uint8_t)217, (uint8_t)158, (uint8_t)45, (uint8_t)185, (uint8_t)147, (uint8_t)130, (uint8_t)168, (uint8_t)102, (uint8_t)1, (uint8_t)220, (uint8_t)119, (uint8_t)230, (uint8_t)206, (uint8_t)205, (uint8_t)223, (uint8_t)180, (uint8_t)51, (uint8_t)233, (uint8_t)72, (uint8_t)81, (uint8_t)49, (uint8_t)24, (uint8_t)111, (uint8_t)134, (uint8_t)26, (uint8_t)108, (uint8_t)73, (uint8_t)17, (uint8_t)54, (uint8_t)234, (uint8_t)76, (uint8_t)92, (uint8_t)156, (uint8_t)39, (uint8_t)213, (uint8_t)1, (uint8_t)79, (uint8_t)173, (uint8_t)247, (uint8_t)186, (uint8_t)46, (uint8_t)133, (uint8_t)76, (uint8_t)81, (uint8_t)59, (uint8_t)232, (uint8_t)80, (uint8_t)129, (uint8_t)160, (uint8_t)214, (uint8_t)40, (uint8_t)77, (uint8_t)89, (uint8_t)247, (uint8_t)155, (uint8_t)36, (uint8_t)194, (uint8_t)247, (uint8_t)105, (uint8_t)26, (uint8_t)99, (uint8_t)57, (uint8_t)224} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)47372);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)50355);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)11657);
    assert(p269_bitrate_GET(pack) == (uint32_t)1249559602L);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)28809);
    assert(p269_framerate_GET(pack) == (float)1.4655262E38F);
    assert(p269_uri_LEN(ph) == 46);
    {
        char16_t * exemplary = u"ubpDibyyaLearvNScifriJvmaegdzdgcqclcdjrtmsmfim";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 92);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)35214);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p270_uri_LEN(ph) == 181);
    {
        char16_t * exemplary = u"sojfzkvtainwsRQomFwcggszqvzmiuqswwaqhufjgwdyPpznusxxEotwkfmjapwvDzJnuatuonkirqnNvbuczxroietwxdrgmfmmzwwtflrfixtcimstyfqwbouejmfenmydtoscemnxgioscaftyukrdtazqzyoHldswKvjoavqjksmxibqB";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 362);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)805819606L);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)62639);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)51216);
    assert(p270_framerate_GET(pack) == (float) -1.0650112E38F);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)109);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 25);
    {
        char16_t * exemplary = u"rqyMecqhgDUnxTnqnpDePfnSb";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 5);
    {
        char16_t * exemplary = u"siezq";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)242, (uint8_t)185, (uint8_t)171, (uint8_t)166, (uint8_t)240, (uint8_t)198, (uint8_t)244, (uint8_t)126} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)3507);
    {
        uint8_t exemplary[] =  {(uint8_t)164, (uint8_t)58, (uint8_t)96, (uint8_t)199, (uint8_t)79, (uint8_t)154, (uint8_t)233, (uint8_t)225} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)25910);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)9587);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1280658777L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_time_usec_GET(pack) == (uint64_t)6753786523748159209L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)56639);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_name_LEN(ph) == 75);
    {
        char16_t * exemplary = u"fwgtsvmjmcwOrgqkwdaCrdjAxszaggimuggktPvhjzmIvuzyBeoztamaEkdKrdgWzhoqCsEuybs";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 150);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)162, (uint8_t)192, (uint8_t)163, (uint8_t)157, (uint8_t)177, (uint8_t)41, (uint8_t)87, (uint8_t)185, (uint8_t)165, (uint8_t)215, (uint8_t)89, (uint8_t)46, (uint8_t)13, (uint8_t)121, (uint8_t)200} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p311_time_usec_GET(pack) == (uint64_t)2551302545452401980L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)455695534L);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2512533801L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"gjplcwNbvyjyfM";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -383);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)54);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)57780);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)9090);
    assert(p322_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"ixdzzqxclqlp";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 125);
    {
        char16_t * exemplary = u"qyBPnVkpalotywbTiykgzoojuvcHoynfiunGtrgrpkugnetAigntBmzhiflvwozKwRbjEJjhhuIonqXgwmxhxigjdslpoiafiXHjhhsspRmdoaukpcjMdDbgdwfzp";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 250);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p323_param_value_LEN(ph) == 15);
    {
        char16_t * exemplary = u"GokjxnnrwvtwyJt";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p323_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"mwygplgyq";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_value_LEN(ph) == 74);
    {
        char16_t * exemplary = u"xkuhcilnjsdkrvbuWcpjdbshqfitUdrPfzxxxsemxnjGrvrfnqxeYncadvtfmngxrjcmMbnygc";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 148);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p324_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"gxrsgvkecveri";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_time_usec_GET(pack) == (uint64_t)8733638146357056116L);
    {
        uint16_t exemplary[] =  {(uint16_t)14070, (uint16_t)63973, (uint16_t)24813, (uint16_t)6500, (uint16_t)39927, (uint16_t)6256, (uint16_t)30108, (uint16_t)11781, (uint16_t)58938, (uint16_t)22138, (uint16_t)46200, (uint16_t)24786, (uint16_t)12770, (uint16_t)5362, (uint16_t)58826, (uint16_t)22335, (uint16_t)48679, (uint16_t)41866, (uint16_t)6469, (uint16_t)65299, (uint16_t)31415, (uint16_t)32494, (uint16_t)2394, (uint16_t)18522, (uint16_t)21107, (uint16_t)59942, (uint16_t)10674, (uint16_t)50718, (uint16_t)28940, (uint16_t)37083, (uint16_t)43123, (uint16_t)57110, (uint16_t)50185, (uint16_t)51344, (uint16_t)54, (uint16_t)6414, (uint16_t)53310, (uint16_t)61757, (uint16_t)3695, (uint16_t)24632, (uint16_t)56161, (uint16_t)2907, (uint16_t)4007, (uint16_t)52954, (uint16_t)9007, (uint16_t)61125, (uint16_t)60359, (uint16_t)62162, (uint16_t)64203, (uint16_t)62014, (uint16_t)40322, (uint16_t)7618, (uint16_t)20236, (uint16_t)27210, (uint16_t)58967, (uint16_t)29004, (uint16_t)45884, (uint16_t)599, (uint16_t)15741, (uint16_t)14926, (uint16_t)30230, (uint16_t)17048, (uint16_t)6696, (uint16_t)46269, (uint16_t)7873, (uint16_t)23352, (uint16_t)17144, (uint16_t)5402, (uint16_t)39849, (uint16_t)48204, (uint16_t)23765, (uint16_t)59991} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)52364);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)4835);
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
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)3987814143L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_current_battery_SET((int16_t)(int16_t) -12968, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)44729, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)39756, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)16898, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)52738, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)34646, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)37769, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -51, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)18270, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)40795, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG), PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)7149049800920340969L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)271560254L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_rate_SET((float)4.842398E37F, PH.base.pack) ;
        p3_afx_SET((float) -4.3526837E37F, PH.base.pack) ;
        p3_vz_SET((float) -1.1321557E38F, PH.base.pack) ;
        p3_yaw_SET((float)4.8177487E37F, PH.base.pack) ;
        p3_vy_SET((float)9.359445E37F, PH.base.pack) ;
        p3_x_SET((float)1.6999316E38F, PH.base.pack) ;
        p3_z_SET((float) -2.0352122E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)370, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)4056374365L, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p3_vx_SET((float)3.3343386E37F, PH.base.pack) ;
        p3_afy_SET((float)2.8962591E38F, PH.base.pack) ;
        p3_afz_SET((float)2.025005E38F, PH.base.pack) ;
        p3_y_SET((float)2.5373217E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)7454558731081332350L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p4_seq_SET((uint32_t)4241381451L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        {
            char16_t* passkey = u"hyjkchhawvjsiwHytomv";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"XPqaHaDodIQgo";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)4232978769L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        {
            char16_t* param_id = u"veixYmgsybn";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t) -6913, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"tbcuJ";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float) -1.4223395E37F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)7274, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)25632, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        {
            char16_t* param_id = u"NtfSxh";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p23_param_value_SET((float) -1.7118003E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_hdg_acc_SET((uint32_t)2417298517L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p24_alt_SET((int32_t)1613165542, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)1783650429L, &PH) ;
        p24_lat_SET((int32_t) -237063330, PH.base.pack) ;
        p24_lon_SET((int32_t) -944980178, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)3115386608L, &PH) ;
        p24_time_usec_SET((uint64_t)4883894581440238095L, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -661520520, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)16941, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)3554, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)30608, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)11051, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)72307326L, &PH) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_elevation[] =  {(uint8_t)192, (uint8_t)67, (uint8_t)126, (uint8_t)68, (uint8_t)146, (uint8_t)83, (uint8_t)171, (uint8_t)69, (uint8_t)195, (uint8_t)244, (uint8_t)190, (uint8_t)174, (uint8_t)166, (uint8_t)88, (uint8_t)63, (uint8_t)152, (uint8_t)11, (uint8_t)35, (uint8_t)169, (uint8_t)124};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        {
            uint8_t satellite_snr[] =  {(uint8_t)170, (uint8_t)50, (uint8_t)35, (uint8_t)131, (uint8_t)227, (uint8_t)128, (uint8_t)169, (uint8_t)87, (uint8_t)247, (uint8_t)172, (uint8_t)219, (uint8_t)254, (uint8_t)28, (uint8_t)239, (uint8_t)81, (uint8_t)116, (uint8_t)234, (uint8_t)76, (uint8_t)181, (uint8_t)175};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)178, (uint8_t)167, (uint8_t)127, (uint8_t)155, (uint8_t)170, (uint8_t)73, (uint8_t)238, (uint8_t)68, (uint8_t)249, (uint8_t)169, (uint8_t)8, (uint8_t)218, (uint8_t)80, (uint8_t)204, (uint8_t)254, (uint8_t)76, (uint8_t)222, (uint8_t)15, (uint8_t)143, (uint8_t)166};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)12, (uint8_t)134, (uint8_t)85, (uint8_t)188, (uint8_t)195, (uint8_t)239, (uint8_t)173, (uint8_t)54, (uint8_t)0, (uint8_t)49, (uint8_t)207, (uint8_t)133, (uint8_t)18, (uint8_t)116, (uint8_t)123, (uint8_t)172, (uint8_t)131, (uint8_t)39, (uint8_t)209, (uint8_t)110};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)132, (uint8_t)129, (uint8_t)120, (uint8_t)54, (uint8_t)80, (uint8_t)166, (uint8_t)71, (uint8_t)96, (uint8_t)62, (uint8_t)60, (uint8_t)52, (uint8_t)196, (uint8_t)114, (uint8_t)253, (uint8_t)38, (uint8_t)116, (uint8_t)247, (uint8_t)15, (uint8_t)153, (uint8_t)189};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_ymag_SET((int16_t)(int16_t) -20548, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)18130, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)4470, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3949195841L, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)9190, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -19728, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)9708, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)20975, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -17989, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -5323, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t) -21620, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)18720, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -21018, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -31699, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)1310, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)748, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)5206282137199934679L, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)18098, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -205, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)2207, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -29528, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)6919, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -23042, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)18, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)1688610272083548404L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)4259, PH.base.pack) ;
        p29_press_diff_SET((float) -1.9821772E38F, PH.base.pack) ;
        p29_press_abs_SET((float)2.9046435E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3276141460L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)2.3555457E38F, PH.base.pack) ;
        p30_yaw_SET((float)2.2750381E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)1.6823054E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)1.6192204E38F, PH.base.pack) ;
        p30_pitch_SET((float)7.528729E37F, PH.base.pack) ;
        p30_roll_SET((float) -8.611308E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)1568594698L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -1.290549E38F, PH.base.pack) ;
        p31_q1_SET((float)2.4568673E38F, PH.base.pack) ;
        p31_q2_SET((float) -1.9485037E38F, PH.base.pack) ;
        p31_q3_SET((float)9.720614E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1343082906L, PH.base.pack) ;
        p31_rollspeed_SET((float) -2.0625866E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)2.4810038E38F, PH.base.pack) ;
        p31_q4_SET((float)3.2381484E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_z_SET((float) -7.559855E37F, PH.base.pack) ;
        p32_x_SET((float)6.562412E37F, PH.base.pack) ;
        p32_vx_SET((float) -3.0495612E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)527139452L, PH.base.pack) ;
        p32_y_SET((float)2.8301415E38F, PH.base.pack) ;
        p32_vy_SET((float) -1.8647647E38F, PH.base.pack) ;
        p32_vz_SET((float)2.4358691E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_vy_SET((int16_t)(int16_t)24003, PH.base.pack) ;
        p33_alt_SET((int32_t)200097094, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)46195, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)3064660787L, PH.base.pack) ;
        p33_lat_SET((int32_t)1024000856, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -1103683449, PH.base.pack) ;
        p33_lon_SET((int32_t)1751368219, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)26421, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)6149, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan2_scaled_SET((int16_t)(int16_t) -118, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)12310, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -12663, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)19532, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -23621, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)19413, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1892351263L, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -29657, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -20225, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan1_raw_SET((uint16_t)(uint16_t)32452, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)2529802364L, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)53200, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)31007, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)14506, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)31148, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)12399, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)55273, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)19562, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo13_raw_SET((uint16_t)(uint16_t)13672, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)2150, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)33257, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)45049, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)37096, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)59410, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)10299, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)3475, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)12444, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)51078, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)18972, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)52381, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)5029, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)27635, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)65317, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)39252, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)1657687733L, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t) -15390, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -26756, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)31489, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -30083, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p39_x_SET((float) -3.259754E38F, PH.base.pack) ;
        p39_param3_SET((float) -1.8722364E37F, PH.base.pack) ;
        p39_param4_SET((float) -3.3149053E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE, PH.base.pack) ;
        p39_param1_SET((float)3.1294554E37F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p39_y_SET((float)1.7853997E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)46798, PH.base.pack) ;
        p39_z_SET((float) -8.411506E37F, PH.base.pack) ;
        p39_param2_SET((float)1.1960267E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)49233, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)59259, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)19146, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)32058, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)24715, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ERROR, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t)2059712, PH.base.pack) ;
        p48_latitude_SET((int32_t)873751517, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1923219053396279582L, &PH) ;
        p48_altitude_SET((int32_t)666249786, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t)1567629641, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)5142321724795212631L, &PH) ;
        p49_longitude_SET((int32_t)1654399152, PH.base.pack) ;
        p49_altitude_SET((int32_t) -1237381391, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_scale_SET((float)9.039135E37F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -15012, PH.base.pack) ;
        p50_param_value_min_SET((float) -1.1958294E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        {
            char16_t* param_id = u"whtJpcyvdayr";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p50_param_value0_SET((float)1.3165329E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.7567283E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)29370, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float)3.3498724E38F, PH.base.pack) ;
        p54_p1y_SET((float)1.6024705E38F, PH.base.pack) ;
        p54_p1x_SET((float) -2.0251313E38F, PH.base.pack) ;
        p54_p2x_SET((float)1.5915134E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p54_p2y_SET((float)2.8604491E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p54_p2z_SET((float) -6.105977E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float)2.088433E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p55_p1z_SET((float)1.6748643E38F, PH.base.pack) ;
        p55_p1y_SET((float)1.8708155E37F, PH.base.pack) ;
        p55_p2z_SET((float) -1.0958896E38F, PH.base.pack) ;
        p55_p2x_SET((float)1.4166262E38F, PH.base.pack) ;
        p55_p2y_SET((float) -2.8086065E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float)3.0408636E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.0942626E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)7299278394145823672L, PH.base.pack) ;
        p61_yawspeed_SET((float)1.779212E38F, PH.base.pack) ;
        {
            float q[] =  {2.985172E38F, -1.2907591E38F, 1.2891452E38F, -8.851783E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float covariance[] =  {-1.0997585E37F, 1.6336377E38F, -1.0179133E38F, 2.2624322E38F, 2.19328E38F, -1.8191796E38F, 2.6074496E38F, -2.5314877E38F, -1.315054E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_target_bearing_SET((int16_t)(int16_t)3163, PH.base.pack) ;
        p62_nav_pitch_SET((float)1.9230278E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)63575, PH.base.pack) ;
        p62_nav_roll_SET((float) -2.5619959E38F, PH.base.pack) ;
        p62_alt_error_SET((float)1.1575848E37F, PH.base.pack) ;
        p62_aspd_error_SET((float)1.572086E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -15946, PH.base.pack) ;
        p62_xtrack_error_SET((float) -3.0081886E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lat_SET((int32_t)2005155596, PH.base.pack) ;
        p63_lon_SET((int32_t)197955571, PH.base.pack) ;
        p63_vz_SET((float)1.874497E38F, PH.base.pack) ;
        p63_alt_SET((int32_t) -1343901399, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)134086187806832567L, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        {
            float covariance[] =  {8.992675E37F, -1.6390304E38F, 3.153418E38F, 2.1353313E38F, 3.3615877E38F, -1.8219922E38F, 1.4986667E38F, 3.3786647E38F, -1.942408E38F, -9.066253E37F, 4.392285E37F, -9.609272E37F, 2.4104802E38F, -2.4555911E38F, 2.3230322E38F, 1.9258785E38F, -3.1872565E38F, -1.6188197E38F, -1.836981E38F, 3.1984192E38F, -3.0499329E38F, 2.7278794E38F, 2.0470752E38F, 1.8526503E38F, 1.3472153E38F, 9.692387E37F, 1.3123091E38F, -1.5454715E38F, -2.3559465E38F, 1.1614236E38F, 2.2435225E38F, -2.2071724E38F, 2.3686717E38F, 1.2336006E38F, 1.4333819E38F, 1.4278382E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vx_SET((float)2.5787534E38F, PH.base.pack) ;
        p63_vy_SET((float)2.7001655E37F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1649695389, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ax_SET((float)2.673099E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)7008051352561144337L, PH.base.pack) ;
        p64_vy_SET((float)3.2233795E38F, PH.base.pack) ;
        p64_ay_SET((float) -2.3661966E38F, PH.base.pack) ;
        p64_y_SET((float)2.5544973E38F, PH.base.pack) ;
        p64_az_SET((float)1.3892256E38F, PH.base.pack) ;
        p64_vz_SET((float)1.9142197E38F, PH.base.pack) ;
        p64_vx_SET((float)2.328527E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.4369427E38F, -3.3208042E38F, -1.3389458E38F, -9.430279E37F, -1.8597765E38F, 3.3384883E38F, -4.4910934E37F, 1.8883625E38F, 2.129482E38F, -2.0884361E38F, 3.0508573E38F, 1.93677E38F, -1.7797244E38F, 2.1108119E38F, 3.3125826E38F, 2.899527E38F, -3.787093E37F, -1.3114915E38F, 2.440276E38F, 1.9818362E38F, 1.113373E38F, -2.6944157E38F, -9.090668E37F, -2.9472838E38F, 2.612785E38F, 2.6178303E38F, 1.6223046E38F, 1.8090905E38F, -2.535822E38F, 2.0534129E38F, -3.4489964E37F, 4.8771307E36F, 1.2909491E37F, -1.5376475E38F, 1.4806603E38F, -9.528364E37F, 1.4519774E38F, 1.9520022E38F, 2.5179975E38F, -2.6344394E38F, -2.7592072E38F, -2.4105396E38F, 3.5974768E37F, 2.239268E38F, -1.2152455E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_x_SET((float)1.6033444E38F, PH.base.pack) ;
        p64_z_SET((float) -1.6698122E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan16_raw_SET((uint16_t)(uint16_t)10739, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)17483, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)11308, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)37563, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)1363, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)61282, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)13460, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)43, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)60204, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)59555, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)38146, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3740241577L, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)10440, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)20493, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)21752, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)30313, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)64818, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)47613, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)8073, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)30826, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)24066, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)31066, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -17020, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)18700, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -4472, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)299, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan2_raw_SET((uint16_t)(uint16_t)34591, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)15019, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)13956, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)58918, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)35951, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)10152, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)6402, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)259, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_y_SET((int32_t)825533370, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_x_SET((int32_t) -367111639, PH.base.pack) ;
        p73_param1_SET((float) -2.3333663E38F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p73_param4_SET((float) -1.0674067E38F, PH.base.pack) ;
        p73_z_SET((float) -2.1168045E38F, PH.base.pack) ;
        p73_param2_SET((float)1.0252338E38F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)30545, PH.base.pack) ;
        p73_param3_SET((float)2.4616499E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float)2.8218693E38F, PH.base.pack) ;
        p74_climb_SET((float) -4.1114863E37F, PH.base.pack) ;
        p74_airspeed_SET((float)9.069406E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)47260, PH.base.pack) ;
        p74_groundspeed_SET((float) -2.9335299E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)4356, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_autocontinue_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL, PH.base.pack) ;
        p75_param2_SET((float)2.923574E38F, PH.base.pack) ;
        p75_z_SET((float) -1.301891E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p75_param1_SET((float) -1.781822E38F, PH.base.pack) ;
        p75_x_SET((int32_t) -1804366421, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p75_y_SET((int32_t) -1031454205, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p75_param3_SET((float) -1.5840722E38F, PH.base.pack) ;
        p75_param4_SET((float)1.9604083E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p76_param4_SET((float) -3.3606957E38F, PH.base.pack) ;
        p76_param5_SET((float)1.6715292E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p76_param6_SET((float)1.2219337E38F, PH.base.pack) ;
        p76_param3_SET((float)2.7163288E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS, PH.base.pack) ;
        p76_param2_SET((float) -2.1814985E38F, PH.base.pack) ;
        p76_param1_SET((float)1.885954E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p76_param7_SET((float) -2.5346562E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL, PH.base.pack) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)53, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)186, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)158, &PH) ;
        p77_result_param2_SET((int32_t) -1641692374, &PH) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p81_thrust_SET((float) -5.0912803E37F, PH.base.pack) ;
        p81_yaw_SET((float) -4.784882E36F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p81_roll_SET((float)1.3877806E38F, PH.base.pack) ;
        p81_pitch_SET((float)6.9463795E37F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3801605090L, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_roll_rate_SET((float) -3.3070327E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -2.9375065E38F, PH.base.pack) ;
        {
            float q[] =  {1.7283897E38F, -5.048295E37F, -9.709133E37F, 3.280587E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_thrust_SET((float) -2.249642E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)2651332951L, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -1.006726E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        {
            float q[] =  {-6.871629E36F, 2.6017701E38F, -8.345181E37F, -2.9318444E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p83_body_roll_rate_SET((float)2.8643188E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -8.44872E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)1973478928L, PH.base.pack) ;
        p83_body_pitch_rate_SET((float) -1.3850263E38F, PH.base.pack) ;
        p83_thrust_SET((float)2.0417495E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_type_mask_SET((uint16_t)(uint16_t)6833, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)685642074L, PH.base.pack) ;
        p84_vz_SET((float)7.9023843E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p84_vy_SET((float)3.2036352E38F, PH.base.pack) ;
        p84_z_SET((float) -3.8447377E36F, PH.base.pack) ;
        p84_afx_SET((float)3.0762434E38F, PH.base.pack) ;
        p84_afy_SET((float) -3.0810463E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p84_vx_SET((float)1.9501916E38F, PH.base.pack) ;
        p84_yaw_SET((float) -1.1869339E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p84_afz_SET((float)1.7047067E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.0530615E38F, PH.base.pack) ;
        p84_x_SET((float) -7.861003E36F, PH.base.pack) ;
        p84_y_SET((float)6.43701E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_lat_int_SET((int32_t)269272145, PH.base.pack) ;
        p86_afy_SET((float)1.189635E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3139229058L, PH.base.pack) ;
        p86_afx_SET((float)1.1961693E38F, PH.base.pack) ;
        p86_alt_SET((float)1.0759623E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)30320, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p86_vx_SET((float)3.3075128E38F, PH.base.pack) ;
        p86_vz_SET((float) -1.8338479E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p86_yaw_rate_SET((float)3.3033407E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p86_vy_SET((float)1.4527239E38F, PH.base.pack) ;
        p86_afz_SET((float)2.0050444E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)1233219105, PH.base.pack) ;
        p86_yaw_SET((float)2.8374817E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afy_SET((float)2.6227259E38F, PH.base.pack) ;
        p87_afx_SET((float) -2.7659584E38F, PH.base.pack) ;
        p87_vz_SET((float) -1.5103252E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p87_yaw_SET((float)1.6378949E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)58709, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.1799289E38F, PH.base.pack) ;
        p87_alt_SET((float) -3.3465511E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2749554933L, PH.base.pack) ;
        p87_vy_SET((float)1.6826324E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1237434558, PH.base.pack) ;
        p87_afz_SET((float) -6.318017E37F, PH.base.pack) ;
        p87_vx_SET((float)5.409404E37F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -1344863223, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_time_boot_ms_SET((uint32_t)2581991961L, PH.base.pack) ;
        p89_roll_SET((float)1.2011131E38F, PH.base.pack) ;
        p89_y_SET((float)2.1871656E37F, PH.base.pack) ;
        p89_yaw_SET((float)2.1783194E38F, PH.base.pack) ;
        p89_pitch_SET((float) -3.0061614E37F, PH.base.pack) ;
        p89_x_SET((float)3.025476E38F, PH.base.pack) ;
        p89_z_SET((float) -6.225537E37F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_yaw_SET((float)1.5356899E38F, PH.base.pack) ;
        p90_alt_SET((int32_t) -1201546970, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -30850, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)26843, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -25501, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2781226436743815201L, PH.base.pack) ;
        p90_pitchspeed_SET((float)2.4272454E38F, PH.base.pack) ;
        p90_yawspeed_SET((float) -2.0266133E38F, PH.base.pack) ;
        p90_rollspeed_SET((float)2.8651346E38F, PH.base.pack) ;
        p90_pitch_SET((float)2.1765747E38F, PH.base.pack) ;
        p90_roll_SET((float)2.511215E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)14331, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -30622, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)13665, PH.base.pack) ;
        p90_lon_SET((int32_t) -951764448, PH.base.pack) ;
        p90_lat_SET((int32_t)989732767, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_time_usec_SET((uint64_t)8316745193885672204L, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -1.9758152E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -1.8279102E38F, PH.base.pack) ;
        p91_aux1_SET((float)3.331718E38F, PH.base.pack) ;
        p91_aux2_SET((float)3.0482997E38F, PH.base.pack) ;
        p91_throttle_SET((float)1.3298584E38F, PH.base.pack) ;
        p91_aux3_SET((float) -5.6478734E36F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)3.3393044E38F, PH.base.pack) ;
        p91_aux4_SET((float) -3.1848476E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan3_raw_SET((uint16_t)(uint16_t)36011, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)59112, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)3663797172986814734L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)30286, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)20803, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)54161, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)1525, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)8973, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)25623, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)6333, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)48405, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)30406, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)2079, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {-8.0268924E37F, -1.2366151E38F, 1.7417728E38F, -2.3602995E38F, -1.6827568E38F, -2.9575138E38F, 3.0938272E38F, -1.1506472E38F, -2.0907603E38F, -1.6707755E38F, 1.0218559E38F, -2.7035932E37F, 1.2965905E38F, 2.1756868E38F, 1.9776948E38F, -3.354939E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_flags_SET((uint64_t)6526834267480741586L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)221181000520005436L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_x_SET((int16_t)(int16_t)17050, PH.base.pack) ;
        p100_flow_rate_x_SET((float)1.4115999E38F, &PH) ;
        p100_ground_distance_SET((float)9.683519E36F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)1.1704209E38F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)24702, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)1.8206489E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)7.1048115E37F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)412618383370504699L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)8118770812204925674L, PH.base.pack) ;
        p101_pitch_SET((float)1.3967989E38F, PH.base.pack) ;
        p101_y_SET((float) -2.6963281E38F, PH.base.pack) ;
        p101_x_SET((float)1.4915651E38F, PH.base.pack) ;
        p101_roll_SET((float) -7.450939E37F, PH.base.pack) ;
        p101_yaw_SET((float)2.7876129E37F, PH.base.pack) ;
        p101_z_SET((float) -2.3277199E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_z_SET((float)4.9711375E37F, PH.base.pack) ;
        p102_yaw_SET((float) -1.4256395E38F, PH.base.pack) ;
        p102_y_SET((float) -3.0788213E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)6703553654120414539L, PH.base.pack) ;
        p102_pitch_SET((float) -1.4892465E38F, PH.base.pack) ;
        p102_x_SET((float) -3.067035E38F, PH.base.pack) ;
        p102_roll_SET((float)1.0264344E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float) -2.4832367E38F, PH.base.pack) ;
        p103_y_SET((float) -3.0625459E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)8065180117579933809L, PH.base.pack) ;
        p103_z_SET((float)6.5736E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float) -1.1747345E38F, PH.base.pack) ;
        p104_pitch_SET((float) -8.917477E36F, PH.base.pack) ;
        p104_usec_SET((uint64_t)1206017022636779189L, PH.base.pack) ;
        p104_roll_SET((float)6.020918E36F, PH.base.pack) ;
        p104_yaw_SET((float) -2.9551025E38F, PH.base.pack) ;
        p104_y_SET((float)2.8818974E38F, PH.base.pack) ;
        p104_x_SET((float)9.377127E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zgyro_SET((float) -1.908982E38F, PH.base.pack) ;
        p105_zacc_SET((float)2.8598092E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.5565715E38F, PH.base.pack) ;
        p105_xgyro_SET((float) -1.6830517E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)36403, PH.base.pack) ;
        p105_temperature_SET((float)2.959566E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)1.2186993E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)1.7656933E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.2934097E38F, PH.base.pack) ;
        p105_zmag_SET((float)3.0253713E38F, PH.base.pack) ;
        p105_xacc_SET((float)8.671136E37F, PH.base.pack) ;
        p105_yacc_SET((float)1.9135076E37F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)2531961716453451517L, PH.base.pack) ;
        p105_pressure_alt_SET((float) -1.0170709E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -6.1699587E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_temperature_SET((int16_t)(int16_t)30848, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)3.6370214E37F, PH.base.pack) ;
        p106_distance_SET((float) -6.7511136E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)2862142505220433122L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)2.1265676E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)1.0314596E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3015893999L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p106_integrated_y_SET((float) -1.1055013E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p106_integrated_x_SET((float)1.3031659E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)748522867L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_zgyro_SET((float)1.9269397E38F, PH.base.pack) ;
        p107_ymag_SET((float)7.734063E37F, PH.base.pack) ;
        p107_xmag_SET((float)1.8548197E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -3.3884674E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -1.8690873E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)9.421345E37F, PH.base.pack) ;
        p107_yacc_SET((float)2.968856E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)3478981554L, PH.base.pack) ;
        p107_zmag_SET((float) -1.6180555E38F, PH.base.pack) ;
        p107_ygyro_SET((float)1.7916848E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)3235133965418775373L, PH.base.pack) ;
        p107_pressure_alt_SET((float)3.290046E38F, PH.base.pack) ;
        p107_xacc_SET((float)7.867336E37F, PH.base.pack) ;
        p107_zacc_SET((float) -3.2575795E38F, PH.base.pack) ;
        p107_temperature_SET((float)2.7398995E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_xgyro_SET((float)1.5496138E38F, PH.base.pack) ;
        p108_vn_SET((float)2.0313687E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -9.790762E37F, PH.base.pack) ;
        p108_zacc_SET((float) -2.1743911E38F, PH.base.pack) ;
        p108_zgyro_SET((float)1.9061439E38F, PH.base.pack) ;
        p108_xacc_SET((float) -1.9483755E38F, PH.base.pack) ;
        p108_yaw_SET((float)1.0419518E38F, PH.base.pack) ;
        p108_yacc_SET((float)1.4546914E38F, PH.base.pack) ;
        p108_vd_SET((float)1.1004289E38F, PH.base.pack) ;
        p108_pitch_SET((float)1.7841411E38F, PH.base.pack) ;
        p108_q4_SET((float) -1.5925506E38F, PH.base.pack) ;
        p108_q3_SET((float) -2.8979748E38F, PH.base.pack) ;
        p108_ygyro_SET((float)3.1993307E38F, PH.base.pack) ;
        p108_alt_SET((float) -1.7902543E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)1.8342624E38F, PH.base.pack) ;
        p108_lon_SET((float)6.6733123E37F, PH.base.pack) ;
        p108_q1_SET((float) -1.7923815E38F, PH.base.pack) ;
        p108_ve_SET((float)1.858377E38F, PH.base.pack) ;
        p108_lat_SET((float) -5.468513E37F, PH.base.pack) ;
        p108_roll_SET((float)1.4552023E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.053697E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_remnoise_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)20199, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)59872, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)143, (uint8_t)168, (uint8_t)25, (uint8_t)55, (uint8_t)48, (uint8_t)78, (uint8_t)201, (uint8_t)125, (uint8_t)249, (uint8_t)206, (uint8_t)83, (uint8_t)94, (uint8_t)238, (uint8_t)122, (uint8_t)125, (uint8_t)57, (uint8_t)196, (uint8_t)204, (uint8_t)203, (uint8_t)19, (uint8_t)172, (uint8_t)199, (uint8_t)221, (uint8_t)67, (uint8_t)14, (uint8_t)93, (uint8_t)160, (uint8_t)4, (uint8_t)53, (uint8_t)188, (uint8_t)120, (uint8_t)46, (uint8_t)162, (uint8_t)118, (uint8_t)147, (uint8_t)143, (uint8_t)175, (uint8_t)137, (uint8_t)28, (uint8_t)140, (uint8_t)48, (uint8_t)182, (uint8_t)51, (uint8_t)231, (uint8_t)231, (uint8_t)188, (uint8_t)93, (uint8_t)63, (uint8_t)212, (uint8_t)61, (uint8_t)165, (uint8_t)224, (uint8_t)154, (uint8_t)166, (uint8_t)174, (uint8_t)9, (uint8_t)231, (uint8_t)186, (uint8_t)185, (uint8_t)227, (uint8_t)137, (uint8_t)149, (uint8_t)140, (uint8_t)109, (uint8_t)5, (uint8_t)235, (uint8_t)8, (uint8_t)224, (uint8_t)252, (uint8_t)195, (uint8_t)17, (uint8_t)109, (uint8_t)64, (uint8_t)171, (uint8_t)30, (uint8_t)181, (uint8_t)153, (uint8_t)95, (uint8_t)15, (uint8_t)8, (uint8_t)211, (uint8_t)2, (uint8_t)232, (uint8_t)220, (uint8_t)76, (uint8_t)82, (uint8_t)148, (uint8_t)37, (uint8_t)112, (uint8_t)217, (uint8_t)230, (uint8_t)201, (uint8_t)18, (uint8_t)23, (uint8_t)178, (uint8_t)215, (uint8_t)194, (uint8_t)205, (uint8_t)104, (uint8_t)77, (uint8_t)5, (uint8_t)207, (uint8_t)199, (uint8_t)220, (uint8_t)82, (uint8_t)230, (uint8_t)226, (uint8_t)147, (uint8_t)208, (uint8_t)164, (uint8_t)238, (uint8_t)219, (uint8_t)83, (uint8_t)54, (uint8_t)238, (uint8_t)214, (uint8_t)12, (uint8_t)78, (uint8_t)28, (uint8_t)52, (uint8_t)116, (uint8_t)231, (uint8_t)23, (uint8_t)246, (uint8_t)8, (uint8_t)86, (uint8_t)186, (uint8_t)99, (uint8_t)80, (uint8_t)163, (uint8_t)207, (uint8_t)72, (uint8_t)54, (uint8_t)63, (uint8_t)78, (uint8_t)17, (uint8_t)177, (uint8_t)126, (uint8_t)52, (uint8_t)13, (uint8_t)149, (uint8_t)79, (uint8_t)231, (uint8_t)176, (uint8_t)127, (uint8_t)134, (uint8_t)213, (uint8_t)20, (uint8_t)104, (uint8_t)199, (uint8_t)106, (uint8_t)25, (uint8_t)106, (uint8_t)210, (uint8_t)157, (uint8_t)111, (uint8_t)4, (uint8_t)66, (uint8_t)217, (uint8_t)188, (uint8_t)24, (uint8_t)47, (uint8_t)117, (uint8_t)30, (uint8_t)65, (uint8_t)191, (uint8_t)76, (uint8_t)64, (uint8_t)75, (uint8_t)227, (uint8_t)21, (uint8_t)161, (uint8_t)203, (uint8_t)27, (uint8_t)203, (uint8_t)104, (uint8_t)27, (uint8_t)149, (uint8_t)7, (uint8_t)241, (uint8_t)166, (uint8_t)139, (uint8_t)1, (uint8_t)89, (uint8_t)165, (uint8_t)26, (uint8_t)231, (uint8_t)116, (uint8_t)103, (uint8_t)159, (uint8_t)8, (uint8_t)27, (uint8_t)107, (uint8_t)163, (uint8_t)221, (uint8_t)204, (uint8_t)240, (uint8_t)15, (uint8_t)146, (uint8_t)58, (uint8_t)204, (uint8_t)12, (uint8_t)6, (uint8_t)228, (uint8_t)195, (uint8_t)131, (uint8_t)41, (uint8_t)249, (uint8_t)138, (uint8_t)226, (uint8_t)220, (uint8_t)248, (uint8_t)6, (uint8_t)167, (uint8_t)224, (uint8_t)133, (uint8_t)150, (uint8_t)113, (uint8_t)66, (uint8_t)190, (uint8_t)223, (uint8_t)198, (uint8_t)240, (uint8_t)121, (uint8_t)13, (uint8_t)111, (uint8_t)224, (uint8_t)97, (uint8_t)49, (uint8_t)78, (uint8_t)197, (uint8_t)152, (uint8_t)165, (uint8_t)201, (uint8_t)2, (uint8_t)107, (uint8_t)234, (uint8_t)183, (uint8_t)62, (uint8_t)174, (uint8_t)155, (uint8_t)224, (uint8_t)3, (uint8_t)52, (uint8_t)9, (uint8_t)59, (uint8_t)24, (uint8_t)175, (uint8_t)221, (uint8_t)28, (uint8_t)203};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)4349254374565009158L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -2689485646187033610L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)1835393123970923715L, PH.base.pack) ;
        p112_seq_SET((uint32_t)3765137406L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_vel_SET((uint16_t)(uint16_t)45056, PH.base.pack) ;
        p113_lon_SET((int32_t)989301673, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -24025, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)49939, PH.base.pack) ;
        p113_lat_SET((int32_t) -557786525, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)9091569222351114604L, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -26274, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)41664, PH.base.pack) ;
        p113_alt_SET((int32_t) -267639183, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)39670, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -9661, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_zgyro_SET((float)1.7435268E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -3.272776E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -2.8288003E37F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3370393311L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)694269199L, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)4278723362360207712L, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)4461, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.6651567E38F, PH.base.pack) ;
        p114_distance_SET((float)3.1374845E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)4.0098007E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_true_airspeed_SET((uint16_t)(uint16_t)30767, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)40919, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)28828, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)11849, PH.base.pack) ;
        p115_lat_SET((int32_t)1205960289, PH.base.pack) ;
        p115_lon_SET((int32_t)1293429311, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -19730, PH.base.pack) ;
        p115_rollspeed_SET((float) -1.6793678E38F, PH.base.pack) ;
        p115_alt_SET((int32_t)1719213762, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)19583, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)22067, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.230368E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {1.49382E38F, -1.7509638E38F, 1.703202E37F, -1.5994473E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_xacc_SET((int16_t)(int16_t) -23728, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)4260791009304458455L, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.1178937E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zgyro_SET((int16_t)(int16_t) -15495, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)1446029072L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)27000, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -17197, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -6612, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)2621, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)6685, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)16005, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)31482, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)588, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)17095, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)10959, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)378694524L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)16607, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1947518875L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)48833, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)64804, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)299600496L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p119_ofs_SET((uint32_t)134805848L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)42429, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)11605, PH.base.pack) ;
        p120_ofs_SET((uint32_t)701919745L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)94, (uint8_t)53, (uint8_t)105, (uint8_t)101, (uint8_t)127, (uint8_t)66, (uint8_t)204, (uint8_t)15, (uint8_t)122, (uint8_t)190, (uint8_t)28, (uint8_t)250, (uint8_t)174, (uint8_t)211, (uint8_t)171, (uint8_t)35, (uint8_t)176, (uint8_t)7, (uint8_t)192, (uint8_t)251, (uint8_t)22, (uint8_t)42, (uint8_t)43, (uint8_t)243, (uint8_t)10, (uint8_t)205, (uint8_t)143, (uint8_t)172, (uint8_t)88, (uint8_t)231, (uint8_t)75, (uint8_t)112, (uint8_t)168, (uint8_t)74, (uint8_t)153, (uint8_t)189, (uint8_t)155, (uint8_t)131, (uint8_t)197, (uint8_t)24, (uint8_t)15, (uint8_t)41, (uint8_t)50, (uint8_t)206, (uint8_t)117, (uint8_t)161, (uint8_t)107, (uint8_t)190, (uint8_t)49, (uint8_t)55, (uint8_t)124, (uint8_t)145, (uint8_t)18, (uint8_t)107, (uint8_t)214, (uint8_t)117, (uint8_t)190, (uint8_t)36, (uint8_t)74, (uint8_t)220, (uint8_t)52, (uint8_t)204, (uint8_t)92, (uint8_t)203, (uint8_t)230, (uint8_t)49, (uint8_t)82, (uint8_t)95, (uint8_t)246, (uint8_t)15, (uint8_t)162, (uint8_t)198, (uint8_t)141, (uint8_t)85, (uint8_t)4, (uint8_t)167, (uint8_t)195, (uint8_t)184, (uint8_t)203, (uint8_t)232, (uint8_t)242, (uint8_t)188, (uint8_t)213, (uint8_t)52, (uint8_t)142, (uint8_t)240, (uint8_t)75, (uint8_t)18, (uint8_t)202, (uint8_t)19};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)205, (uint8_t)70, (uint8_t)162, (uint8_t)154, (uint8_t)251, (uint8_t)60, (uint8_t)171, (uint8_t)243, (uint8_t)189, (uint8_t)232, (uint8_t)20, (uint8_t)45, (uint8_t)191, (uint8_t)146, (uint8_t)55, (uint8_t)121, (uint8_t)124, (uint8_t)85, (uint8_t)164, (uint8_t)86, (uint8_t)140, (uint8_t)166, (uint8_t)87, (uint8_t)209, (uint8_t)212, (uint8_t)249, (uint8_t)92, (uint8_t)90, (uint8_t)172, (uint8_t)39, (uint8_t)193, (uint8_t)98, (uint8_t)192, (uint8_t)45, (uint8_t)113, (uint8_t)88, (uint8_t)164, (uint8_t)119, (uint8_t)232, (uint8_t)250, (uint8_t)184, (uint8_t)85, (uint8_t)20, (uint8_t)156, (uint8_t)123, (uint8_t)94, (uint8_t)244, (uint8_t)47, (uint8_t)60, (uint8_t)41, (uint8_t)197, (uint8_t)237, (uint8_t)209, (uint8_t)152, (uint8_t)169, (uint8_t)27, (uint8_t)122, (uint8_t)125, (uint8_t)161, (uint8_t)233, (uint8_t)122, (uint8_t)225, (uint8_t)173, (uint8_t)57, (uint8_t)1, (uint8_t)202, (uint8_t)46, (uint8_t)139, (uint8_t)246, (uint8_t)30, (uint8_t)91, (uint8_t)37, (uint8_t)138, (uint8_t)71, (uint8_t)72, (uint8_t)192, (uint8_t)44, (uint8_t)251, (uint8_t)111, (uint8_t)105, (uint8_t)229, (uint8_t)138, (uint8_t)231, (uint8_t)46, (uint8_t)36, (uint8_t)207, (uint8_t)104, (uint8_t)215, (uint8_t)6, (uint8_t)44, (uint8_t)147, (uint8_t)6, (uint8_t)175, (uint8_t)224, (uint8_t)15, (uint8_t)204, (uint8_t)33, (uint8_t)128, (uint8_t)46, (uint8_t)81, (uint8_t)160, (uint8_t)42, (uint8_t)1, (uint8_t)65, (uint8_t)118, (uint8_t)176, (uint8_t)70, (uint8_t)132, (uint8_t)55, (uint8_t)172};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_lon_SET((int32_t)881997607, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)201436132L, PH.base.pack) ;
        p124_lat_SET((int32_t)359316989, PH.base.pack) ;
        p124_alt_SET((int32_t) -1391247789, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)57955, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6791614626726841459L, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)8247, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)11607, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)30833, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)107, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)12567, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)41955, PH.base.pack) ;
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI), PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)83, (uint8_t)56, (uint8_t)145, (uint8_t)160, (uint8_t)78, (uint8_t)224, (uint8_t)239, (uint8_t)140, (uint8_t)43, (uint8_t)137, (uint8_t)184, (uint8_t)224, (uint8_t)42, (uint8_t)196, (uint8_t)228, (uint8_t)206, (uint8_t)48, (uint8_t)166, (uint8_t)161, (uint8_t)208, (uint8_t)26, (uint8_t)107, (uint8_t)155, (uint8_t)55, (uint8_t)50, (uint8_t)238, (uint8_t)242, (uint8_t)18, (uint8_t)60, (uint8_t)244, (uint8_t)224, (uint8_t)30, (uint8_t)177, (uint8_t)239, (uint8_t)110, (uint8_t)105, (uint8_t)211, (uint8_t)193, (uint8_t)112, (uint8_t)241, (uint8_t)46, (uint8_t)23, (uint8_t)88, (uint8_t)245, (uint8_t)251, (uint8_t)34, (uint8_t)239, (uint8_t)130, (uint8_t)150, (uint8_t)200, (uint8_t)189, (uint8_t)180, (uint8_t)170, (uint8_t)2, (uint8_t)241, (uint8_t)10, (uint8_t)166, (uint8_t)27, (uint8_t)229, (uint8_t)154, (uint8_t)202, (uint8_t)199, (uint8_t)24, (uint8_t)50, (uint8_t)33, (uint8_t)33, (uint8_t)193, (uint8_t)128, (uint8_t)233, (uint8_t)48};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_baudrate_SET((uint32_t)977200459L, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_a_mm_SET((int32_t)1753853744, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -457147882, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p127_tow_SET((uint32_t)642253106L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -853782139, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)1356473552L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)967647187, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)532127456L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)14848, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_accuracy_SET((uint32_t)694948962L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1379115072, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)506120324, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)1499956976, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)56818, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)324646647, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)3042815528L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p128_tow_SET((uint32_t)3562588082L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_xgyro_SET((int16_t)(int16_t)405, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -11094, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -21749, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -25543, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)20816, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)383, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)352802680L, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -14576, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)3035, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -18511, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_height_SET((uint16_t)(uint16_t)24552, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)53907, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)18680, PH.base.pack) ;
        p130_size_SET((uint32_t)1617041137L, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)187, (uint8_t)82, (uint8_t)168, (uint8_t)199, (uint8_t)61, (uint8_t)5, (uint8_t)24, (uint8_t)164, (uint8_t)208, (uint8_t)167, (uint8_t)124, (uint8_t)130, (uint8_t)1, (uint8_t)181, (uint8_t)4, (uint8_t)197, (uint8_t)100, (uint8_t)73, (uint8_t)4, (uint8_t)126, (uint8_t)218, (uint8_t)234, (uint8_t)166, (uint8_t)171, (uint8_t)57, (uint8_t)233, (uint8_t)80, (uint8_t)72, (uint8_t)129, (uint8_t)56, (uint8_t)148, (uint8_t)95, (uint8_t)96, (uint8_t)76, (uint8_t)70, (uint8_t)96, (uint8_t)224, (uint8_t)107, (uint8_t)23, (uint8_t)202, (uint8_t)142, (uint8_t)45, (uint8_t)241, (uint8_t)219, (uint8_t)78, (uint8_t)241, (uint8_t)97, (uint8_t)136, (uint8_t)101, (uint8_t)38, (uint8_t)178, (uint8_t)82, (uint8_t)182, (uint8_t)167, (uint8_t)112, (uint8_t)181, (uint8_t)129, (uint8_t)200, (uint8_t)241, (uint8_t)18, (uint8_t)247, (uint8_t)228, (uint8_t)189, (uint8_t)71, (uint8_t)176, (uint8_t)162, (uint8_t)97, (uint8_t)216, (uint8_t)29, (uint8_t)74, (uint8_t)108, (uint8_t)59, (uint8_t)234, (uint8_t)46, (uint8_t)168, (uint8_t)253, (uint8_t)171, (uint8_t)82, (uint8_t)144, (uint8_t)214, (uint8_t)123, (uint8_t)127, (uint8_t)127, (uint8_t)61, (uint8_t)253, (uint8_t)91, (uint8_t)119, (uint8_t)191, (uint8_t)174, (uint8_t)4, (uint8_t)79, (uint8_t)84, (uint8_t)231, (uint8_t)55, (uint8_t)136, (uint8_t)169, (uint8_t)42, (uint8_t)212, (uint8_t)199, (uint8_t)75, (uint8_t)57, (uint8_t)108, (uint8_t)116, (uint8_t)203, (uint8_t)164, (uint8_t)189, (uint8_t)124, (uint8_t)13, (uint8_t)101, (uint8_t)204, (uint8_t)194, (uint8_t)19, (uint8_t)82, (uint8_t)62, (uint8_t)9, (uint8_t)41, (uint8_t)201, (uint8_t)251, (uint8_t)169, (uint8_t)77, (uint8_t)255, (uint8_t)70, (uint8_t)69, (uint8_t)96, (uint8_t)242, (uint8_t)105, (uint8_t)106, (uint8_t)232, (uint8_t)212, (uint8_t)103, (uint8_t)111, (uint8_t)178, (uint8_t)148, (uint8_t)86, (uint8_t)76, (uint8_t)191, (uint8_t)109, (uint8_t)60, (uint8_t)20, (uint8_t)157, (uint8_t)235, (uint8_t)125, (uint8_t)8, (uint8_t)127, (uint8_t)161, (uint8_t)144, (uint8_t)99, (uint8_t)15, (uint8_t)255, (uint8_t)213, (uint8_t)179, (uint8_t)142, (uint8_t)91, (uint8_t)77, (uint8_t)186, (uint8_t)37, (uint8_t)228, (uint8_t)119, (uint8_t)195, (uint8_t)224, (uint8_t)15, (uint8_t)220, (uint8_t)225, (uint8_t)79, (uint8_t)85, (uint8_t)111, (uint8_t)66, (uint8_t)229, (uint8_t)152, (uint8_t)155, (uint8_t)71, (uint8_t)116, (uint8_t)67, (uint8_t)98, (uint8_t)11, (uint8_t)146, (uint8_t)180, (uint8_t)205, (uint8_t)73, (uint8_t)159, (uint8_t)127, (uint8_t)189, (uint8_t)166, (uint8_t)161, (uint8_t)3, (uint8_t)244, (uint8_t)126, (uint8_t)163, (uint8_t)57, (uint8_t)236, (uint8_t)2, (uint8_t)161, (uint8_t)253, (uint8_t)86, (uint8_t)163, (uint8_t)237, (uint8_t)4, (uint8_t)101, (uint8_t)219, (uint8_t)73, (uint8_t)53, (uint8_t)161, (uint8_t)233, (uint8_t)242, (uint8_t)120, (uint8_t)40, (uint8_t)241, (uint8_t)138, (uint8_t)101, (uint8_t)193, (uint8_t)224, (uint8_t)122, (uint8_t)20, (uint8_t)228, (uint8_t)79, (uint8_t)117, (uint8_t)128, (uint8_t)20, (uint8_t)17, (uint8_t)53, (uint8_t)192, (uint8_t)41, (uint8_t)107, (uint8_t)27, (uint8_t)151, (uint8_t)175, (uint8_t)61, (uint8_t)100, (uint8_t)18, (uint8_t)20, (uint8_t)79, (uint8_t)123, (uint8_t)55, (uint8_t)250, (uint8_t)89, (uint8_t)23, (uint8_t)110, (uint8_t)78, (uint8_t)221, (uint8_t)221, (uint8_t)61, (uint8_t)134, (uint8_t)103, (uint8_t)156, (uint8_t)17, (uint8_t)139, (uint8_t)22, (uint8_t)241, (uint8_t)183, (uint8_t)33, (uint8_t)232, (uint8_t)74, (uint8_t)138};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)35913, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_time_boot_ms_SET((uint32_t)2329771615L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)7750, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)37953, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)20846, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_90, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t) -498464944, PH.base.pack) ;
        p133_mask_SET((uint64_t)5226407373882906714L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)42856, PH.base.pack) ;
        p133_lon_SET((int32_t) -1381466486, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)52221, PH.base.pack) ;
        p134_lat_SET((int32_t) -1695693672, PH.base.pack) ;
        p134_lon_SET((int32_t)77044834, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -14931, (int16_t) -27367, (int16_t)8130, (int16_t) -9211, (int16_t)11696, (int16_t)18148, (int16_t)8081, (int16_t)30033, (int16_t) -31835, (int16_t)11275, (int16_t)12801, (int16_t)32025, (int16_t)13956, (int16_t)31123, (int16_t) -24388, (int16_t)20705};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)282975167, PH.base.pack) ;
        p135_lon_SET((int32_t) -1376015770, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float)1.6126564E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)35925, PH.base.pack) ;
        p136_lon_SET((int32_t)1391397828, PH.base.pack) ;
        p136_lat_SET((int32_t) -280934409, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)41801, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)26507, PH.base.pack) ;
        p136_terrain_height_SET((float)2.2932576E38F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)3618817596L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -22334, PH.base.pack) ;
        p137_press_abs_SET((float) -7.300449E37F, PH.base.pack) ;
        p137_press_diff_SET((float)8.607557E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {-1.0086597E38F, 7.033651E36F, 1.6166746E38F, 7.4238766E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_z_SET((float)1.8867152E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)8385775580460573572L, PH.base.pack) ;
        p138_x_SET((float) -3.2552685E38F, PH.base.pack) ;
        p138_y_SET((float) -2.1917882E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        {
            float controls[] =  {-5.660196E37F, 1.763918E38F, 2.425814E38F, -2.7227023E37F, 2.499352E38F, 1.1184288E38F, 1.3564305E38F, -1.9668671E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)1911272738448978091L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)5634389353038876433L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        {
            float controls[] =  {-3.3307655E38F, 2.4665438E37F, -2.545521E38F, 1.5849667E38F, -2.5727949E38F, 1.375263E38F, -1.5249613E38F, -1.934323E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDE_141(), &PH);
        p141_time_usec_SET((uint64_t)7329697944692647208L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.499503E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -2.1392024E38F, PH.base.pack) ;
        p141_altitude_local_SET((float)3.1059265E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float)9.621488E36F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -2.3590436E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)3.1155523E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)174, (uint8_t)64, (uint8_t)107, (uint8_t)121, (uint8_t)15, (uint8_t)232, (uint8_t)129, (uint8_t)225, (uint8_t)252, (uint8_t)6, (uint8_t)135, (uint8_t)204, (uint8_t)39, (uint8_t)41, (uint8_t)141, (uint8_t)116, (uint8_t)185, (uint8_t)170, (uint8_t)221, (uint8_t)177, (uint8_t)238, (uint8_t)94, (uint8_t)233, (uint8_t)24, (uint8_t)2, (uint8_t)46, (uint8_t)64, (uint8_t)52, (uint8_t)218, (uint8_t)85, (uint8_t)54, (uint8_t)243, (uint8_t)71, (uint8_t)119, (uint8_t)247, (uint8_t)41, (uint8_t)195, (uint8_t)215, (uint8_t)113, (uint8_t)190, (uint8_t)12, (uint8_t)17, (uint8_t)161, (uint8_t)162, (uint8_t)145, (uint8_t)56, (uint8_t)50, (uint8_t)6, (uint8_t)95, (uint8_t)199, (uint8_t)66, (uint8_t)163, (uint8_t)116, (uint8_t)26, (uint8_t)38, (uint8_t)157, (uint8_t)63, (uint8_t)67, (uint8_t)142, (uint8_t)170, (uint8_t)245, (uint8_t)129, (uint8_t)2, (uint8_t)147, (uint8_t)82, (uint8_t)142, (uint8_t)202, (uint8_t)219, (uint8_t)129, (uint8_t)226, (uint8_t)90, (uint8_t)204, (uint8_t)121, (uint8_t)196, (uint8_t)23, (uint8_t)134, (uint8_t)59, (uint8_t)192, (uint8_t)152, (uint8_t)116, (uint8_t)205, (uint8_t)32, (uint8_t)154, (uint8_t)239, (uint8_t)204, (uint8_t)29, (uint8_t)247, (uint8_t)124, (uint8_t)111, (uint8_t)185, (uint8_t)143, (uint8_t)173, (uint8_t)146, (uint8_t)215, (uint8_t)7, (uint8_t)109, (uint8_t)173, (uint8_t)105, (uint8_t)150, (uint8_t)90, (uint8_t)1, (uint8_t)191, (uint8_t)19, (uint8_t)178, (uint8_t)34, (uint8_t)246, (uint8_t)190, (uint8_t)210, (uint8_t)230, (uint8_t)156, (uint8_t)94, (uint8_t)101, (uint8_t)97, (uint8_t)67, (uint8_t)130, (uint8_t)238, (uint8_t)85, (uint8_t)73, (uint8_t)176, (uint8_t)79};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)82, (uint8_t)108, (uint8_t)230, (uint8_t)66, (uint8_t)91, (uint8_t)90, (uint8_t)141, (uint8_t)195, (uint8_t)38, (uint8_t)241, (uint8_t)20, (uint8_t)86, (uint8_t)252, (uint8_t)233, (uint8_t)179, (uint8_t)76, (uint8_t)34, (uint8_t)97, (uint8_t)132, (uint8_t)117, (uint8_t)99, (uint8_t)96, (uint8_t)214, (uint8_t)208, (uint8_t)242, (uint8_t)240, (uint8_t)240, (uint8_t)89, (uint8_t)29, (uint8_t)209, (uint8_t)116, (uint8_t)30, (uint8_t)141, (uint8_t)207, (uint8_t)176, (uint8_t)76, (uint8_t)249, (uint8_t)115, (uint8_t)80, (uint8_t)234, (uint8_t)138, (uint8_t)98, (uint8_t)78, (uint8_t)133, (uint8_t)109, (uint8_t)165, (uint8_t)253, (uint8_t)246, (uint8_t)205, (uint8_t)126, (uint8_t)185, (uint8_t)156, (uint8_t)201, (uint8_t)137, (uint8_t)124, (uint8_t)224, (uint8_t)226, (uint8_t)252, (uint8_t)35, (uint8_t)232, (uint8_t)62, (uint8_t)73, (uint8_t)42, (uint8_t)244, (uint8_t)111, (uint8_t)80, (uint8_t)179, (uint8_t)200, (uint8_t)210, (uint8_t)222, (uint8_t)21, (uint8_t)160, (uint8_t)247, (uint8_t)176, (uint8_t)17, (uint8_t)24, (uint8_t)214, (uint8_t)111, (uint8_t)78, (uint8_t)34, (uint8_t)210, (uint8_t)148, (uint8_t)241, (uint8_t)10, (uint8_t)110, (uint8_t)47, (uint8_t)180, (uint8_t)254, (uint8_t)158, (uint8_t)36, (uint8_t)141, (uint8_t)3, (uint8_t)183, (uint8_t)165, (uint8_t)100, (uint8_t)64, (uint8_t)83, (uint8_t)239, (uint8_t)12, (uint8_t)203, (uint8_t)128, (uint8_t)1, (uint8_t)55, (uint8_t)41, (uint8_t)91, (uint8_t)104, (uint8_t)35, (uint8_t)239, (uint8_t)195, (uint8_t)235, (uint8_t)171, (uint8_t)60, (uint8_t)15, (uint8_t)133, (uint8_t)189, (uint8_t)145, (uint8_t)221, (uint8_t)26, (uint8_t)145, (uint8_t)82};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t) -18627, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)431280461L, PH.base.pack) ;
        p143_press_abs_SET((float) -8.958351E37F, PH.base.pack) ;
        p143_press_diff_SET((float) -3.1260177E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_est_capabilities_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p144_lon_SET((int32_t) -139403957, PH.base.pack) ;
        {
            float attitude_q[] =  {-1.4574601E38F, 1.6794376E38F, -1.4702749E38F, 4.9766583E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)5308689589440451670L, PH.base.pack) ;
        {
            float position_cov[] =  {2.983449E38F, -2.983097E38F, 2.932498E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -73795928, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)1650628327616744111L, PH.base.pack) ;
        {
            float vel[] =  {1.3192758E37F, 5.5901617E37F, 1.0300591E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)1.3598978E38F, PH.base.pack) ;
        {
            float acc[] =  {9.587057E37F, -1.906359E38F, 1.8313742E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {1.0222001E36F, -1.5232758E38F, 1.4080457E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        {
            float pos_variance[] =  {1.2292276E38F, -1.8710002E38F, 8.649498E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)1619038403660305502L, PH.base.pack) ;
        p146_x_acc_SET((float)2.9966203E37F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.561504E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -2.2457412E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {2.4377953E38F, -8.828426E37F, -1.3793106E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float)5.119441E37F, PH.base.pack) ;
        p146_z_pos_SET((float)7.652396E37F, PH.base.pack) ;
        p146_z_vel_SET((float) -2.95608E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.1000786E38F, PH.base.pack) ;
        p146_y_pos_SET((float)4.8843186E37F, PH.base.pack) ;
        p146_y_acc_SET((float) -3.0109687E38F, PH.base.pack) ;
        p146_z_acc_SET((float)1.0283785E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -1.8314758E38F, PH.base.pack) ;
        {
            float q[] =  {2.2482755E38F, -1.7719528E38F, 5.704137E37F, 2.8615768E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float) -6.220076E37F, PH.base.pack) ;
        p146_x_vel_SET((float)1.8530441E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_temperature_SET((int16_t)(int16_t)32078, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)691693029, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)1, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -11409, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)47917, (uint16_t)53509, (uint16_t)19400, (uint16_t)50274, (uint16_t)57778, (uint16_t)49310, (uint16_t)55932, (uint16_t)13781, (uint16_t)20578, (uint16_t)53645};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)2085033280, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_middleware_sw_version_SET((uint32_t)1543635465L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)226, (uint8_t)130, (uint8_t)149, (uint8_t)155, (uint8_t)246, (uint8_t)137, (uint8_t)145, (uint8_t)255};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)241, (uint8_t)245, (uint8_t)30, (uint8_t)184, (uint8_t)236, (uint8_t)161, (uint8_t)108, (uint8_t)192, (uint8_t)165, (uint8_t)234, (uint8_t)214, (uint8_t)241, (uint8_t)79, (uint8_t)116, (uint8_t)18, (uint8_t)167, (uint8_t)71, (uint8_t)171};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_flight_sw_version_SET((uint32_t)778016811L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)3844912591L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)27350, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT), PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)11647, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)176, (uint8_t)20, (uint8_t)14, (uint8_t)212, (uint8_t)21, (uint8_t)103, (uint8_t)190, (uint8_t)2};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t os_custom_version[] =  {(uint8_t)82, (uint8_t)39, (uint8_t)219, (uint8_t)200, (uint8_t)137, (uint8_t)2, (uint8_t)62, (uint8_t)243};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_os_sw_version_SET((uint32_t)1883292021L, PH.base.pack) ;
        p148_uid_SET((uint64_t)6995396170291095570L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        {
            float q[] =  {-2.684405E38F, 1.8285028E38F, -1.3970531E38F, -2.0990876E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_angle_y_SET((float)2.370314E38F, PH.base.pack) ;
        p149_size_y_SET((float)2.9565153E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_x_SET((float)1.0808698E37F, &PH) ;
        p149_distance_SET((float)6.4034995E37F, PH.base.pack) ;
        p149_angle_x_SET((float)3.1746248E37F, PH.base.pack) ;
        p149_z_SET((float)5.327722E37F, &PH) ;
        p149_size_x_SET((float) -2.1226349E38F, PH.base.pack) ;
        p149_y_SET((float)1.4066739E38F, &PH) ;
        p149_time_usec_SET((uint64_t)4407418101831889848L, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)86, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AQ_TELEMETRY_F_150(), &PH);
        p150_value3_SET((float) -3.3682697E38F, PH.base.pack) ;
        p150_value6_SET((float) -1.3713405E37F, PH.base.pack) ;
        p150_value17_SET((float)6.2901817E37F, PH.base.pack) ;
        p150_value1_SET((float) -3.2472858E37F, PH.base.pack) ;
        p150_value13_SET((float) -5.1004263E35F, PH.base.pack) ;
        p150_value7_SET((float) -3.2967233E38F, PH.base.pack) ;
        p150_value19_SET((float) -2.7692994E38F, PH.base.pack) ;
        p150_value9_SET((float)5.4435244E37F, PH.base.pack) ;
        p150_value4_SET((float)1.0884099E38F, PH.base.pack) ;
        p150_value15_SET((float) -3.0737434E38F, PH.base.pack) ;
        p150_value16_SET((float)2.8570613E38F, PH.base.pack) ;
        p150_Index_SET((uint16_t)(uint16_t)17866, PH.base.pack) ;
        p150_value18_SET((float)1.4266858E38F, PH.base.pack) ;
        p150_value11_SET((float)1.2369319E38F, PH.base.pack) ;
        p150_value2_SET((float)1.3191691E38F, PH.base.pack) ;
        p150_value14_SET((float)2.4136445E38F, PH.base.pack) ;
        p150_value20_SET((float)2.0093114E38F, PH.base.pack) ;
        p150_value12_SET((float)1.2104812E38F, PH.base.pack) ;
        p150_value8_SET((float)7.606547E37F, PH.base.pack) ;
        p150_value5_SET((float)2.6295747E38F, PH.base.pack) ;
        p150_value10_SET((float)3.919579E37F, PH.base.pack) ;
        c_CommunicationChannel_on_AQ_TELEMETRY_F_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AQ_ESC_TELEMETRY_152(), &PH);
        p152_num_motors_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        {
            uint8_t escid[] =  {(uint8_t)201, (uint8_t)81, (uint8_t)229, (uint8_t)56};
            p152_escid_SET(&escid, 0, PH.base.pack) ;
        }
        p152_time_boot_ms_SET((uint32_t)3454567170L, PH.base.pack) ;
        {
            uint32_t data1[] =  {3767355010L, 451454289L, 2184363697L, 2946792672L};
            p152_data1_SET(&data1, 0, PH.base.pack) ;
        }
        p152_num_in_seq_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        {
            uint32_t data0[] =  {1157178701L, 3132872100L, 412329233L, 1176023367L};
            p152_data0_SET(&data0, 0, PH.base.pack) ;
        }
        p152_seq_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        {
            uint8_t data_version[] =  {(uint8_t)198, (uint8_t)250, (uint8_t)169, (uint8_t)37};
            p152_data_version_SET(&data_version, 0, PH.base.pack) ;
        }
        {
            uint16_t status_age[] =  {(uint16_t)47998, (uint16_t)19648, (uint16_t)64121, (uint16_t)2713};
            p152_status_age_SET(&status_age, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_AQ_ESC_TELEMETRY_152(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_tas_ratio_SET((float)5.174929E37F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)7550178073896723329L, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.0657504E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.0069607E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.0243843E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.3820996E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.6114018E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.5459157E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.955588E38F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE), PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float)1.1534208E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -1.3587766E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)3.2007431E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)1.6657603E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -6.880156E36F, PH.base.pack) ;
        p231_wind_alt_SET((float) -2.7393917E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)3206402163156425535L, PH.base.pack) ;
        p231_wind_x_SET((float) -1.0704133E38F, PH.base.pack) ;
        p231_var_vert_SET((float) -5.448522E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_lon_SET((int32_t)191165136, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -3.0493994E37F, PH.base.pack) ;
        p232_vd_SET((float)2.1040686E38F, PH.base.pack) ;
        p232_hdop_SET((float) -8.515767E37F, PH.base.pack) ;
        p232_ve_SET((float)2.4692582E38F, PH.base.pack) ;
        p232_alt_SET((float) -2.988146E38F, PH.base.pack) ;
        p232_vn_SET((float)3.1321713E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.0971364E38F, PH.base.pack) ;
        p232_vdop_SET((float) -3.127378E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1729014026324559933L, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -2.1962381E38F, PH.base.pack) ;
        p232_lat_SET((int32_t)1225144050, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2489352422L, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY), PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)55606, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)69, (uint8_t)26, (uint8_t)139, (uint8_t)217, (uint8_t)60, (uint8_t)146, (uint8_t)196, (uint8_t)55, (uint8_t)186, (uint8_t)209, (uint8_t)250, (uint8_t)70, (uint8_t)22, (uint8_t)203, (uint8_t)86, (uint8_t)52, (uint8_t)42, (uint8_t)45, (uint8_t)65, (uint8_t)46, (uint8_t)125, (uint8_t)86, (uint8_t)118, (uint8_t)23, (uint8_t)114, (uint8_t)12, (uint8_t)41, (uint8_t)160, (uint8_t)118, (uint8_t)173, (uint8_t)160, (uint8_t)35, (uint8_t)239, (uint8_t)250, (uint8_t)6, (uint8_t)27, (uint8_t)184, (uint8_t)37, (uint8_t)154, (uint8_t)104, (uint8_t)228, (uint8_t)115, (uint8_t)101, (uint8_t)58, (uint8_t)155, (uint8_t)235, (uint8_t)11, (uint8_t)170, (uint8_t)185, (uint8_t)60, (uint8_t)86, (uint8_t)97, (uint8_t)89, (uint8_t)118, (uint8_t)83, (uint8_t)86, (uint8_t)27, (uint8_t)12, (uint8_t)76, (uint8_t)190, (uint8_t)254, (uint8_t)209, (uint8_t)62, (uint8_t)115, (uint8_t)185, (uint8_t)68, (uint8_t)245, (uint8_t)36, (uint8_t)34, (uint8_t)44, (uint8_t)210, (uint8_t)168, (uint8_t)67, (uint8_t)248, (uint8_t)134, (uint8_t)239, (uint8_t)188, (uint8_t)35, (uint8_t)88, (uint8_t)216, (uint8_t)184, (uint8_t)131, (uint8_t)210, (uint8_t)249, (uint8_t)21, (uint8_t)165, (uint8_t)4, (uint8_t)246, (uint8_t)45, (uint8_t)98, (uint8_t)91, (uint8_t)122, (uint8_t)49, (uint8_t)27, (uint8_t)75, (uint8_t)51, (uint8_t)200, (uint8_t)120, (uint8_t)58, (uint8_t)160, (uint8_t)169, (uint8_t)209, (uint8_t)67, (uint8_t)94, (uint8_t)131, (uint8_t)166, (uint8_t)86, (uint8_t)52, (uint8_t)125, (uint8_t)79, (uint8_t)223, (uint8_t)175, (uint8_t)156, (uint8_t)116, (uint8_t)66, (uint8_t)82, (uint8_t)87, (uint8_t)243, (uint8_t)120, (uint8_t)12, (uint8_t)107, (uint8_t)175, (uint8_t)224, (uint8_t)190, (uint8_t)8, (uint8_t)187, (uint8_t)122, (uint8_t)158, (uint8_t)155, (uint8_t)123, (uint8_t)19, (uint8_t)22, (uint8_t)45, (uint8_t)149, (uint8_t)10, (uint8_t)126, (uint8_t)72, (uint8_t)210, (uint8_t)195, (uint8_t)106, (uint8_t)194, (uint8_t)197, (uint8_t)95, (uint8_t)238, (uint8_t)8, (uint8_t)139, (uint8_t)232, (uint8_t)201, (uint8_t)198, (uint8_t)25, (uint8_t)37, (uint8_t)204, (uint8_t)207, (uint8_t)76, (uint8_t)214, (uint8_t)244, (uint8_t)254, (uint8_t)177, (uint8_t)79, (uint8_t)82, (uint8_t)147, (uint8_t)0, (uint8_t)238, (uint8_t)150, (uint8_t)14, (uint8_t)159, (uint8_t)13, (uint8_t)21, (uint8_t)147, (uint8_t)155, (uint8_t)65, (uint8_t)115, (uint8_t)103, (uint8_t)245, (uint8_t)251, (uint8_t)189, (uint8_t)26, (uint8_t)225, (uint8_t)157, (uint8_t)115};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_failsafe_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p234_longitude_SET((int32_t) -451132756, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -3627, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -31653, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)91, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)26800, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -113, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -40, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -928, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED), PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)62031, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -26729, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)2473184599L, PH.base.pack) ;
        p234_latitude_SET((int32_t) -983246324, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)9458, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)97, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float)7.6471864E37F, PH.base.pack) ;
        p241_vibration_z_SET((float) -1.2845564E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)254089256L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)7351982156701305515L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.9914823E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)385573319L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)153961006L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_latitude_SET((int32_t)214673482, PH.base.pack) ;
        p242_y_SET((float) -2.868408E38F, PH.base.pack) ;
        p242_approach_x_SET((float)2.6417262E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -6.116728E37F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.8481646E38F, PH.base.pack) ;
        p242_z_SET((float)3.3999993E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t)1157017793, PH.base.pack) ;
        {
            float q[] =  {-1.7888846E38F, 1.3765528E37F, -3.0786883E38F, -1.7288158E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_altitude_SET((int32_t)2029599948, PH.base.pack) ;
        p242_x_SET((float) -1.4312479E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)4757372918215412978L, &PH) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_altitude_SET((int32_t)1322495407, PH.base.pack) ;
        p243_approach_x_SET((float) -2.84697E38F, PH.base.pack) ;
        {
            float q[] =  {-2.416283E38F, 2.214733E38F, -6.241874E37F, -5.0301157E37F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_z_SET((float) -2.7871371E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t)2114027292, PH.base.pack) ;
        p243_x_SET((float)2.5696517E38F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)1474354256405988074L, &PH) ;
        p243_longitude_SET((int32_t) -1096200481, PH.base.pack) ;
        p243_z_SET((float)3.2539167E38F, PH.base.pack) ;
        p243_y_SET((float) -2.5398546E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -6.7840457E37F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)9934, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -523513121, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_hor_velocity_SET((uint16_t)(uint16_t)33697, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)59246, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)64531, PH.base.pack) ;
        p246_lon_SET((int32_t) -1046003890, PH.base.pack) ;
        p246_lat_SET((int32_t)116460501, PH.base.pack) ;
        {
            char16_t* callsign = u"suedllnxb";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_altitude_SET((int32_t)1473517028, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)440944949L, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -26402, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED), PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.3830025E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_id_SET((uint32_t)3243930578L, PH.base.pack) ;
        p247_threat_level_SET((e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH |
                               e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW), PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -2.1499948E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)2.483384E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)40838, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)50, (uint8_t)34, (uint8_t)119, (uint8_t)232, (uint8_t)195, (uint8_t)117, (uint8_t)137, (uint8_t)237, (uint8_t)35, (uint8_t)184, (uint8_t)81, (uint8_t)202, (uint8_t)182, (uint8_t)175, (uint8_t)185, (uint8_t)196, (uint8_t)21, (uint8_t)241, (uint8_t)68, (uint8_t)29, (uint8_t)42, (uint8_t)89, (uint8_t)18, (uint8_t)59, (uint8_t)220, (uint8_t)113, (uint8_t)243, (uint8_t)254, (uint8_t)106, (uint8_t)49, (uint8_t)144, (uint8_t)119, (uint8_t)16, (uint8_t)209, (uint8_t)231, (uint8_t)145, (uint8_t)114, (uint8_t)157, (uint8_t)57, (uint8_t)248, (uint8_t)127, (uint8_t)2, (uint8_t)247, (uint8_t)81, (uint8_t)154, (uint8_t)65, (uint8_t)144, (uint8_t)138, (uint8_t)51, (uint8_t)34, (uint8_t)51, (uint8_t)171, (uint8_t)121, (uint8_t)159, (uint8_t)67, (uint8_t)104, (uint8_t)251, (uint8_t)1, (uint8_t)74, (uint8_t)161, (uint8_t)131, (uint8_t)145, (uint8_t)99, (uint8_t)10, (uint8_t)160, (uint8_t)101, (uint8_t)185, (uint8_t)59, (uint8_t)42, (uint8_t)107, (uint8_t)231, (uint8_t)125, (uint8_t)2, (uint8_t)208, (uint8_t)73, (uint8_t)35, (uint8_t)192, (uint8_t)151, (uint8_t)135, (uint8_t)157, (uint8_t)165, (uint8_t)46, (uint8_t)164, (uint8_t)150, (uint8_t)240, (uint8_t)0, (uint8_t)142, (uint8_t)5, (uint8_t)144, (uint8_t)200, (uint8_t)110, (uint8_t)2, (uint8_t)19, (uint8_t)99, (uint8_t)23, (uint8_t)240, (uint8_t)198, (uint8_t)236, (uint8_t)103, (uint8_t)204, (uint8_t)0, (uint8_t)219, (uint8_t)203, (uint8_t)143, (uint8_t)117, (uint8_t)208, (uint8_t)205, (uint8_t)29, (uint8_t)220, (uint8_t)14, (uint8_t)142, (uint8_t)74, (uint8_t)53, (uint8_t)206, (uint8_t)183, (uint8_t)197, (uint8_t)166, (uint8_t)109, (uint8_t)4, (uint8_t)138, (uint8_t)28, (uint8_t)213, (uint8_t)26, (uint8_t)107, (uint8_t)68, (uint8_t)196, (uint8_t)53, (uint8_t)151, (uint8_t)91, (uint8_t)152, (uint8_t)183, (uint8_t)160, (uint8_t)170, (uint8_t)7, (uint8_t)224, (uint8_t)27, (uint8_t)90, (uint8_t)36, (uint8_t)132, (uint8_t)246, (uint8_t)131, (uint8_t)129, (uint8_t)11, (uint8_t)85, (uint8_t)50, (uint8_t)234, (uint8_t)223, (uint8_t)199, (uint8_t)125, (uint8_t)246, (uint8_t)110, (uint8_t)26, (uint8_t)157, (uint8_t)28, (uint8_t)110, (uint8_t)180, (uint8_t)36, (uint8_t)249, (uint8_t)11, (uint8_t)75, (uint8_t)76, (uint8_t)127, (uint8_t)14, (uint8_t)59, (uint8_t)40, (uint8_t)182, (uint8_t)192, (uint8_t)134, (uint8_t)185, (uint8_t)198, (uint8_t)156, (uint8_t)199, (uint8_t)42, (uint8_t)123, (uint8_t)168, (uint8_t)117, (uint8_t)32, (uint8_t)131, (uint8_t)231, (uint8_t)103, (uint8_t)242, (uint8_t)223, (uint8_t)181, (uint8_t)152, (uint8_t)243, (uint8_t)166, (uint8_t)82, (uint8_t)238, (uint8_t)21, (uint8_t)246, (uint8_t)224, (uint8_t)177, (uint8_t)19, (uint8_t)242, (uint8_t)52, (uint8_t)40, (uint8_t)123, (uint8_t)99, (uint8_t)88, (uint8_t)75, (uint8_t)44, (uint8_t)63, (uint8_t)3, (uint8_t)0, (uint8_t)79, (uint8_t)77, (uint8_t)165, (uint8_t)131, (uint8_t)72, (uint8_t)110, (uint8_t)160, (uint8_t)67, (uint8_t)227, (uint8_t)141, (uint8_t)48, (uint8_t)66, (uint8_t)108, (uint8_t)50, (uint8_t)13, (uint8_t)243, (uint8_t)154, (uint8_t)236, (uint8_t)130, (uint8_t)88, (uint8_t)84, (uint8_t)61, (uint8_t)171, (uint8_t)23, (uint8_t)169, (uint8_t)181, (uint8_t)22, (uint8_t)218, (uint8_t)42, (uint8_t)139, (uint8_t)9, (uint8_t)64, (uint8_t)131, (uint8_t)24, (uint8_t)203, (uint8_t)191, (uint8_t)165, (uint8_t)130, (uint8_t)183, (uint8_t)169, (uint8_t)47, (uint8_t)83, (uint8_t)108, (uint8_t)228, (uint8_t)39};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)46834, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -7, (int8_t)10, (int8_t)51, (int8_t)125, (int8_t)2, (int8_t) -127, (int8_t) -69, (int8_t)120, (int8_t) -63, (int8_t)125, (int8_t)61, (int8_t)48, (int8_t)70, (int8_t) -54, (int8_t) -9, (int8_t) -20, (int8_t) -122, (int8_t)125, (int8_t) -85, (int8_t)28, (int8_t) -113, (int8_t)114, (int8_t) -98, (int8_t) -66, (int8_t) -34, (int8_t) -103, (int8_t)9, (int8_t) -34, (int8_t)121, (int8_t) -6, (int8_t) -6, (int8_t)5};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        {
            char16_t* name = u"osd";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float)6.922714E37F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)8124758947596593065L, PH.base.pack) ;
        p250_y_SET((float)2.0670163E38F, PH.base.pack) ;
        p250_x_SET((float) -2.477569E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -4.284225E37F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)949870768L, PH.base.pack) ;
        {
            char16_t* name = u"ncsrwmsvqc";
            p251_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"Qgz";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t)1540569665, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)1556522035L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"hegUimxpyqbustvtHaxqraidyqtzqhfgVyr";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ERROR, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float) -3.8866246E37F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)3935043085L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)147, (uint8_t)126, (uint8_t)229, (uint8_t)190, (uint8_t)73, (uint8_t)88, (uint8_t)85, (uint8_t)160, (uint8_t)10, (uint8_t)154, (uint8_t)11, (uint8_t)109, (uint8_t)34, (uint8_t)99, (uint8_t)228, (uint8_t)57, (uint8_t)90, (uint8_t)201, (uint8_t)136, (uint8_t)247, (uint8_t)231, (uint8_t)207, (uint8_t)234, (uint8_t)82, (uint8_t)45, (uint8_t)203, (uint8_t)209, (uint8_t)180, (uint8_t)142, (uint8_t)247, (uint8_t)34, (uint8_t)246};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)1082966711951180144L, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)407775262L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)1172484084L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p258_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        {
            char16_t* tune = u"gqjhnotjjuwptaggokljczv";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        {
            char16_t* cam_definition_uri = u"ypDqqavolzvcL";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_sensor_size_h_SET((float) -2.3514785E38F, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)331889379L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)60732, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)13234, PH.base.pack) ;
        p259_sensor_size_v_SET((float)1.0827285E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)106, (uint8_t)178, (uint8_t)81, (uint8_t)57, (uint8_t)31, (uint8_t)102, (uint8_t)157, (uint8_t)124, (uint8_t)57, (uint8_t)69, (uint8_t)102, (uint8_t)242, (uint8_t)38, (uint8_t)40, (uint8_t)17, (uint8_t)66, (uint8_t)22, (uint8_t)10, (uint8_t)173, (uint8_t)9, (uint8_t)94, (uint8_t)104, (uint8_t)6, (uint8_t)54, (uint8_t)228, (uint8_t)75, (uint8_t)224, (uint8_t)26, (uint8_t)119, (uint8_t)60, (uint8_t)246, (uint8_t)202};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        {
            uint8_t vendor_name[] =  {(uint8_t)114, (uint8_t)2, (uint8_t)117, (uint8_t)192, (uint8_t)134, (uint8_t)159, (uint8_t)18, (uint8_t)84, (uint8_t)66, (uint8_t)223, (uint8_t)36, (uint8_t)5, (uint8_t)236, (uint8_t)8, (uint8_t)247, (uint8_t)171, (uint8_t)191, (uint8_t)3, (uint8_t)53, (uint8_t)81, (uint8_t)99, (uint8_t)153, (uint8_t)66, (uint8_t)174, (uint8_t)108, (uint8_t)209, (uint8_t)43, (uint8_t)49, (uint8_t)197, (uint8_t)131, (uint8_t)170, (uint8_t)223};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)18773, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)2171741535L, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p259_focal_length_SET((float)4.610976E37F, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE), PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET((e_CAMERA_MODE_CAMERA_MODE_IMAGE), PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3505527786L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_total_capacity_SET((float)2.3873246E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2631759373L, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.3079482E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -1.0862719E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p261_available_capacity_SET((float)5.9327037E37F, PH.base.pack) ;
        p261_write_speed_SET((float)1.3225602E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_video_status_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p262_image_interval_SET((float)1.1590006E38F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)226072064L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p262_available_capacity_SET((float)9.443459E37F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3519907365L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)1953461888702783785L, PH.base.pack) ;
        p263_alt_SET((int32_t) -151725461, PH.base.pack) ;
        p263_lon_SET((int32_t)1025196, PH.base.pack) ;
        p263_image_index_SET((int32_t)1104771512, PH.base.pack) ;
        {
            char16_t* file_url = u"fshkowultvKtnzjtconwTppgexrwkaNpdYpakqtssicvnsavUjwvddwbuzrigqimgfjnsipvgmujobVnxbaqryxvcwpunQrkGhsykrfoYvrNocvakRtIvsbzbraiuyiipzYbrdzdAolwhwzKlvqexnddjmvdhupjzhlvwgNzcvpvrcvibwjkgxwzyveUsemutiziddrupwy";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_relative_alt_SET((int32_t) -1047988233, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -26, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)3049128121L, PH.base.pack) ;
        {
            float q[] =  {1.139643E38F, 1.5610668E38F, 1.2220242E38F, 7.7502114E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_lat_SET((int32_t)239973729, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)3217478618740819508L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)336319954L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)5909017516484199504L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)8957923858965941624L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float)4.2880675E37F, PH.base.pack) ;
        p265_roll_SET((float) -1.1833299E38F, PH.base.pack) ;
        p265_yaw_SET((float)2.56425E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)2088930417L, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_length_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)10595, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)87, (uint8_t)20, (uint8_t)78, (uint8_t)177, (uint8_t)131, (uint8_t)220, (uint8_t)162, (uint8_t)29, (uint8_t)114, (uint8_t)124, (uint8_t)237, (uint8_t)3, (uint8_t)95, (uint8_t)109, (uint8_t)142, (uint8_t)140, (uint8_t)128, (uint8_t)176, (uint8_t)147, (uint8_t)30, (uint8_t)195, (uint8_t)202, (uint8_t)232, (uint8_t)153, (uint8_t)182, (uint8_t)81, (uint8_t)14, (uint8_t)174, (uint8_t)248, (uint8_t)175, (uint8_t)83, (uint8_t)197, (uint8_t)71, (uint8_t)20, (uint8_t)29, (uint8_t)187, (uint8_t)21, (uint8_t)133, (uint8_t)172, (uint8_t)219, (uint8_t)200, (uint8_t)225, (uint8_t)20, (uint8_t)39, (uint8_t)164, (uint8_t)86, (uint8_t)135, (uint8_t)25, (uint8_t)195, (uint8_t)4, (uint8_t)51, (uint8_t)220, (uint8_t)141, (uint8_t)214, (uint8_t)245, (uint8_t)128, (uint8_t)50, (uint8_t)57, (uint8_t)80, (uint8_t)138, (uint8_t)184, (uint8_t)144, (uint8_t)126, (uint8_t)117, (uint8_t)95, (uint8_t)228, (uint8_t)80, (uint8_t)136, (uint8_t)98, (uint8_t)134, (uint8_t)111, (uint8_t)227, (uint8_t)63, (uint8_t)184, (uint8_t)40, (uint8_t)29, (uint8_t)76, (uint8_t)17, (uint8_t)214, (uint8_t)200, (uint8_t)92, (uint8_t)142, (uint8_t)11, (uint8_t)27, (uint8_t)216, (uint8_t)88, (uint8_t)138, (uint8_t)8, (uint8_t)233, (uint8_t)54, (uint8_t)88, (uint8_t)239, (uint8_t)247, (uint8_t)138, (uint8_t)152, (uint8_t)226, (uint8_t)228, (uint8_t)11, (uint8_t)230, (uint8_t)151, (uint8_t)245, (uint8_t)89, (uint8_t)81, (uint8_t)188, (uint8_t)92, (uint8_t)232, (uint8_t)118, (uint8_t)11, (uint8_t)51, (uint8_t)209, (uint8_t)205, (uint8_t)89, (uint8_t)150, (uint8_t)253, (uint8_t)66, (uint8_t)238, (uint8_t)81, (uint8_t)5, (uint8_t)180, (uint8_t)229, (uint8_t)215, (uint8_t)66, (uint8_t)131, (uint8_t)1, (uint8_t)129, (uint8_t)204, (uint8_t)109, (uint8_t)17, (uint8_t)232, (uint8_t)181, (uint8_t)19, (uint8_t)65, (uint8_t)211, (uint8_t)67, (uint8_t)46, (uint8_t)248, (uint8_t)125, (uint8_t)154, (uint8_t)10, (uint8_t)230, (uint8_t)130, (uint8_t)168, (uint8_t)56, (uint8_t)189, (uint8_t)93, (uint8_t)42, (uint8_t)232, (uint8_t)234, (uint8_t)179, (uint8_t)58, (uint8_t)136, (uint8_t)29, (uint8_t)204, (uint8_t)57, (uint8_t)220, (uint8_t)95, (uint8_t)241, (uint8_t)118, (uint8_t)157, (uint8_t)156, (uint8_t)123, (uint8_t)40, (uint8_t)109, (uint8_t)179, (uint8_t)172, (uint8_t)38, (uint8_t)253, (uint8_t)35, (uint8_t)40, (uint8_t)65, (uint8_t)210, (uint8_t)200, (uint8_t)43, (uint8_t)176, (uint8_t)234, (uint8_t)7, (uint8_t)185, (uint8_t)0, (uint8_t)207, (uint8_t)206, (uint8_t)111, (uint8_t)103, (uint8_t)93, (uint8_t)243, (uint8_t)83, (uint8_t)115, (uint8_t)77, (uint8_t)10, (uint8_t)108, (uint8_t)228, (uint8_t)179, (uint8_t)242, (uint8_t)14, (uint8_t)254, (uint8_t)120, (uint8_t)243, (uint8_t)78, (uint8_t)1, (uint8_t)98, (uint8_t)69, (uint8_t)9, (uint8_t)147, (uint8_t)143, (uint8_t)115, (uint8_t)70, (uint8_t)43, (uint8_t)246, (uint8_t)41, (uint8_t)216, (uint8_t)68, (uint8_t)174, (uint8_t)133, (uint8_t)254, (uint8_t)135, (uint8_t)173, (uint8_t)100, (uint8_t)98, (uint8_t)239, (uint8_t)1, (uint8_t)23, (uint8_t)13, (uint8_t)72, (uint8_t)91, (uint8_t)53, (uint8_t)48, (uint8_t)188, (uint8_t)200, (uint8_t)76, (uint8_t)245, (uint8_t)234, (uint8_t)244, (uint8_t)253, (uint8_t)242, (uint8_t)252, (uint8_t)93, (uint8_t)86, (uint8_t)224, (uint8_t)211, (uint8_t)71, (uint8_t)33, (uint8_t)113, (uint8_t)92, (uint8_t)229, (uint8_t)43, (uint8_t)84, (uint8_t)127, (uint8_t)48, (uint8_t)105, (uint8_t)224};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)245, (uint8_t)159, (uint8_t)61, (uint8_t)197, (uint8_t)197, (uint8_t)228, (uint8_t)25, (uint8_t)101, (uint8_t)82, (uint8_t)158, (uint8_t)251, (uint8_t)179, (uint8_t)189, (uint8_t)73, (uint8_t)16, (uint8_t)181, (uint8_t)3, (uint8_t)56, (uint8_t)130, (uint8_t)237, (uint8_t)138, (uint8_t)66, (uint8_t)69, (uint8_t)238, (uint8_t)122, (uint8_t)21, (uint8_t)115, (uint8_t)227, (uint8_t)125, (uint8_t)14, (uint8_t)69, (uint8_t)182, (uint8_t)106, (uint8_t)98, (uint8_t)51, (uint8_t)68, (uint8_t)91, (uint8_t)164, (uint8_t)45, (uint8_t)229, (uint8_t)209, (uint8_t)6, (uint8_t)225, (uint8_t)101, (uint8_t)60, (uint8_t)113, (uint8_t)141, (uint8_t)129, (uint8_t)32, (uint8_t)237, (uint8_t)63, (uint8_t)178, (uint8_t)9, (uint8_t)140, (uint8_t)163, (uint8_t)235, (uint8_t)108, (uint8_t)252, (uint8_t)146, (uint8_t)188, (uint8_t)80, (uint8_t)238, (uint8_t)93, (uint8_t)146, (uint8_t)35, (uint8_t)209, (uint8_t)5, (uint8_t)96, (uint8_t)55, (uint8_t)103, (uint8_t)19, (uint8_t)157, (uint8_t)31, (uint8_t)142, (uint8_t)130, (uint8_t)41, (uint8_t)13, (uint8_t)107, (uint8_t)88, (uint8_t)100, (uint8_t)68, (uint8_t)11, (uint8_t)53, (uint8_t)121, (uint8_t)167, (uint8_t)56, (uint8_t)113, (uint8_t)220, (uint8_t)181, (uint8_t)204, (uint8_t)131, (uint8_t)255, (uint8_t)203, (uint8_t)147, (uint8_t)101, (uint8_t)141, (uint8_t)140, (uint8_t)205, (uint8_t)218, (uint8_t)55, (uint8_t)93, (uint8_t)107, (uint8_t)224, (uint8_t)168, (uint8_t)246, (uint8_t)83, (uint8_t)79, (uint8_t)17, (uint8_t)171, (uint8_t)33, (uint8_t)177, (uint8_t)79, (uint8_t)103, (uint8_t)192, (uint8_t)187, (uint8_t)136, (uint8_t)49, (uint8_t)143, (uint8_t)90, (uint8_t)5, (uint8_t)102, (uint8_t)168, (uint8_t)203, (uint8_t)178, (uint8_t)213, (uint8_t)40, (uint8_t)62, (uint8_t)252, (uint8_t)130, (uint8_t)10, (uint8_t)115, (uint8_t)93, (uint8_t)6, (uint8_t)193, (uint8_t)7, (uint8_t)109, (uint8_t)73, (uint8_t)228, (uint8_t)102, (uint8_t)219, (uint8_t)245, (uint8_t)95, (uint8_t)148, (uint8_t)86, (uint8_t)27, (uint8_t)251, (uint8_t)79, (uint8_t)186, (uint8_t)48, (uint8_t)95, (uint8_t)91, (uint8_t)23, (uint8_t)248, (uint8_t)129, (uint8_t)23, (uint8_t)70, (uint8_t)227, (uint8_t)221, (uint8_t)201, (uint8_t)4, (uint8_t)85, (uint8_t)195, (uint8_t)11, (uint8_t)251, (uint8_t)19, (uint8_t)128, (uint8_t)136, (uint8_t)244, (uint8_t)235, (uint8_t)62, (uint8_t)117, (uint8_t)56, (uint8_t)107, (uint8_t)98, (uint8_t)190, (uint8_t)36, (uint8_t)156, (uint8_t)84, (uint8_t)180, (uint8_t)46, (uint8_t)214, (uint8_t)192, (uint8_t)199, (uint8_t)65, (uint8_t)100, (uint8_t)56, (uint8_t)217, (uint8_t)158, (uint8_t)45, (uint8_t)185, (uint8_t)147, (uint8_t)130, (uint8_t)168, (uint8_t)102, (uint8_t)1, (uint8_t)220, (uint8_t)119, (uint8_t)230, (uint8_t)206, (uint8_t)205, (uint8_t)223, (uint8_t)180, (uint8_t)51, (uint8_t)233, (uint8_t)72, (uint8_t)81, (uint8_t)49, (uint8_t)24, (uint8_t)111, (uint8_t)134, (uint8_t)26, (uint8_t)108, (uint8_t)73, (uint8_t)17, (uint8_t)54, (uint8_t)234, (uint8_t)76, (uint8_t)92, (uint8_t)156, (uint8_t)39, (uint8_t)213, (uint8_t)1, (uint8_t)79, (uint8_t)173, (uint8_t)247, (uint8_t)186, (uint8_t)46, (uint8_t)133, (uint8_t)76, (uint8_t)81, (uint8_t)59, (uint8_t)232, (uint8_t)80, (uint8_t)129, (uint8_t)160, (uint8_t)214, (uint8_t)40, (uint8_t)77, (uint8_t)89, (uint8_t)247, (uint8_t)155, (uint8_t)36, (uint8_t)194, (uint8_t)247, (uint8_t)105, (uint8_t)26, (uint8_t)99, (uint8_t)57, (uint8_t)224};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)27283, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)47372, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        {
            char16_t* uri = u"ubpDibyyaLearvNScifriJvmaegdzdgcqclcdjrtmsmfim";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_bitrate_SET((uint32_t)1249559602L, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)50355, PH.base.pack) ;
        p269_framerate_SET((float)1.4655262E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)28809, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)11657, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        {
            char16_t* uri = u"sojfzkvtainwsRQomFwcggszqvzmiuqswwaqhufjgwdyPpznusxxEotwkfmjapwvDzJnuatuonkirqnNvbuczxroietwxdrgmfmmzwwtflrfixtcimstyfqwbouejmfenmydtoscemnxgioscaftyukrdtazqzyoHldswKvjoavqjksmxibqB";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_rotation_SET((uint16_t)(uint16_t)35214, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)62639, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)805819606L, PH.base.pack) ;
        p270_framerate_SET((float) -1.0650112E38F, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)51216, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"siezq";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"rqyMecqhgDUnxTnqnpDePfnSb";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)3507, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)9587, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)25910, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)242, (uint8_t)185, (uint8_t)171, (uint8_t)166, (uint8_t)240, (uint8_t)198, (uint8_t)244, (uint8_t)126};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t spec_version_hash[] =  {(uint8_t)164, (uint8_t)58, (uint8_t)96, (uint8_t)199, (uint8_t)79, (uint8_t)154, (uint8_t)233, (uint8_t)225};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_sub_mode_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)6753786523748159209L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)56639, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1280658777L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_time_usec_SET((uint64_t)2551302545452401980L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)455695534L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)2512533801L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)157, (uint8_t)162, (uint8_t)192, (uint8_t)163, (uint8_t)157, (uint8_t)177, (uint8_t)41, (uint8_t)87, (uint8_t)185, (uint8_t)165, (uint8_t)215, (uint8_t)89, (uint8_t)46, (uint8_t)13, (uint8_t)121, (uint8_t)200};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        {
            char16_t* name = u"fwgtsvmjmcwOrgqkwdaCrdjAxszaggimuggktPvhjzmIvuzyBeoztamaEkdKrdgWzhoqCsEuybs";
            p311_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"gjplcwNbvyjyfM";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t) -383, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"ixdzzqxclqlp";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)57780, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)9090, PH.base.pack) ;
        {
            char16_t* param_value = u"qyBPnVkpalotywbTiykgzoojuvcHoynfiunGtrgrpkugnetAigntBmzhiflvwozKwRbjEJjhhuIonqXgwmxhxigjdslpoiafiXHjhhsspRmdoaukpcjMdDbgdwfzp";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        {
            char16_t* param_id = u"mwygplgyq";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"GokjxnnrwvtwyJt";
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
            char16_t* param_value = u"xkuhcilnjsdkrvbuWcpjdbshqfitUdrPfzxxxsemxnjGrvrfnqxeYncadvtfmngxrjcmMbnygc";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"gxrsgvkecveri";
            p324_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)4835, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)52364, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)14070, (uint16_t)63973, (uint16_t)24813, (uint16_t)6500, (uint16_t)39927, (uint16_t)6256, (uint16_t)30108, (uint16_t)11781, (uint16_t)58938, (uint16_t)22138, (uint16_t)46200, (uint16_t)24786, (uint16_t)12770, (uint16_t)5362, (uint16_t)58826, (uint16_t)22335, (uint16_t)48679, (uint16_t)41866, (uint16_t)6469, (uint16_t)65299, (uint16_t)31415, (uint16_t)32494, (uint16_t)2394, (uint16_t)18522, (uint16_t)21107, (uint16_t)59942, (uint16_t)10674, (uint16_t)50718, (uint16_t)28940, (uint16_t)37083, (uint16_t)43123, (uint16_t)57110, (uint16_t)50185, (uint16_t)51344, (uint16_t)54, (uint16_t)6414, (uint16_t)53310, (uint16_t)61757, (uint16_t)3695, (uint16_t)24632, (uint16_t)56161, (uint16_t)2907, (uint16_t)4007, (uint16_t)52954, (uint16_t)9007, (uint16_t)61125, (uint16_t)60359, (uint16_t)62162, (uint16_t)64203, (uint16_t)62014, (uint16_t)40322, (uint16_t)7618, (uint16_t)20236, (uint16_t)27210, (uint16_t)58967, (uint16_t)29004, (uint16_t)45884, (uint16_t)599, (uint16_t)15741, (uint16_t)14926, (uint16_t)30230, (uint16_t)17048, (uint16_t)6696, (uint16_t)46269, (uint16_t)7873, (uint16_t)23352, (uint16_t)17144, (uint16_t)5402, (uint16_t)39849, (uint16_t)48204, (uint16_t)23765, (uint16_t)59991};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)8733638146357056116L, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


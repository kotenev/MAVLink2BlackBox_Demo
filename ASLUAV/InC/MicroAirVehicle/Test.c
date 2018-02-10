
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
    return  _en__S(get_bits(data, 40, 4));
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
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
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
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
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
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
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
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
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
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_TAKEOFF;
        case 18:
            return e_MAV_CMD_MAV_CMD_NAV_VTOL_LAND;
        case 19:
            return e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE;
        case 20:
            return e_MAV_CMD_MAV_CMD_NAV_DELAY;
        case 21:
            return e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE;
        case 22:
            return e_MAV_CMD_MAV_CMD_NAV_LAST;
        case 23:
            return e_MAV_CMD_MAV_CMD_CONDITION_DELAY;
        case 24:
            return e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT;
        case 25:
            return e_MAV_CMD_MAV_CMD_CONDITION_DISTANCE;
        case 26:
            return e_MAV_CMD_MAV_CMD_CONDITION_YAW;
        case 27:
            return e_MAV_CMD_MAV_CMD_CONDITION_LAST;
        case 28:
            return e_MAV_CMD_MAV_CMD_DO_SET_MODE;
        case 29:
            return e_MAV_CMD_MAV_CMD_DO_JUMP;
        case 30:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED;
        case 31:
            return e_MAV_CMD_MAV_CMD_DO_SET_HOME;
        case 32:
            return e_MAV_CMD_MAV_CMD_DO_SET_PARAMETER;
        case 33:
            return e_MAV_CMD_MAV_CMD_DO_SET_RELAY;
        case 34:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_RELAY;
        case 35:
            return e_MAV_CMD_MAV_CMD_DO_SET_SERVO;
        case 36:
            return e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO;
        case 37:
            return e_MAV_CMD_MAV_CMD_DO_FLIGHTTERMINATION;
        case 38:
            return e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE;
        case 39:
            return e_MAV_CMD_MAV_CMD_DO_LAND_START;
        case 40:
            return e_MAV_CMD_MAV_CMD_DO_RALLY_LAND;
        case 41:
            return e_MAV_CMD_MAV_CMD_DO_GO_AROUND;
        case 42:
            return e_MAV_CMD_MAV_CMD_DO_REPOSITION;
        case 43:
            return e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE;
        case 44:
            return e_MAV_CMD_MAV_CMD_DO_SET_REVERSE;
        case 45:
            return e_MAV_CMD_MAV_CMD_DO_CONTROL_VIDEO;
        case 46:
            return e_MAV_CMD_MAV_CMD_DO_SET_ROI;
        case 47:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE;
        case 48:
            return e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL;
        case 49:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE;
        case 50:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL;
        case 51:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        case 52:
            return e_MAV_CMD_MAV_CMD_DO_FENCE_ENABLE;
        case 53:
            return e_MAV_CMD_MAV_CMD_DO_PARACHUTE;
        case 54:
            return e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST;
        case 55:
            return e_MAV_CMD_MAV_CMD_DO_INVERTED_FLIGHT;
        case 56:
            return e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED;
        case 57:
            return e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        case 58:
            return e_MAV_CMD_MAV_CMD_DO_MOUNT_CONTROL_QUAT;
        case 59:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER;
        case 60:
            return e_MAV_CMD_MAV_CMD_DO_GUIDED_LIMITS;
        case 61:
            return e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL;
        case 62:
            return e_MAV_CMD_MAV_CMD_DO_LAST;
        case 63:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION;
        case 64:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
        case 65:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN;
        case 66:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE;
        case 67:
            return e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
        case 68:
            return e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO;
        case 69:
            return e_MAV_CMD_MAV_CMD_MISSION_START;
        case 70:
            return e_MAV_CMD_MAV_CMD_COMPONENT_ARM_DISARM;
        case 71:
            return e_MAV_CMD_MAV_CMD_GET_HOME_POSITION;
        case 72:
            return e_MAV_CMD_MAV_CMD_START_RX_PAIR;
        case 73:
            return e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL;
        case 74:
            return e_MAV_CMD_MAV_CMD_SET_MESSAGE_INTERVAL;
        case 75:
            return e_MAV_CMD_MAV_CMD_REQUEST_PROTOCOL_VERSION;
        case 76:
            return e_MAV_CMD_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
        case 77:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION;
        case 78:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS;
        case 79:
            return e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION;
        case 80:
            return e_MAV_CMD_MAV_CMD_STORAGE_FORMAT;
        case 81:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
        case 82:
            return e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION;
        case 83:
            return e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS;
        case 84:
            return e_MAV_CMD_MAV_CMD_SET_CAMERA_MODE;
        case 85:
            return e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE;
        case 86:
            return e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE;
        case 87:
            return e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
        case 88:
            return e_MAV_CMD_MAV_CMD_DO_TRIGGER_CONTROL;
        case 89:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE;
        case 90:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_CAPTURE;
        case 91:
            return e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING;
        case 92:
            return e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING;
        case 93:
            return e_MAV_CMD_MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
        case 94:
            return e_MAV_CMD_MAV_CMD_LOGGING_START;
        case 95:
            return e_MAV_CMD_MAV_CMD_LOGGING_STOP;
        case 96:
            return e_MAV_CMD_MAV_CMD_AIRFRAME_CONFIGURATION;
        case 97:
            return e_MAV_CMD_MAV_CMD_PANORAMA_CREATE;
        case 98:
            return e_MAV_CMD_MAV_CMD_DO_VTOL_TRANSITION;
        case 99:
            return e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        case 100:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_STANDARD;
        case 101:
            return e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
        case 102:
            return e_MAV_CMD_MAV_CMD_CONDITION_GATE;
        case 103:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT;
        case 104:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
        case 105:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
        case 106:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
        case 107:
            return e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION;
        case 108:
            return e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT;
        case 109:
            return e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO;
        case 110:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 111:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 112:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 113:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 114:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 115:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 116:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 117:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 118:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 119:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 120:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 121:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 122:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 123:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 124:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 125:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 126:
            return e_MAV_CMD_MAV_CMD_USER_5;
        case 127:
            return e_MAV_CMD_MAV_CMD_RESET_MPPT;
        case 128:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL;
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
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_flight_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
INLINER void p148_middleware_custom_version_SET(uint8_t*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
/**
*Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
*	should allow to identify the commit using the main version number even for very large code bases*/
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
*	use uid*/
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
*	the landing targe*/
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
INLINER void p201_adc121_vspb_volt_SET(float  src, Pack * dst)//Power board voltage sensor reading in volts
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p201_adc121_cspb_amp_SET(float  src, Pack * dst)//Power board current sensor reading in amps
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p201_adc121_cs1_amp_SET(float  src, Pack * dst)//Board current sensor 1 reading in amps
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p201_adc121_cs2_amp_SET(float  src, Pack * dst)//Board current sensor 2 reading in amps
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
Pack * c_TEST_Channel_new_SENS_POWER_201()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 201));
};
INLINER void p202_mppt1_pwm_SET(uint16_t  src, Pack * dst)//MPPT1 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p202_mppt2_pwm_SET(uint16_t  src, Pack * dst)//MPPT2 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p202_mppt3_pwm_SET(uint16_t  src, Pack * dst)//MPPT3 pwm
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p202_mppt_timestamp_SET(uint64_t  src, Pack * dst)//MPPT last timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  6);
}
INLINER void p202_mppt1_volt_SET(float  src, Pack * dst)//MPPT1 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p202_mppt1_amp_SET(float  src, Pack * dst)//MPPT1 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p202_mppt1_status_SET(uint8_t  src, Pack * dst)//MPPT1 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  22);
}
INLINER void p202_mppt2_volt_SET(float  src, Pack * dst)//MPPT2 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  23);
}
INLINER void p202_mppt2_amp_SET(float  src, Pack * dst)//MPPT2 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  27);
}
INLINER void p202_mppt2_status_SET(uint8_t  src, Pack * dst)//MPPT2 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
INLINER void p202_mppt3_volt_SET(float  src, Pack * dst)//MPPT3 voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p202_mppt3_amp_SET(float  src, Pack * dst)//MPPT3 current
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p202_mppt3_status_SET(uint8_t  src, Pack * dst)//MPPT3 status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  40);
}
Pack * c_TEST_Channel_new_SENS_MPPT_202()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 202));
};
INLINER void p203_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p203_aslctrl_mode_SET(uint8_t  src, Pack * dst)//ASLCTRL control-mode (manual, stabilized, auto, etc...)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p203_h_SET(float  src, Pack * dst)//See sourcecode for a description of these values...
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER void p203_hRef_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER void p203_hRef_t_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
INLINER void p203_PitchAngle_SET(float  src, Pack * dst)//Pitch angle [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  21);
}
INLINER void p203_PitchAngleRef_SET(float  src, Pack * dst)//Pitch angle reference[deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  25);
}
INLINER void p203_q_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  29);
}
INLINER void p203_qRef_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  33);
}
INLINER void p203_uElev_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  37);
}
INLINER void p203_uThrot_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
INLINER void p203_uThrot2_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
INLINER void p203_nZ_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  49);
}
INLINER void p203_AirspeedRef_SET(float  src, Pack * dst)//Airspeed reference [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  53);
}
INLINER void p203_SpoilersEngaged_SET(uint8_t  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  57);
}
INLINER void p203_YawAngle_SET(float  src, Pack * dst)//Yaw angle [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  58);
}
INLINER void p203_YawAngleRef_SET(float  src, Pack * dst)//Yaw angle reference[deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  62);
}
INLINER void p203_RollAngle_SET(float  src, Pack * dst)//Roll angle [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  66);
}
INLINER void p203_RollAngleRef_SET(float  src, Pack * dst)//Roll angle reference[deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  70);
}
INLINER void p203_p_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  74);
}
INLINER void p203_pRef_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  78);
}
INLINER void p203_r_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  82);
}
INLINER void p203_rRef_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  86);
}
INLINER void p203_uAil_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  90);
}
INLINER void p203_uRud_SET(float  src, Pack * dst)//null
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  94);
}
Pack * c_TEST_Channel_new_ASLCTRL_DATA_203()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 203));
};
INLINER void p204_i32_1_SET(uint32_t  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
INLINER void p204_i8_1_SET(uint8_t  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p204_i8_2_SET(uint8_t  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p204_f_1_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p204_f_2_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p204_f_3_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p204_f_4_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p204_f_5_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p204_f_6_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p204_f_7_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p204_f_8_SET(float  src, Pack * dst)//Debug data
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
Pack * c_TEST_Channel_new_ASLCTRL_DEBUG_204()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 204));
};
INLINER void p205_LED_status_SET(uint8_t  src, Pack * dst)//Status of the position-indicator LEDs
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p205_SATCOM_status_SET(uint8_t  src, Pack * dst)//Status of the IRIDIUM satellite communication system
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p205_Servo_status_SET(uint8_t*  src, int32_t pos, Pack * dst) //Status vector for up to 8 servos
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 8; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p205_Motor_rpm_SET(float  src, Pack * dst)//Motor RPM
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_ASLUAV_STATUS_205()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 205));
};
INLINER void p206_timestamp_SET(uint64_t  src, Pack * dst)//Time since system start [us]
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p206_Windspeed_SET(float  src, Pack * dst)//Magnitude of wind velocity (in lateral inertial plane) [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p206_WindDir_SET(float  src, Pack * dst)//Wind heading angle from North [rad]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p206_WindZ_SET(float  src, Pack * dst)//Z (Down) component of inertial wind velocity [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p206_Airspeed_SET(float  src, Pack * dst)//Magnitude of air velocity [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p206_beta_SET(float  src, Pack * dst)//Sideslip angle [rad]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p206_alpha_SET(float  src, Pack * dst)//Angle of attack [rad]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
Pack * c_TEST_Channel_new_EKF_EXT_206()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 206));
};
INLINER void p207_timestamp_SET(uint64_t  src, Pack * dst)//Time since system start [us]
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p207_uElev_SET(float  src, Pack * dst)//Elevator command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p207_uThrot_SET(float  src, Pack * dst)//Throttle command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p207_uThrot2_SET(float  src, Pack * dst)//Throttle 2 command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p207_uAilL_SET(float  src, Pack * dst)//Left aileron command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p207_uAilR_SET(float  src, Pack * dst)//Right aileron command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p207_uRud_SET(float  src, Pack * dst)//Rudder command [~]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p207_obctrl_status_SET(uint8_t  src, Pack * dst)//Off-board computer status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  32);
}
Pack * c_TEST_Channel_new_ASL_OBCTRL_207()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 207));
};
INLINER void p208_TempAmbient_SET(float  src, Pack * dst)//Ambient temperature [degrees Celsius]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p208_Humidity_SET(float  src, Pack * dst)//Relative humidity [%]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
Pack * c_TEST_Channel_new_SENS_ATMOS_208()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 208));
};
INLINER void p209_voltage_SET(uint16_t  src, Pack * dst)//Battery pack voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p209_batterystatus_SET(uint16_t  src, Pack * dst)//Battery monitor status report bits in Hex
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p209_serialnumber_SET(uint16_t  src, Pack * dst)//Battery monitor serial number in Hex
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  4);
}
INLINER void p209_hostfetcontrol_SET(uint16_t  src, Pack * dst)//Battery monitor sensor host FET control in Hex
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  6);
}
INLINER void p209_cellvoltage1_SET(uint16_t  src, Pack * dst)//Battery pack cell 1 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  8);
}
INLINER void p209_cellvoltage2_SET(uint16_t  src, Pack * dst)//Battery pack cell 2 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  10);
}
INLINER void p209_cellvoltage3_SET(uint16_t  src, Pack * dst)//Battery pack cell 3 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  12);
}
INLINER void p209_cellvoltage4_SET(uint16_t  src, Pack * dst)//Battery pack cell 4 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  14);
}
INLINER void p209_cellvoltage5_SET(uint16_t  src, Pack * dst)//Battery pack cell 5 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  16);
}
INLINER void p209_cellvoltage6_SET(uint16_t  src, Pack * dst)//Battery pack cell 6 voltage in [mV]
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  18);
}
INLINER void p209_temperature_SET(float  src, Pack * dst)//Battery pack temperature in [deg C]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p209_current_SET(int16_t  src, Pack * dst)//Battery pack current in [mA]
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  24);
}
INLINER void p209_SoC_SET(uint8_t  src, Pack * dst)//Battery pack state-of-charge
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  26);
}
Pack * c_TEST_Channel_new_SENS_BATMON_209()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 209));
};
INLINER void p210_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp [ms]
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p210_timestampModeChanged_SET(uint64_t  src, Pack * dst)//Timestamp since last mode change[ms]
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  8);
}
INLINER void p210_xW_SET(float  src, Pack * dst)//Thermal core updraft strength [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p210_xR_SET(float  src, Pack * dst)//Thermal radius [m]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
INLINER void p210_xLat_SET(float  src, Pack * dst)//Thermal center latitude [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  24);
}
INLINER void p210_xLon_SET(float  src, Pack * dst)//Thermal center longitude [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  28);
}
INLINER void p210_VarW_SET(float  src, Pack * dst)//Variance W
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  32);
}
INLINER void p210_VarR_SET(float  src, Pack * dst)//Variance R
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  36);
}
INLINER void p210_VarLat_SET(float  src, Pack * dst)//Variance Lat
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
INLINER void p210_VarLon_SET(float  src, Pack * dst)//Variance Lon
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
INLINER void p210_LoiterRadius_SET(float  src, Pack * dst)//Suggested loiter radius [m]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  48);
}
INLINER void p210_LoiterDirection_SET(float  src, Pack * dst)//Suggested loiter direction
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  52);
}
INLINER void p210_DistToSoarPoint_SET(float  src, Pack * dst)//Distance to soar point [m]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  56);
}
INLINER void p210_vSinkExp_SET(float  src, Pack * dst)//Expected sink rate at current airspeed, roll and throttle [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  60);
}
INLINER void p210_z1_LocalUpdraftSpeed_SET(float  src, Pack * dst)//Measurement / updraft speed at current/local airplane position [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  64);
}
INLINER void p210_z2_DeltaRoll_SET(float  src, Pack * dst)//Measurement / roll angle tracking error [deg]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  68);
}
INLINER void p210_z1_exp_SET(float  src, Pack * dst)//Expected measurement 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  72);
}
INLINER void p210_z2_exp_SET(float  src, Pack * dst)//Expected measurement 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  76);
}
INLINER void p210_ThermalGSNorth_SET(float  src, Pack * dst)//Thermal drift (from estimator prediction step only) [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  80);
}
INLINER void p210_ThermalGSEast_SET(float  src, Pack * dst)//Thermal drift (from estimator prediction step only) [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  84);
}
INLINER void p210_TSE_dot_SET(float  src, Pack * dst)//Total specific energy change (filtered) [m/s]
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  88);
}
INLINER void p210_DebugVar1_SET(float  src, Pack * dst)//Debug variable 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  92);
}
INLINER void p210_DebugVar2_SET(float  src, Pack * dst)//Debug variable 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  96);
}
INLINER void p210_ControlMode_SET(uint8_t  src, Pack * dst)//Control Mode [-]
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  100);
}
INLINER void p210_valid_SET(uint8_t  src, Pack * dst)//Data valid [-]
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  101);
}
Pack * c_TEST_Channel_new_FW_SOARING_DATA_210()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 210));
};
INLINER void p211_free_space_SET(uint16_t  src, Pack * dst)//Free space available in recordings directory in [Gb] * 1e2
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p211_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp in linuxtime [ms] (since 1.1.1970)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  2);
}
INLINER void p211_visensor_rate_1_SET(uint8_t  src, Pack * dst)//Rate of ROS topic 1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p211_visensor_rate_2_SET(uint8_t  src, Pack * dst)//Rate of ROS topic 2
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
INLINER void p211_visensor_rate_3_SET(uint8_t  src, Pack * dst)//Rate of ROS topic 3
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  12);
}
INLINER void p211_visensor_rate_4_SET(uint8_t  src, Pack * dst)//Rate of ROS topic 4
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  13);
}
INLINER void p211_recording_nodes_count_SET(uint8_t  src, Pack * dst)//Number of recording nodes
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER void p211_cpu_temp_SET(uint8_t  src, Pack * dst)//Temperature of sensorpod CPU in [deg C]
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
Pack * c_TEST_Channel_new_SENSORPOD_STATUS_211()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 211));
};
INLINER void p212_timestamp_SET(uint64_t  src, Pack * dst)//Timestamp
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
INLINER void p212_pwr_brd_status_SET(uint8_t  src, Pack * dst)//Power board status register
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p212_pwr_brd_led_status_SET(uint8_t  src, Pack * dst)//Power board leds status
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p212_pwr_brd_system_volt_SET(float  src, Pack * dst)//Power board system voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p212_pwr_brd_servo_volt_SET(float  src, Pack * dst)//Power board servo voltage
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p212_pwr_brd_mot_l_amp_SET(float  src, Pack * dst)//Power board left motor current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p212_pwr_brd_mot_r_amp_SET(float  src, Pack * dst)//Power board right motor current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p212_pwr_brd_servo_1_amp_SET(float  src, Pack * dst)//Power board servo1 current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p212_pwr_brd_servo_2_amp_SET(float  src, Pack * dst)//Power board servo1 current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  30);
}
INLINER void p212_pwr_brd_servo_3_amp_SET(float  src, Pack * dst)//Power board servo1 current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  34);
}
INLINER void p212_pwr_brd_servo_4_amp_SET(float  src, Pack * dst)//Power board servo1 current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  38);
}
INLINER void p212_pwr_brd_aux_amp_SET(float  src, Pack * dst)//Power board aux current sensor
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  42);
}
Pack * c_TEST_Channel_new_SENS_POWER_BOARD_212()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 212));
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
*	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
*	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
*	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
*	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
*	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
*	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
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
*	bit3:GCS, bit4:fence*/
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
*	and slope of the groun*/
INLINER void p242_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  24, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p242_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  40);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p242_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  44);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
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
*	and slope of the groun*/
INLINER void p243_q_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
/**
*Local X position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p243_approach_x_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  41);
}
/**
*Local Y position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
INLINER void p243_approach_y_SET(float  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  45);
}
/**
*Local Z position of the end of the approach vector. Multicopters should set this position based on their
*	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
*	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
*	from the threshold / touchdown zone*/
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
    assert(p0_custom_mode_GET(pack) == (uint32_t)2765560595L);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_POWEROFF);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED));
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)41235);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)34483);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -17940);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)71);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)6691);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN));
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)12110);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)59);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)22888);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)57141);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)50154);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)389453065L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2904164346115591784L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_afz_GET(pack) == (float)2.28606E38F);
    assert(p3_vz_GET(pack) == (float)1.1184797E37F);
    assert(p3_yaw_rate_GET(pack) == (float) -1.7340803E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3687649025L);
    assert(p3_yaw_GET(pack) == (float)1.8057346E38F);
    assert(p3_z_GET(pack) == (float) -2.8835187E38F);
    assert(p3_x_GET(pack) == (float)1.5294025E38F);
    assert(p3_afy_GET(pack) == (float) -1.1907058E38F);
    assert(p3_afx_GET(pack) == (float)1.5692594E38F);
    assert(p3_y_GET(pack) == (float) -1.5284626E38F);
    assert(p3_vy_GET(pack) == (float)3.0810749E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)29615);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p3_vx_GET(pack) == (float) -2.7560558E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p4_seq_GET(pack) == (uint32_t)1508622707L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p4_time_usec_GET(pack) == (uint64_t)4300620714996981620L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_passkey_LEN(ph) == 16);
    {
        char16_t * exemplary = u"Fbmflercsrbgpgtn";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)31);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 22);
    {
        char16_t * exemplary = u"cyLlxofkutacvrtcwimlzw";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_DISARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2945153450L);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -21370);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p20_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"r";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)163);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)74);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)31439);
    assert(p22_param_value_GET(pack) == (float) -2.791541E38F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)62353);
    assert(p22_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"uKykosap";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p23_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"fXlfhw";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16);
    assert(p23_param_value_GET(pack) == (float) -2.9887598E36F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -1165129026);
    assert(p24_time_usec_GET(pack) == (uint64_t)548337702172422829L);
    assert(p24_lat_GET(pack) == (int32_t)818619306);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)61369);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)26241);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)502445952L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)48332);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)41855);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)801040295L);
    assert(p24_lon_GET(pack) == (int32_t)537885413);
    assert(p24_h_acc_TRY(ph) == (uint32_t)429879186L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)790189333L);
    assert(p24_alt_GET(pack) == (int32_t)480573365);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)132, (uint8_t)221, (uint8_t)35, (uint8_t)12, (uint8_t)240, (uint8_t)110, (uint8_t)94, (uint8_t)233, (uint8_t)24, (uint8_t)171, (uint8_t)238, (uint8_t)72, (uint8_t)64, (uint8_t)219, (uint8_t)190, (uint8_t)162, (uint8_t)12, (uint8_t)131, (uint8_t)55, (uint8_t)72} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)12);
    {
        uint8_t exemplary[] =  {(uint8_t)35, (uint8_t)181, (uint8_t)88, (uint8_t)172, (uint8_t)27, (uint8_t)188, (uint8_t)243, (uint8_t)220, (uint8_t)216, (uint8_t)202, (uint8_t)105, (uint8_t)107, (uint8_t)235, (uint8_t)166, (uint8_t)235, (uint8_t)234, (uint8_t)136, (uint8_t)163, (uint8_t)182, (uint8_t)124} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)250, (uint8_t)163, (uint8_t)214, (uint8_t)154, (uint8_t)50, (uint8_t)245, (uint8_t)130, (uint8_t)50, (uint8_t)136, (uint8_t)39, (uint8_t)69, (uint8_t)116, (uint8_t)146, (uint8_t)109, (uint8_t)121, (uint8_t)108, (uint8_t)75, (uint8_t)40, (uint8_t)123, (uint8_t)150} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)30, (uint8_t)65, (uint8_t)6, (uint8_t)103, (uint8_t)55, (uint8_t)243, (uint8_t)213, (uint8_t)240, (uint8_t)27, (uint8_t)124, (uint8_t)39, (uint8_t)138, (uint8_t)210, (uint8_t)66, (uint8_t)46, (uint8_t)59, (uint8_t)81, (uint8_t)27, (uint8_t)22} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)25, (uint8_t)52, (uint8_t)74, (uint8_t)112, (uint8_t)240, (uint8_t)241, (uint8_t)244, (uint8_t)205, (uint8_t)49, (uint8_t)94, (uint8_t)214, (uint8_t)82, (uint8_t)52, (uint8_t)62, (uint8_t)228, (uint8_t)235, (uint8_t)133, (uint8_t)164, (uint8_t)29, (uint8_t)220} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)8699);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)20867);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)29652);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -9127);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -16718);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2022357312L);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)5228);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)30435);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -9055);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)22823);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)26801);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)11868);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)20769);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)12889);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -14048);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -95);
    assert(p27_time_usec_GET(pack) == (uint64_t)3018704613867489834L);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)11187);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)32237);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)4802);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)135);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -26596);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)8262);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)4929);
    assert(p28_time_usec_GET(pack) == (uint64_t)4824776670802958611L);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float) -1.1724734E38F);
    assert(p29_press_diff_GET(pack) == (float) -2.4906438E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1258189388L);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -2543);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)1078058709L);
    assert(p30_yawspeed_GET(pack) == (float) -1.5910043E37F);
    assert(p30_roll_GET(pack) == (float) -9.858389E37F);
    assert(p30_rollspeed_GET(pack) == (float)2.0982719E38F);
    assert(p30_yaw_GET(pack) == (float) -2.6713252E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -3.206644E38F);
    assert(p30_pitch_GET(pack) == (float)2.723118E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float) -2.4475597E38F);
    assert(p31_rollspeed_GET(pack) == (float) -1.0160632E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -2.1011183E38F);
    assert(p31_q1_GET(pack) == (float)2.3889117E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2866446519L);
    assert(p31_q2_GET(pack) == (float) -2.3637392E38F);
    assert(p31_q4_GET(pack) == (float) -3.435712E37F);
    assert(p31_yawspeed_GET(pack) == (float) -2.5997522E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2642280173L);
    assert(p32_vy_GET(pack) == (float)2.7353186E38F);
    assert(p32_y_GET(pack) == (float) -1.9428575E38F);
    assert(p32_x_GET(pack) == (float) -5.5791057E37F);
    assert(p32_z_GET(pack) == (float)3.2133255E38F);
    assert(p32_vz_GET(pack) == (float) -3.3085784E38F);
    assert(p32_vx_GET(pack) == (float)2.827804E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)4114887637L);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)9228);
    assert(p33_relative_alt_GET(pack) == (int32_t)1497693465);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)60207);
    assert(p33_lon_GET(pack) == (int32_t)411384470);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)14369);
    assert(p33_alt_GET(pack) == (int32_t) -1801273062);
    assert(p33_lat_GET(pack) == (int32_t) -212468275);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)13637);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)5999);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -831);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -12276);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)29835);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -13375);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)8547);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)17586);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -17827);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2225287753L);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)30068);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3199596468L);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)16200);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)30198);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)14644);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)36863);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)3542);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)13043);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)31381);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)21138);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)39925);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)17194);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)46632);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)31285);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)37365);
    assert(p36_time_usec_GET(pack) == (uint32_t)2559541550L);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)12339);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)39368);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)15996);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)56874);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)35003);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)46686);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)56631);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)16147);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)26119);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)16978);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -29822);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)18893);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)204);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -15543);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)7690);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)84);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_y_GET(pack) == (float)3.2811053E38F);
    assert(p39_param1_GET(pack) == (float)1.413518E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)24642);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p39_param4_GET(pack) == (float) -2.1585352E38F);
    assert(p39_z_GET(pack) == (float)6.821716E36F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p39_param2_GET(pack) == (float)2.3775986E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_x_GET(pack) == (float) -2.1482076E38F);
    assert(p39_param3_GET(pack) == (float)1.9861726E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)12);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)17071);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)4764);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)39991);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)56);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)26445);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)175);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)54535);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_DENIED);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)194);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t) -87210337);
    assert(p48_longitude_GET(pack) == (int32_t)737923821);
    assert(p48_latitude_GET(pack) == (int32_t) -1221526684);
    assert(p48_time_usec_TRY(ph) == (uint64_t)5554771672005870911L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)3771774416740675194L);
    assert(p49_longitude_GET(pack) == (int32_t) -1152321568);
    assert(p49_altitude_GET(pack) == (int32_t)1593725200);
    assert(p49_latitude_GET(pack) == (int32_t)1391041402);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"lltdlf";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p50_param_value_min_GET(pack) == (float) -9.74519E37F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -5179);
    assert(p50_param_value0_GET(pack) == (float) -3.3710417E38F);
    assert(p50_param_value_max_GET(pack) == (float) -1.5813724E38F);
    assert(p50_scale_GET(pack) == (float)1.529815E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)35866);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p54_p2z_GET(pack) == (float)2.3098728E38F);
    assert(p54_p1z_GET(pack) == (float) -2.9351482E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p54_p2x_GET(pack) == (float) -1.717018E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p54_p2y_GET(pack) == (float) -2.985116E38F);
    assert(p54_p1x_GET(pack) == (float)2.3615428E38F);
    assert(p54_p1y_GET(pack) == (float)2.3971222E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float)1.1878581E38F);
    assert(p55_p1y_GET(pack) == (float) -2.7469124E38F);
    assert(p55_p1z_GET(pack) == (float) -3.1616534E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p55_p2y_GET(pack) == (float)2.687833E38F);
    assert(p55_p2z_GET(pack) == (float)1.6697206E38F);
    assert(p55_p2x_GET(pack) == (float) -1.3836933E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_time_usec_GET(pack) == (uint64_t)504932440565637652L);
    assert(p61_yawspeed_GET(pack) == (float)8.641544E37F);
    assert(p61_pitchspeed_GET(pack) == (float)8.883747E37F);
    {
        float exemplary[] =  {6.705601E37F, 6.082038E37F, 2.4419015E38F, 3.1459472E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -8.660185E37F);
    {
        float exemplary[] =  {3.2484912E38F, 1.6568845E38F, -1.3075093E37F, -2.1894364E38F, -4.7509334E37F, -1.3305802E38F, 1.58605E37F, 2.6180847E38F, 3.1964242E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)56258);
    assert(p62_alt_error_GET(pack) == (float)7.20684E37F);
    assert(p62_aspd_error_GET(pack) == (float) -2.6977704E38F);
    assert(p62_nav_pitch_GET(pack) == (float) -2.49632E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -2.0374413E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -98);
    assert(p62_nav_roll_GET(pack) == (float)2.0906606E37F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)32148);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vy_GET(pack) == (float) -1.5260762E38F);
    {
        float exemplary[] =  {-1.6336284E38F, -1.7525947E38F, -8.3366007E37F, 1.0797073E38F, -1.4674999E38F, -1.699904E38F, 4.2340278E37F, -1.7932378E38F, 7.481944E37F, -1.7216403E38F, -6.75903E37F, -1.061725E37F, 1.0108357E38F, 1.1923779E38F, -1.1735329E38F, -1.3566328E38F, -1.8322679E38F, 1.756308E38F, 1.826495E38F, -3.283705E38F, -7.6467975E37F, -1.733567E38F, 1.9427897E38F, -1.7305278E38F, -1.9478543E38F, -9.953078E37F, -2.7793424E38F, 3.3665092E38F, 2.1310438E38F, -3.4844972E37F, 3.3343046E38F, -1.9570002E38F, 2.6099772E37F, -3.1705189E38F, 3.3979622E38F, -3.3707967E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_lon_GET(pack) == (int32_t)531195949);
    assert(p63_relative_alt_GET(pack) == (int32_t) -1734102324);
    assert(p63_time_usec_GET(pack) == (uint64_t)6910023968277355277L);
    assert(p63_lat_GET(pack) == (int32_t)1935886934);
    assert(p63_vx_GET(pack) == (float)1.461029E38F);
    assert(p63_vz_GET(pack) == (float)5.189075E37F);
    assert(p63_alt_GET(pack) == (int32_t)714640761);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vz_GET(pack) == (float) -2.784719E38F);
    assert(p64_x_GET(pack) == (float)2.569028E38F);
    assert(p64_vy_GET(pack) == (float) -3.1453405E38F);
    assert(p64_ay_GET(pack) == (float) -2.993234E38F);
    assert(p64_ax_GET(pack) == (float) -1.1272131E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
    assert(p64_time_usec_GET(pack) == (uint64_t)1243273728801440186L);
    assert(p64_z_GET(pack) == (float) -2.899734E38F);
    assert(p64_y_GET(pack) == (float) -6.5168006E37F);
    assert(p64_az_GET(pack) == (float)2.2328884E38F);
    assert(p64_vx_GET(pack) == (float)1.4072036E38F);
    {
        float exemplary[] =  {-1.2414482E38F, 1.7600872E38F, -2.2954213E38F, -1.856652E38F, -2.3349522E38F, 1.1506343E38F, -2.6191166E38F, 1.2032473E38F, -2.3101904E38F, 1.6458957E37F, -1.711168E37F, -2.5143716E38F, -7.123413E37F, 1.3240676E38F, -7.707169E37F, 2.0442754E38F, -2.4997328E38F, -3.2127655E38F, -2.6852233E38F, -2.2803432E38F, -2.1297232E38F, 1.3315088E38F, -1.958259E38F, -1.9939935E38F, 1.9013222E38F, -2.7606388E38F, -1.796513E38F, -1.6648603E38F, 1.1853338E38F, 2.0207054E38F, -1.9865953E38F, 2.9566202E38F, -1.0853504E38F, -8.869235E37F, 1.6102348E38F, -2.412652E38F, 1.0875981E38F, 1.8218368E38F, -1.081966E38F, 2.5017867E38F, 2.1683575E38F, 8.836392E37F, 2.4132358E38F, -2.4604905E38F, -2.908512E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)822729750L);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)29873);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)43344);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)42076);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)3759);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)46329);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)22954);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)50789);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)56292);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)11260);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)3365);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)59751);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)44669);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)44728);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)61254);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)26364);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)61332);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)55975);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)5804);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)43377);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)36);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)42571);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)198);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)25896);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)58188);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -17081);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)19256);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)5815);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)23167);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)62049);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)38093);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)22386);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)2838);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)39296);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)24467);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)43240);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_param3_GET(pack) == (float) -2.7265112E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p73_param1_GET(pack) == (float) -4.834003E37F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION);
    assert(p73_param4_GET(pack) == (float) -1.6046381E38F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)212);
    assert(p73_y_GET(pack) == (int32_t) -1627602531);
    assert(p73_param2_GET(pack) == (float)1.4410962E38F);
    assert(p73_x_GET(pack) == (int32_t)1966636293);
    assert(p73_z_GET(pack) == (float)3.2321985E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_alt_GET(pack) == (float) -3.9373114E37F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)58823);
    assert(p74_climb_GET(pack) == (float) -3.987933E37F);
    assert(p74_groundspeed_GET(pack) == (float) -1.0442035E38F);
    assert(p74_airspeed_GET(pack) == (float)7.044281E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)6954);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p75_z_GET(pack) == (float) -2.9496735E38F);
    assert(p75_param2_GET(pack) == (float)7.7799287E37F);
    assert(p75_param4_GET(pack) == (float) -7.78055E37F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p75_param1_GET(pack) == (float) -2.4005228E38F);
    assert(p75_x_GET(pack) == (int32_t)888939576);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p75_param3_GET(pack) == (float)2.3381805E38F);
    assert(p75_y_GET(pack) == (int32_t)932515132);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_RELAY);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)141);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float)1.2146495E38F);
    assert(p76_param1_GET(pack) == (float) -8.7984244E36F);
    assert(p76_param3_GET(pack) == (float)3.2084841E38F);
    assert(p76_param4_GET(pack) == (float)1.8624874E38F);
    assert(p76_param6_GET(pack) == (float)2.4021202E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED);
    assert(p76_param7_GET(pack) == (float)1.898853E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p76_param5_GET(pack) == (float)2.7341126E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)63);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)6);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)206);
    assert(p77_result_param2_TRY(ph) == (int32_t) -733126142);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_WAYPOINT);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_yaw_GET(pack) == (float)5.493314E37F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p81_thrust_GET(pack) == (float)2.5057286E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p81_roll_GET(pack) == (float) -2.3932769E38F);
    assert(p81_pitch_GET(pack) == (float) -1.1149528E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1121550506L);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_roll_rate_GET(pack) == (float)2.4651131E38F);
    assert(p82_thrust_GET(pack) == (float) -4.1609665E37F);
    assert(p82_body_pitch_rate_GET(pack) == (float)3.2751331E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)4023736381L);
    {
        float exemplary[] =  {-1.3508732E38F, 2.4452937E38F, -9.714557E36F, 6.1933215E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p82_body_yaw_rate_GET(pack) == (float)7.137042E37F);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p83_body_roll_rate_GET(pack) == (float)2.9337958E38F);
    assert(p83_thrust_GET(pack) == (float) -2.330182E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)2.1823798E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)630258726L);
    assert(p83_body_pitch_rate_GET(pack) == (float)1.4508214E38F);
    {
        float exemplary[] =  {1.8296695E38F, -1.5013688E38F, -6.303796E37F, -1.7825145E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p84_afz_GET(pack) == (float) -1.6416033E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p84_yaw_GET(pack) == (float)1.9134636E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -3.221492E38F);
    assert(p84_x_GET(pack) == (float)2.957576E38F);
    assert(p84_vx_GET(pack) == (float)2.156836E38F);
    assert(p84_y_GET(pack) == (float)3.1753842E38F);
    assert(p84_afx_GET(pack) == (float)3.0877853E38F);
    assert(p84_vz_GET(pack) == (float)8.648285E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3496519776L);
    assert(p84_z_GET(pack) == (float)1.53507E38F);
    assert(p84_afy_GET(pack) == (float) -3.0100029E38F);
    assert(p84_vy_GET(pack) == (float) -3.9861849E37F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)63551);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_alt_GET(pack) == (float)1.3151898E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2029955245L);
    assert(p86_lat_int_GET(pack) == (int32_t) -2079461291);
    assert(p86_yaw_GET(pack) == (float)1.3802159E38F);
    assert(p86_vz_GET(pack) == (float)2.3835884E38F);
    assert(p86_vy_GET(pack) == (float) -2.4878156E38F);
    assert(p86_vx_GET(pack) == (float) -1.6808125E37F);
    assert(p86_afz_GET(pack) == (float) -2.0598783E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -2.335574E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)169839176);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)37300);
    assert(p86_afx_GET(pack) == (float) -1.1747156E37F);
    assert(p86_afy_GET(pack) == (float)2.188726E38F);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_vx_GET(pack) == (float)2.8481606E38F);
    assert(p87_yaw_GET(pack) == (float)3.2404664E38F);
    assert(p87_alt_GET(pack) == (float)1.2891385E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)35757);
    assert(p87_lon_int_GET(pack) == (int32_t)724419513);
    assert(p87_afz_GET(pack) == (float) -1.9577375E38F);
    assert(p87_lat_int_GET(pack) == (int32_t)1400318430);
    assert(p87_afy_GET(pack) == (float)1.6131639E38F);
    assert(p87_vz_GET(pack) == (float) -2.2817204E38F);
    assert(p87_vy_GET(pack) == (float) -1.5180126E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)306592126L);
    assert(p87_afx_GET(pack) == (float)2.0580547E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p87_yaw_rate_GET(pack) == (float) -1.0139975E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)737846300L);
    assert(p89_roll_GET(pack) == (float) -3.2615492E38F);
    assert(p89_yaw_GET(pack) == (float)8.93284E37F);
    assert(p89_x_GET(pack) == (float) -2.4288751E38F);
    assert(p89_pitch_GET(pack) == (float)1.378589E38F);
    assert(p89_y_GET(pack) == (float)1.442257E38F);
    assert(p89_z_GET(pack) == (float)1.6132474E37F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_pitch_GET(pack) == (float) -2.6513219E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)19836);
    assert(p90_time_usec_GET(pack) == (uint64_t)440135096532141811L);
    assert(p90_pitchspeed_GET(pack) == (float) -3.2211527E38F);
    assert(p90_yawspeed_GET(pack) == (float)2.2436404E38F);
    assert(p90_lon_GET(pack) == (int32_t) -2071187641);
    assert(p90_roll_GET(pack) == (float) -3.0624136E38F);
    assert(p90_alt_GET(pack) == (int32_t)605984641);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -21496);
    assert(p90_rollspeed_GET(pack) == (float)1.9714812E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -17870);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)9634);
    assert(p90_yaw_GET(pack) == (float)2.397357E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -9829);
    assert(p90_lat_GET(pack) == (int32_t) -313568902);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)28953);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p91_throttle_GET(pack) == (float)3.0359544E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p91_aux3_GET(pack) == (float) -3.2733004E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -2.6084298E38F);
    assert(p91_aux2_GET(pack) == (float) -1.2189926E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -1.5529365E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)2.4230853E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)7732800735633922047L);
    assert(p91_aux4_GET(pack) == (float)2.1467632E38F);
    assert(p91_aux1_GET(pack) == (float) -2.5489237E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_time_usec_GET(pack) == (uint64_t)4990373399804635633L);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)42475);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)33528);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)50812);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)61294);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)15003);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)58620);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)21744);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)31443);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)9490);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)55756);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)35377);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)54238);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.9051563E38F, -1.061897E38F, -2.6617436E38F, 2.5953696E38F, 1.8149421E37F, -1.0928376E38F, -2.8882524E37F, -1.0977422E37F, 7.59596E37F, -4.703896E37F, -3.0071296E37F, -2.244871E38F, 3.126668E38F, 9.458305E37F, 5.298124E37F, 3.1206327E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)6453685364167101055L);
    assert(p93_time_usec_GET(pack) == (uint64_t)5255576608208803422L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_comp_m_y_GET(pack) == (float)2.8918485E38F);
    assert(p100_ground_distance_GET(pack) == (float) -5.9977626E37F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.3243757E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)12300);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)18641);
    assert(p100_flow_rate_y_TRY(ph) == (float) -1.2604758E36F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.9603462E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)5385234053433603527L);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_y_GET(pack) == (float)2.1137256E38F);
    assert(p101_yaw_GET(pack) == (float)3.112193E38F);
    assert(p101_x_GET(pack) == (float)7.909868E37F);
    assert(p101_pitch_GET(pack) == (float) -1.5756092E38F);
    assert(p101_roll_GET(pack) == (float) -3.1647627E38F);
    assert(p101_z_GET(pack) == (float)3.0806999E38F);
    assert(p101_usec_GET(pack) == (uint64_t)865265721480554490L);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_usec_GET(pack) == (uint64_t)7619618762669387886L);
    assert(p102_roll_GET(pack) == (float)1.6781606E38F);
    assert(p102_y_GET(pack) == (float) -2.716056E38F);
    assert(p102_z_GET(pack) == (float)2.3860598E38F);
    assert(p102_x_GET(pack) == (float)1.3414545E37F);
    assert(p102_yaw_GET(pack) == (float) -6.2354805E37F);
    assert(p102_pitch_GET(pack) == (float)1.881858E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float)7.934822E37F);
    assert(p103_usec_GET(pack) == (uint64_t)4168804493007514797L);
    assert(p103_x_GET(pack) == (float) -7.395469E36F);
    assert(p103_y_GET(pack) == (float) -1.083757E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_usec_GET(pack) == (uint64_t)8795994295406990599L);
    assert(p104_x_GET(pack) == (float) -1.9784783E38F);
    assert(p104_y_GET(pack) == (float)2.2089268E38F);
    assert(p104_roll_GET(pack) == (float)3.016782E38F);
    assert(p104_pitch_GET(pack) == (float)6.876075E37F);
    assert(p104_z_GET(pack) == (float)2.68513E38F);
    assert(p104_yaw_GET(pack) == (float)8.0956984E37F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_zmag_GET(pack) == (float) -3.084696E38F);
    assert(p105_ymag_GET(pack) == (float)1.4772264E38F);
    assert(p105_xmag_GET(pack) == (float)2.6629901E38F);
    assert(p105_zacc_GET(pack) == (float)2.1635766E38F);
    assert(p105_abs_pressure_GET(pack) == (float)9.315963E37F);
    assert(p105_zgyro_GET(pack) == (float)2.6750229E38F);
    assert(p105_pressure_alt_GET(pack) == (float)3.2928403E38F);
    assert(p105_ygyro_GET(pack) == (float)2.1448283E38F);
    assert(p105_xgyro_GET(pack) == (float)2.1798473E38F);
    assert(p105_yacc_GET(pack) == (float)2.0091737E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.1180095E38F);
    assert(p105_xacc_GET(pack) == (float) -2.1815214E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)2692725512311767966L);
    assert(p105_temperature_GET(pack) == (float)1.0782369E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)25291);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3673300684L);
    assert(p106_integrated_ygyro_GET(pack) == (float)6.0287796E36F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -7.2757425E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p106_integrated_x_GET(pack) == (float) -1.4337514E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)3432953820719864489L);
    assert(p106_distance_GET(pack) == (float)2.2292686E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -1.5611486E37F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1684602428L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -18683);
    assert(p106_integrated_y_GET(pack) == (float)2.4044766E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_ygyro_GET(pack) == (float)1.1394049E38F);
    assert(p107_zacc_GET(pack) == (float)2.0385364E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)8408069849837580862L);
    assert(p107_diff_pressure_GET(pack) == (float) -2.7990575E38F);
    assert(p107_xmag_GET(pack) == (float) -3.0129824E38F);
    assert(p107_yacc_GET(pack) == (float)2.1935493E38F);
    assert(p107_ymag_GET(pack) == (float)1.4388346E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -3.2416627E38F);
    assert(p107_xacc_GET(pack) == (float) -2.0921774E38F);
    assert(p107_xgyro_GET(pack) == (float) -1.0606056E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -4.9913773E37F);
    assert(p107_zmag_GET(pack) == (float)1.6663808E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2238306458L);
    assert(p107_temperature_GET(pack) == (float)4.5618805E37F);
    assert(p107_zgyro_GET(pack) == (float)1.5649051E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_std_dev_vert_GET(pack) == (float)2.0802463E38F);
    assert(p108_yaw_GET(pack) == (float)9.408274E37F);
    assert(p108_std_dev_horz_GET(pack) == (float)1.331162E38F);
    assert(p108_vn_GET(pack) == (float)1.8324407E38F);
    assert(p108_q3_GET(pack) == (float)1.8930585E38F);
    assert(p108_yacc_GET(pack) == (float)1.9577557E38F);
    assert(p108_ygyro_GET(pack) == (float) -5.0784355E37F);
    assert(p108_xgyro_GET(pack) == (float) -7.4874863E37F);
    assert(p108_roll_GET(pack) == (float) -8.592887E37F);
    assert(p108_zgyro_GET(pack) == (float)1.0522486E38F);
    assert(p108_q1_GET(pack) == (float)2.7665337E38F);
    assert(p108_lon_GET(pack) == (float)2.0981658E38F);
    assert(p108_q4_GET(pack) == (float)1.3751737E38F);
    assert(p108_lat_GET(pack) == (float) -2.7244079E38F);
    assert(p108_ve_GET(pack) == (float)6.3651977E37F);
    assert(p108_q2_GET(pack) == (float) -2.0444782E38F);
    assert(p108_pitch_GET(pack) == (float) -2.6886354E38F);
    assert(p108_xacc_GET(pack) == (float)1.9671895E38F);
    assert(p108_vd_GET(pack) == (float) -7.965017E37F);
    assert(p108_zacc_GET(pack) == (float)1.0138871E38F);
    assert(p108_alt_GET(pack) == (float) -1.8948588E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)6490);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)26162);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)126);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)181);
    {
        uint8_t exemplary[] =  {(uint8_t)202, (uint8_t)19, (uint8_t)216, (uint8_t)10, (uint8_t)205, (uint8_t)132, (uint8_t)155, (uint8_t)157, (uint8_t)181, (uint8_t)138, (uint8_t)111, (uint8_t)190, (uint8_t)120, (uint8_t)68, (uint8_t)107, (uint8_t)91, (uint8_t)58, (uint8_t)203, (uint8_t)106, (uint8_t)221, (uint8_t)178, (uint8_t)40, (uint8_t)164, (uint8_t)53, (uint8_t)55, (uint8_t)88, (uint8_t)7, (uint8_t)181, (uint8_t)219, (uint8_t)175, (uint8_t)22, (uint8_t)39, (uint8_t)65, (uint8_t)1, (uint8_t)114, (uint8_t)161, (uint8_t)179, (uint8_t)166, (uint8_t)93, (uint8_t)146, (uint8_t)146, (uint8_t)194, (uint8_t)238, (uint8_t)129, (uint8_t)145, (uint8_t)149, (uint8_t)177, (uint8_t)98, (uint8_t)200, (uint8_t)224, (uint8_t)1, (uint8_t)229, (uint8_t)30, (uint8_t)108, (uint8_t)135, (uint8_t)192, (uint8_t)215, (uint8_t)107, (uint8_t)245, (uint8_t)90, (uint8_t)23, (uint8_t)235, (uint8_t)12, (uint8_t)16, (uint8_t)216, (uint8_t)41, (uint8_t)138, (uint8_t)99, (uint8_t)12, (uint8_t)202, (uint8_t)150, (uint8_t)189, (uint8_t)187, (uint8_t)119, (uint8_t)73, (uint8_t)134, (uint8_t)224, (uint8_t)224, (uint8_t)34, (uint8_t)11, (uint8_t)160, (uint8_t)17, (uint8_t)253, (uint8_t)45, (uint8_t)235, (uint8_t)252, (uint8_t)172, (uint8_t)188, (uint8_t)212, (uint8_t)57, (uint8_t)127, (uint8_t)33, (uint8_t)61, (uint8_t)246, (uint8_t)120, (uint8_t)134, (uint8_t)165, (uint8_t)30, (uint8_t)32, (uint8_t)88, (uint8_t)48, (uint8_t)84, (uint8_t)152, (uint8_t)53, (uint8_t)180, (uint8_t)160, (uint8_t)178, (uint8_t)66, (uint8_t)130, (uint8_t)203, (uint8_t)30, (uint8_t)70, (uint8_t)36, (uint8_t)224, (uint8_t)252, (uint8_t)133, (uint8_t)54, (uint8_t)109, (uint8_t)136, (uint8_t)17, (uint8_t)178, (uint8_t)148, (uint8_t)199, (uint8_t)253, (uint8_t)85, (uint8_t)250, (uint8_t)138, (uint8_t)35, (uint8_t)199, (uint8_t)80, (uint8_t)96, (uint8_t)88, (uint8_t)71, (uint8_t)0, (uint8_t)225, (uint8_t)189, (uint8_t)216, (uint8_t)152, (uint8_t)171, (uint8_t)235, (uint8_t)115, (uint8_t)226, (uint8_t)2, (uint8_t)34, (uint8_t)127, (uint8_t)123, (uint8_t)97, (uint8_t)95, (uint8_t)252, (uint8_t)88, (uint8_t)140, (uint8_t)108, (uint8_t)131, (uint8_t)62, (uint8_t)233, (uint8_t)207, (uint8_t)13, (uint8_t)181, (uint8_t)169, (uint8_t)251, (uint8_t)73, (uint8_t)147, (uint8_t)192, (uint8_t)129, (uint8_t)248, (uint8_t)127, (uint8_t)43, (uint8_t)10, (uint8_t)117, (uint8_t)89, (uint8_t)67, (uint8_t)55, (uint8_t)51, (uint8_t)234, (uint8_t)77, (uint8_t)223, (uint8_t)169, (uint8_t)146, (uint8_t)84, (uint8_t)196, (uint8_t)252, (uint8_t)41, (uint8_t)104, (uint8_t)110, (uint8_t)91, (uint8_t)46, (uint8_t)251, (uint8_t)74, (uint8_t)32, (uint8_t)241, (uint8_t)195, (uint8_t)176, (uint8_t)157, (uint8_t)68, (uint8_t)222, (uint8_t)145, (uint8_t)57, (uint8_t)234, (uint8_t)167, (uint8_t)4, (uint8_t)89, (uint8_t)231, (uint8_t)105, (uint8_t)129, (uint8_t)46, (uint8_t)33, (uint8_t)65, (uint8_t)192, (uint8_t)165, (uint8_t)109, (uint8_t)83, (uint8_t)206, (uint8_t)43, (uint8_t)29, (uint8_t)196, (uint8_t)159, (uint8_t)206, (uint8_t)33, (uint8_t)40, (uint8_t)232, (uint8_t)231, (uint8_t)176, (uint8_t)120, (uint8_t)176, (uint8_t)53, (uint8_t)234, (uint8_t)38, (uint8_t)87, (uint8_t)59, (uint8_t)173, (uint8_t)59, (uint8_t)161, (uint8_t)232, (uint8_t)230, (uint8_t)60, (uint8_t)49, (uint8_t)139, (uint8_t)221, (uint8_t)116, (uint8_t)48, (uint8_t)27, (uint8_t)168, (uint8_t)227, (uint8_t)147, (uint8_t)69, (uint8_t)33, (uint8_t)224, (uint8_t)42, (uint8_t)171, (uint8_t)27, (uint8_t)109} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -5661291639675183456L);
    assert(p111_tc1_GET(pack) == (int64_t) -3519508074601737126L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)1581092729751413986L);
    assert(p112_seq_GET(pack) == (uint32_t)306545998L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_time_usec_GET(pack) == (uint64_t)6112075186638870857L);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)10689);
    assert(p113_alt_GET(pack) == (int32_t) -1418259808);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)4844);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)2489);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)51739);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)17477);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)59138);
    assert(p113_lon_GET(pack) == (int32_t) -1489340241);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -31751);
    assert(p113_lat_GET(pack) == (int32_t) -1551286042);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_x_GET(pack) == (float) -2.007425E38F);
    assert(p114_integrated_y_GET(pack) == (float) -1.9330055E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.847266E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)3685658382347545169L);
    assert(p114_integrated_xgyro_GET(pack) == (float)6.899169E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)1947027844L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4274085146L);
    assert(p114_integrated_zgyro_GET(pack) == (float)3.1476292E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)10435);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p114_distance_GET(pack) == (float) -3.006049E38F);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_lat_GET(pack) == (int32_t)1996842973);
    assert(p115_time_usec_GET(pack) == (uint64_t)3573070863460337349L);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)37757);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -15803);
    assert(p115_rollspeed_GET(pack) == (float) -3.2248945E37F);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)7161);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -10836);
    assert(p115_lon_GET(pack) == (int32_t)70388355);
    assert(p115_yawspeed_GET(pack) == (float)3.0364414E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)28308);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -1504);
    {
        float exemplary[] =  {-8.9060694E35F, -2.4381008E38F, 3.3206032E38F, -2.5177543E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)13901);
    assert(p115_pitchspeed_GET(pack) == (float) -1.195181E38F);
    assert(p115_alt_GET(pack) == (int32_t) -1149005288);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)15526);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2315676151L);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)18244);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)7935);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)3961);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -23561);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)3931);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -10219);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)29945);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -32064);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -8463);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)61703);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)9714);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)47048);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)19270);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)52904);
    assert(p118_size_GET(pack) == (uint32_t)2202937657L);
    assert(p118_time_utc_GET(pack) == (uint32_t)1868786887L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_count_GET(pack) == (uint32_t)3042034430L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p119_ofs_GET(pack) == (uint32_t)4261066505L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)26792);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)212);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)83, (uint8_t)79, (uint8_t)79, (uint8_t)189, (uint8_t)100, (uint8_t)243, (uint8_t)10, (uint8_t)145, (uint8_t)95, (uint8_t)49, (uint8_t)96, (uint8_t)244, (uint8_t)51, (uint8_t)172, (uint8_t)193, (uint8_t)161, (uint8_t)26, (uint8_t)37, (uint8_t)131, (uint8_t)130, (uint8_t)253, (uint8_t)141, (uint8_t)87, (uint8_t)14, (uint8_t)253, (uint8_t)123, (uint8_t)221, (uint8_t)3, (uint8_t)91, (uint8_t)184, (uint8_t)189, (uint8_t)135, (uint8_t)153, (uint8_t)120, (uint8_t)236, (uint8_t)113, (uint8_t)179, (uint8_t)15, (uint8_t)67, (uint8_t)236, (uint8_t)42, (uint8_t)27, (uint8_t)185, (uint8_t)70, (uint8_t)103, (uint8_t)204, (uint8_t)227, (uint8_t)105, (uint8_t)128, (uint8_t)36, (uint8_t)251, (uint8_t)233, (uint8_t)16, (uint8_t)141, (uint8_t)203, (uint8_t)97, (uint8_t)112, (uint8_t)232, (uint8_t)33, (uint8_t)19, (uint8_t)188, (uint8_t)246, (uint8_t)162, (uint8_t)94, (uint8_t)197, (uint8_t)23, (uint8_t)138, (uint8_t)136, (uint8_t)96, (uint8_t)98, (uint8_t)196, (uint8_t)174, (uint8_t)220, (uint8_t)87, (uint8_t)132, (uint8_t)96, (uint8_t)101, (uint8_t)21, (uint8_t)47, (uint8_t)178, (uint8_t)170, (uint8_t)35, (uint8_t)63, (uint8_t)154, (uint8_t)7, (uint8_t)4, (uint8_t)101, (uint8_t)164, (uint8_t)6, (uint8_t)200} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)11938);
    assert(p120_ofs_GET(pack) == (uint32_t)4251109799L);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)185);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)106);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)174);
    {
        uint8_t exemplary[] =  {(uint8_t)211, (uint8_t)154, (uint8_t)152, (uint8_t)111, (uint8_t)253, (uint8_t)225, (uint8_t)154, (uint8_t)97, (uint8_t)254, (uint8_t)227, (uint8_t)181, (uint8_t)189, (uint8_t)183, (uint8_t)250, (uint8_t)203, (uint8_t)206, (uint8_t)200, (uint8_t)249, (uint8_t)226, (uint8_t)82, (uint8_t)171, (uint8_t)97, (uint8_t)170, (uint8_t)212, (uint8_t)78, (uint8_t)252, (uint8_t)231, (uint8_t)5, (uint8_t)130, (uint8_t)165, (uint8_t)147, (uint8_t)60, (uint8_t)184, (uint8_t)67, (uint8_t)102, (uint8_t)87, (uint8_t)170, (uint8_t)51, (uint8_t)251, (uint8_t)196, (uint8_t)125, (uint8_t)10, (uint8_t)163, (uint8_t)37, (uint8_t)145, (uint8_t)243, (uint8_t)91, (uint8_t)13, (uint8_t)67, (uint8_t)180, (uint8_t)185, (uint8_t)178, (uint8_t)139, (uint8_t)248, (uint8_t)204, (uint8_t)247, (uint8_t)80, (uint8_t)203, (uint8_t)238, (uint8_t)124, (uint8_t)132, (uint8_t)15, (uint8_t)138, (uint8_t)25, (uint8_t)60, (uint8_t)91, (uint8_t)202, (uint8_t)150, (uint8_t)119, (uint8_t)188, (uint8_t)13, (uint8_t)110, (uint8_t)6, (uint8_t)23, (uint8_t)159, (uint8_t)107, (uint8_t)30, (uint8_t)111, (uint8_t)203, (uint8_t)71, (uint8_t)237, (uint8_t)208, (uint8_t)239, (uint8_t)77, (uint8_t)192, (uint8_t)51, (uint8_t)80, (uint8_t)107, (uint8_t)245, (uint8_t)66, (uint8_t)98, (uint8_t)81, (uint8_t)49, (uint8_t)155, (uint8_t)167, (uint8_t)64, (uint8_t)188, (uint8_t)140, (uint8_t)118, (uint8_t)200, (uint8_t)86, (uint8_t)220, (uint8_t)117, (uint8_t)147, (uint8_t)43, (uint8_t)166, (uint8_t)219, (uint8_t)129, (uint8_t)165, (uint8_t)99} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)195);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_time_usec_GET(pack) == (uint64_t)3174395424994094556L);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)30096);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)26856);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p124_lat_GET(pack) == (int32_t) -1673918017);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)32809);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)656);
    assert(p124_lon_GET(pack) == (int32_t)639066999);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1080131957L);
    assert(p124_alt_GET(pack) == (int32_t)154152148);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)62378);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)62451);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)143, (uint8_t)37, (uint8_t)57, (uint8_t)159, (uint8_t)140, (uint8_t)32, (uint8_t)141, (uint8_t)83, (uint8_t)138, (uint8_t)61, (uint8_t)239, (uint8_t)110, (uint8_t)7, (uint8_t)0, (uint8_t)8, (uint8_t)160, (uint8_t)61, (uint8_t)70, (uint8_t)196, (uint8_t)118, (uint8_t)76, (uint8_t)49, (uint8_t)2, (uint8_t)167, (uint8_t)65, (uint8_t)196, (uint8_t)137, (uint8_t)94, (uint8_t)252, (uint8_t)17, (uint8_t)131, (uint8_t)184, (uint8_t)159, (uint8_t)78, (uint8_t)235, (uint8_t)180, (uint8_t)201, (uint8_t)236, (uint8_t)97, (uint8_t)30, (uint8_t)224, (uint8_t)227, (uint8_t)72, (uint8_t)26, (uint8_t)249, (uint8_t)132, (uint8_t)213, (uint8_t)73, (uint8_t)101, (uint8_t)90, (uint8_t)227, (uint8_t)143, (uint8_t)1, (uint8_t)112, (uint8_t)195, (uint8_t)81, (uint8_t)87, (uint8_t)237, (uint8_t)91, (uint8_t)223, (uint8_t)163, (uint8_t)24, (uint8_t)4, (uint8_t)171, (uint8_t)6, (uint8_t)51, (uint8_t)120, (uint8_t)35, (uint8_t)213, (uint8_t)213} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)53108);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING));
    assert(p126_baudrate_GET(pack) == (uint32_t)1020505612L);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -2079477454);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)8839);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)106988358L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)868218239);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -162489806);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1745317382);
    assert(p127_accuracy_GET(pack) == (uint32_t)1735505008L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p127_tow_GET(pack) == (uint32_t)2147003435L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)101);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -127357716);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)1809751482);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)2663007119L);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)1351442447);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p128_accuracy_GET(pack) == (uint32_t)4230312724L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1225865703);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)33559);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p128_tow_GET(pack) == (uint32_t)470932486L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)11193);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)26814);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)31363);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -10537);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1548431394L);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -16304);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)29361);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)891);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)797);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -13471);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_size_GET(pack) == (uint32_t)2080673138L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)29150);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)37523);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)33212);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)224);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)112, (uint8_t)32, (uint8_t)56, (uint8_t)237, (uint8_t)166, (uint8_t)149, (uint8_t)46, (uint8_t)118, (uint8_t)180, (uint8_t)169, (uint8_t)129, (uint8_t)16, (uint8_t)76, (uint8_t)104, (uint8_t)124, (uint8_t)189, (uint8_t)212, (uint8_t)232, (uint8_t)64, (uint8_t)128, (uint8_t)89, (uint8_t)110, (uint8_t)2, (uint8_t)239, (uint8_t)137, (uint8_t)245, (uint8_t)81, (uint8_t)41, (uint8_t)226, (uint8_t)172, (uint8_t)211, (uint8_t)16, (uint8_t)67, (uint8_t)120, (uint8_t)207, (uint8_t)43, (uint8_t)8, (uint8_t)206, (uint8_t)221, (uint8_t)156, (uint8_t)18, (uint8_t)179, (uint8_t)159, (uint8_t)92, (uint8_t)123, (uint8_t)135, (uint8_t)143, (uint8_t)237, (uint8_t)27, (uint8_t)218, (uint8_t)42, (uint8_t)246, (uint8_t)58, (uint8_t)230, (uint8_t)9, (uint8_t)108, (uint8_t)98, (uint8_t)111, (uint8_t)104, (uint8_t)197, (uint8_t)113, (uint8_t)74, (uint8_t)46, (uint8_t)58, (uint8_t)146, (uint8_t)0, (uint8_t)243, (uint8_t)3, (uint8_t)193, (uint8_t)120, (uint8_t)220, (uint8_t)236, (uint8_t)95, (uint8_t)67, (uint8_t)131, (uint8_t)17, (uint8_t)120, (uint8_t)47, (uint8_t)254, (uint8_t)173, (uint8_t)242, (uint8_t)181, (uint8_t)160, (uint8_t)199, (uint8_t)39, (uint8_t)201, (uint8_t)124, (uint8_t)112, (uint8_t)35, (uint8_t)170, (uint8_t)111, (uint8_t)72, (uint8_t)159, (uint8_t)14, (uint8_t)87, (uint8_t)101, (uint8_t)171, (uint8_t)214, (uint8_t)222, (uint8_t)30, (uint8_t)198, (uint8_t)45, (uint8_t)180, (uint8_t)210, (uint8_t)204, (uint8_t)246, (uint8_t)244, (uint8_t)143, (uint8_t)47, (uint8_t)218, (uint8_t)20, (uint8_t)135, (uint8_t)143, (uint8_t)86, (uint8_t)239, (uint8_t)147, (uint8_t)89, (uint8_t)20, (uint8_t)1, (uint8_t)186, (uint8_t)23, (uint8_t)82, (uint8_t)9, (uint8_t)113, (uint8_t)18, (uint8_t)127, (uint8_t)175, (uint8_t)34, (uint8_t)8, (uint8_t)8, (uint8_t)239, (uint8_t)200, (uint8_t)92, (uint8_t)109, (uint8_t)82, (uint8_t)104, (uint8_t)52, (uint8_t)161, (uint8_t)247, (uint8_t)154, (uint8_t)114, (uint8_t)165, (uint8_t)22, (uint8_t)134, (uint8_t)34, (uint8_t)53, (uint8_t)178, (uint8_t)24, (uint8_t)84, (uint8_t)81, (uint8_t)92, (uint8_t)24, (uint8_t)21, (uint8_t)48, (uint8_t)49, (uint8_t)46, (uint8_t)41, (uint8_t)210, (uint8_t)210, (uint8_t)253, (uint8_t)57, (uint8_t)47, (uint8_t)30, (uint8_t)62, (uint8_t)225, (uint8_t)108, (uint8_t)14, (uint8_t)255, (uint8_t)16, (uint8_t)142, (uint8_t)66, (uint8_t)124, (uint8_t)176, (uint8_t)228, (uint8_t)162, (uint8_t)85, (uint8_t)226, (uint8_t)62, (uint8_t)139, (uint8_t)243, (uint8_t)135, (uint8_t)0, (uint8_t)215, (uint8_t)195, (uint8_t)89, (uint8_t)149, (uint8_t)118, (uint8_t)20, (uint8_t)76, (uint8_t)105, (uint8_t)232, (uint8_t)244, (uint8_t)124, (uint8_t)153, (uint8_t)25, (uint8_t)41, (uint8_t)242, (uint8_t)73, (uint8_t)161, (uint8_t)148, (uint8_t)81, (uint8_t)181, (uint8_t)250, (uint8_t)68, (uint8_t)152, (uint8_t)64, (uint8_t)168, (uint8_t)193, (uint8_t)53, (uint8_t)122, (uint8_t)141, (uint8_t)187, (uint8_t)13, (uint8_t)234, (uint8_t)115, (uint8_t)238, (uint8_t)109, (uint8_t)141, (uint8_t)202, (uint8_t)8, (uint8_t)66, (uint8_t)106, (uint8_t)234, (uint8_t)70, (uint8_t)102, (uint8_t)75, (uint8_t)79, (uint8_t)20, (uint8_t)63, (uint8_t)41, (uint8_t)229, (uint8_t)97, (uint8_t)60, (uint8_t)158, (uint8_t)249, (uint8_t)255, (uint8_t)159, (uint8_t)47, (uint8_t)111, (uint8_t)131, (uint8_t)35, (uint8_t)199, (uint8_t)228, (uint8_t)173, (uint8_t)6, (uint8_t)201, (uint8_t)74, (uint8_t)245, (uint8_t)129, (uint8_t)230, (uint8_t)231, (uint8_t)206, (uint8_t)153} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)12609);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)47487);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3929050721L);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)57588);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)47447);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t) -1650183395);
    assert(p133_mask_GET(pack) == (uint64_t)7142430660652634481L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)16068);
    assert(p133_lon_GET(pack) == (int32_t)860656203);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lat_GET(pack) == (int32_t) -1307708020);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)171);
    {
        int16_t exemplary[] =  {(int16_t)12329, (int16_t)25304, (int16_t)11329, (int16_t) -18567, (int16_t) -8631, (int16_t)2801, (int16_t) -3771, (int16_t) -29410, (int16_t) -12931, (int16_t)25475, (int16_t) -23582, (int16_t) -30593, (int16_t)23704, (int16_t) -2204, (int16_t)3009, (int16_t)30347} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t) -185220575);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)55374);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)553852956);
    assert(p135_lon_GET(pack) == (int32_t)1541502441);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lon_GET(pack) == (int32_t) -433166983);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)30168);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)61132);
    assert(p136_terrain_height_GET(pack) == (float)2.3446822E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)18692);
    assert(p136_current_height_GET(pack) == (float)2.3808515E38F);
    assert(p136_lat_GET(pack) == (int32_t)1124509925);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)1625246619L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)7676);
    assert(p137_press_diff_GET(pack) == (float)2.6170963E38F);
    assert(p137_press_abs_GET(pack) == (float) -1.750134E38F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)3655874965958432471L);
    assert(p138_z_GET(pack) == (float)2.0006983E38F);
    assert(p138_y_GET(pack) == (float) -1.4543669E38F);
    assert(p138_x_GET(pack) == (float) -7.2635746E37F);
    {
        float exemplary[] =  {-3.1664385E38F, 2.933916E38F, 2.0031352E38F, -2.5617482E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p139_time_usec_GET(pack) == (uint64_t)4098417378326167877L);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)2);
    {
        float exemplary[] =  {-1.4454737E38F, -2.450314E38F, -5.0044843E37F, 2.7733589E38F, -2.912193E38F, 3.2973227E38F, -2.0633103E38F, -3.1971316E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p140_time_usec_GET(pack) == (uint64_t)8787075891538174162L);
    {
        float exemplary[] =  {2.7731066E38F, -2.3891387E38F, -2.9487419E38F, 3.2484407E38F, -1.219474E36F, 2.744994E38F, -3.0853652E38F, 6.876433E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_terrain_GET(pack) == (float) -1.6785081E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)1.3911744E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)2603011888920540343L);
    assert(p141_altitude_monotonic_GET(pack) == (float)3.891034E37F);
    assert(p141_altitude_relative_GET(pack) == (float)2.4050125E38F);
    assert(p141_altitude_local_GET(pack) == (float) -1.2287382E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.6917523E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)242, (uint8_t)190, (uint8_t)56, (uint8_t)137, (uint8_t)16, (uint8_t)89, (uint8_t)92, (uint8_t)140, (uint8_t)85, (uint8_t)88, (uint8_t)251, (uint8_t)3, (uint8_t)218, (uint8_t)6, (uint8_t)185, (uint8_t)199, (uint8_t)199, (uint8_t)154, (uint8_t)206, (uint8_t)77, (uint8_t)200, (uint8_t)182, (uint8_t)142, (uint8_t)73, (uint8_t)162, (uint8_t)126, (uint8_t)15, (uint8_t)42, (uint8_t)16, (uint8_t)46, (uint8_t)137, (uint8_t)210, (uint8_t)116, (uint8_t)52, (uint8_t)219, (uint8_t)51, (uint8_t)169, (uint8_t)132, (uint8_t)213, (uint8_t)216, (uint8_t)22, (uint8_t)205, (uint8_t)148, (uint8_t)211, (uint8_t)15, (uint8_t)124, (uint8_t)9, (uint8_t)44, (uint8_t)201, (uint8_t)171, (uint8_t)108, (uint8_t)207, (uint8_t)46, (uint8_t)44, (uint8_t)144, (uint8_t)109, (uint8_t)82, (uint8_t)200, (uint8_t)108, (uint8_t)229, (uint8_t)221, (uint8_t)14, (uint8_t)121, (uint8_t)241, (uint8_t)60, (uint8_t)144, (uint8_t)179, (uint8_t)89, (uint8_t)92, (uint8_t)136, (uint8_t)2, (uint8_t)94, (uint8_t)158, (uint8_t)221, (uint8_t)123, (uint8_t)176, (uint8_t)142, (uint8_t)138, (uint8_t)117, (uint8_t)239, (uint8_t)3, (uint8_t)86, (uint8_t)238, (uint8_t)138, (uint8_t)233, (uint8_t)9, (uint8_t)245, (uint8_t)237, (uint8_t)213, (uint8_t)119, (uint8_t)96, (uint8_t)22, (uint8_t)31, (uint8_t)179, (uint8_t)246, (uint8_t)46, (uint8_t)180, (uint8_t)250, (uint8_t)223, (uint8_t)180, (uint8_t)103, (uint8_t)196, (uint8_t)53, (uint8_t)127, (uint8_t)196, (uint8_t)7, (uint8_t)40, (uint8_t)230, (uint8_t)255, (uint8_t)72, (uint8_t)211, (uint8_t)52, (uint8_t)216, (uint8_t)239, (uint8_t)88, (uint8_t)129, (uint8_t)221, (uint8_t)23, (uint8_t)230, (uint8_t)128} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)198);
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)29, (uint8_t)53, (uint8_t)216, (uint8_t)239, (uint8_t)91, (uint8_t)73, (uint8_t)188, (uint8_t)218, (uint8_t)35, (uint8_t)176, (uint8_t)12, (uint8_t)106, (uint8_t)129, (uint8_t)254, (uint8_t)52, (uint8_t)46, (uint8_t)159, (uint8_t)215, (uint8_t)57, (uint8_t)84, (uint8_t)214, (uint8_t)141, (uint8_t)152, (uint8_t)21, (uint8_t)94, (uint8_t)0, (uint8_t)196, (uint8_t)73, (uint8_t)126, (uint8_t)12, (uint8_t)3, (uint8_t)32, (uint8_t)14, (uint8_t)3, (uint8_t)244, (uint8_t)102, (uint8_t)66, (uint8_t)113, (uint8_t)136, (uint8_t)254, (uint8_t)188, (uint8_t)226, (uint8_t)207, (uint8_t)176, (uint8_t)45, (uint8_t)12, (uint8_t)131, (uint8_t)150, (uint8_t)186, (uint8_t)191, (uint8_t)206, (uint8_t)131, (uint8_t)44, (uint8_t)93, (uint8_t)219, (uint8_t)70, (uint8_t)81, (uint8_t)212, (uint8_t)247, (uint8_t)41, (uint8_t)215, (uint8_t)218, (uint8_t)217, (uint8_t)177, (uint8_t)63, (uint8_t)194, (uint8_t)16, (uint8_t)227, (uint8_t)116, (uint8_t)137, (uint8_t)88, (uint8_t)226, (uint8_t)174, (uint8_t)242, (uint8_t)39, (uint8_t)241, (uint8_t)62, (uint8_t)40, (uint8_t)228, (uint8_t)159, (uint8_t)179, (uint8_t)104, (uint8_t)201, (uint8_t)222, (uint8_t)164, (uint8_t)147, (uint8_t)183, (uint8_t)240, (uint8_t)45, (uint8_t)169, (uint8_t)195, (uint8_t)7, (uint8_t)164, (uint8_t)25, (uint8_t)163, (uint8_t)244, (uint8_t)90, (uint8_t)32, (uint8_t)164, (uint8_t)165, (uint8_t)6, (uint8_t)110, (uint8_t)93, (uint8_t)232, (uint8_t)90, (uint8_t)118, (uint8_t)71, (uint8_t)240, (uint8_t)33, (uint8_t)180, (uint8_t)53, (uint8_t)226, (uint8_t)107, (uint8_t)144, (uint8_t)164, (uint8_t)84, (uint8_t)100, (uint8_t)142, (uint8_t)134} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)194);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3491459089L);
    assert(p143_press_diff_GET(pack) == (float)1.9175905E38F);
    assert(p143_press_abs_GET(pack) == (float)2.7894392E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -21256);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-6.6749826E37F, -2.2842161E38F, -1.3761412E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -1729436600);
    {
        float exemplary[] =  {4.0510914E37F, 3.2270232E38F, -2.1487009E36F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)2959187900089460293L);
    assert(p144_alt_GET(pack) == (float) -1.0555975E38F);
    {
        float exemplary[] =  {-2.4172198E37F, -7.505534E37F, 1.6607099E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.3280743E38F, -3.0572748E37F, -4.236031E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.8645123E38F, -1.7121443E38F, -2.8425497E38F, -3.1939696E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p144_timestamp_GET(pack) == (uint64_t)5179437851203008076L);
    assert(p144_lon_GET(pack) == (int32_t) -230821665);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_y_pos_GET(pack) == (float) -2.2296485E38F);
    assert(p146_z_acc_GET(pack) == (float) -8.978267E37F);
    assert(p146_yaw_rate_GET(pack) == (float)1.7113772E38F);
    assert(p146_x_acc_GET(pack) == (float) -1.4291175E38F);
    assert(p146_z_vel_GET(pack) == (float) -1.0725178E38F);
    {
        float exemplary[] =  {-2.76961E38F, 2.5937857E38F, -2.0718735E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float) -2.3504583E38F);
    {
        float exemplary[] =  {-3.223272E36F, -2.9693133E38F, 1.1791592E38F, -2.1449784E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float)2.9858946E38F);
    assert(p146_z_pos_GET(pack) == (float) -2.1294031E38F);
    assert(p146_x_pos_GET(pack) == (float)1.0118824E38F);
    assert(p146_roll_rate_GET(pack) == (float)1.5158082E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.6377846E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)5232233134099404015L);
    assert(p146_x_vel_GET(pack) == (float)9.952708E37F);
    {
        float exemplary[] =  {1.4162652E38F, 1.0901792E38F, -2.0308732E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float) -6.0889386E37F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_current_consumed_GET(pack) == (int32_t) -616319523);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -29600);
    {
        uint16_t exemplary[] =  {(uint16_t)25608, (uint16_t)34307, (uint16_t)17935, (uint16_t)40849, (uint16_t)24566, (uint16_t)23190, (uint16_t)29393, (uint16_t)42848, (uint16_t)21341, (uint16_t)32006} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_energy_consumed_GET(pack) == (int32_t) -44783661);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -106);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)7324);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3336507060L);
    {
        uint8_t exemplary[] =  {(uint8_t)189, (uint8_t)105, (uint8_t)203, (uint8_t)46, (uint8_t)4, (uint8_t)210, (uint8_t)42, (uint8_t)68} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY));
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3453013498L);
    {
        uint8_t exemplary[] =  {(uint8_t)17, (uint8_t)92, (uint8_t)120, (uint8_t)159, (uint8_t)25, (uint8_t)146, (uint8_t)89, (uint8_t)207} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)14783);
    assert(p148_uid_GET(pack) == (uint64_t)8761676970266768747L);
    assert(p148_board_version_GET(pack) == (uint32_t)686299430L);
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)238, (uint8_t)125, (uint8_t)152, (uint8_t)131, (uint8_t)80, (uint8_t)64, (uint8_t)192, (uint8_t)71, (uint8_t)210, (uint8_t)57, (uint8_t)48, (uint8_t)34, (uint8_t)202, (uint8_t)150, (uint8_t)118, (uint8_t)141, (uint8_t)39} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)103, (uint8_t)234, (uint8_t)97, (uint8_t)255, (uint8_t)61, (uint8_t)49, (uint8_t)172, (uint8_t)182} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)3124264619L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)39221);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_size_x_GET(pack) == (float)2.0522669E38F);
    assert(p149_angle_x_GET(pack) == (float) -2.0980995E38F);
    assert(p149_angle_y_GET(pack) == (float) -2.1411052E37F);
    assert(p149_size_y_GET(pack) == (float) -3.1350679E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    {
        float exemplary[] =  {5.2795056E37F, -1.69894E37F, -2.577135E38F, -1.8414152E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_x_TRY(ph) == (float) -2.446685E37F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)137);
    assert(p149_distance_GET(pack) == (float) -5.8678786E37F);
    assert(p149_time_usec_GET(pack) == (uint64_t)4485267712437206597L);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p149_z_TRY(ph) == (float)2.7459586E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_y_TRY(ph) == (float) -1.8297983E38F);
};


void c_CommunicationChannel_on_SENS_POWER_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_adc121_cs2_amp_GET(pack) == (float)3.3988816E38F);
    assert(p201_adc121_vspb_volt_GET(pack) == (float) -2.6967977E38F);
    assert(p201_adc121_cspb_amp_GET(pack) == (float)1.0282093E38F);
    assert(p201_adc121_cs1_amp_GET(pack) == (float)1.0825496E38F);
};


void c_CommunicationChannel_on_SENS_MPPT_202(Bounds_Inside * ph, Pack * pack)
{
    assert(p202_mppt2_pwm_GET(pack) == (uint16_t)(uint16_t)25716);
    assert(p202_mppt3_amp_GET(pack) == (float) -1.8708063E38F);
    assert(p202_mppt3_status_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p202_mppt2_amp_GET(pack) == (float) -2.2082907E38F);
    assert(p202_mppt2_status_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p202_mppt3_pwm_GET(pack) == (uint16_t)(uint16_t)37452);
    assert(p202_mppt1_status_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p202_mppt1_amp_GET(pack) == (float) -1.1424517E38F);
    assert(p202_mppt1_volt_GET(pack) == (float)6.981507E37F);
    assert(p202_mppt2_volt_GET(pack) == (float)1.1836987E38F);
    assert(p202_mppt1_pwm_GET(pack) == (uint16_t)(uint16_t)39525);
    assert(p202_mppt3_volt_GET(pack) == (float)2.964E38F);
    assert(p202_mppt_timestamp_GET(pack) == (uint64_t)9190535815262464431L);
};


void c_CommunicationChannel_on_ASLCTRL_DATA_203(Bounds_Inside * ph, Pack * pack)
{
    assert(p203_RollAngle_GET(pack) == (float)3.1133615E37F);
    assert(p203_uThrot2_GET(pack) == (float)2.3287058E38F);
    assert(p203_uThrot_GET(pack) == (float)1.5428256E38F);
    assert(p203_hRef_t_GET(pack) == (float)2.1744495E38F);
    assert(p203_q_GET(pack) == (float) -2.960124E38F);
    assert(p203_RollAngleRef_GET(pack) == (float) -5.140219E37F);
    assert(p203_rRef_GET(pack) == (float)1.0298218E38F);
    assert(p203_PitchAngle_GET(pack) == (float)2.1484739E38F);
    assert(p203_r_GET(pack) == (float) -1.4112538E38F);
    assert(p203_h_GET(pack) == (float) -1.6295831E38F);
    assert(p203_hRef_GET(pack) == (float)2.3183382E37F);
    assert(p203_aslctrl_mode_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p203_uAil_GET(pack) == (float)3.1332264E38F);
    assert(p203_PitchAngleRef_GET(pack) == (float)3.3580436E38F);
    assert(p203_YawAngle_GET(pack) == (float) -1.0809843E38F);
    assert(p203_SpoilersEngaged_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p203_nZ_GET(pack) == (float) -2.099024E37F);
    assert(p203_pRef_GET(pack) == (float)1.2630461E38F);
    assert(p203_uElev_GET(pack) == (float) -9.325526E37F);
    assert(p203_AirspeedRef_GET(pack) == (float)9.083275E37F);
    assert(p203_qRef_GET(pack) == (float)1.6768969E38F);
    assert(p203_uRud_GET(pack) == (float) -2.3189929E37F);
    assert(p203_timestamp_GET(pack) == (uint64_t)4491710795493874348L);
    assert(p203_YawAngleRef_GET(pack) == (float)3.3649002E38F);
    assert(p203_p_GET(pack) == (float) -1.3254494E38F);
};


void c_CommunicationChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * ph, Pack * pack)
{
    assert(p204_f_8_GET(pack) == (float)1.0079533E38F);
    assert(p204_f_6_GET(pack) == (float)2.3557244E38F);
    assert(p204_f_3_GET(pack) == (float) -2.8096646E38F);
    assert(p204_i8_2_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p204_f_5_GET(pack) == (float)7.4846234E37F);
    assert(p204_i8_1_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p204_f_1_GET(pack) == (float)3.3193883E38F);
    assert(p204_f_4_GET(pack) == (float)2.111506E38F);
    assert(p204_f_7_GET(pack) == (float) -1.7406388E38F);
    assert(p204_i32_1_GET(pack) == (uint32_t)412052952L);
    assert(p204_f_2_GET(pack) == (float)2.1151032E38F);
};


void c_CommunicationChannel_on_ASLUAV_STATUS_205(Bounds_Inside * ph, Pack * pack)
{
    assert(p205_LED_status_GET(pack) == (uint8_t)(uint8_t)245);
    {
        uint8_t exemplary[] =  {(uint8_t)134, (uint8_t)185, (uint8_t)108, (uint8_t)15, (uint8_t)188, (uint8_t)117, (uint8_t)38, (uint8_t)106} ;
        uint8_t*  sample = p205_Servo_status_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p205_SATCOM_status_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p205_Motor_rpm_GET(pack) == (float)2.5038503E38F);
};


void c_CommunicationChannel_on_EKF_EXT_206(Bounds_Inside * ph, Pack * pack)
{
    assert(p206_Airspeed_GET(pack) == (float)3.095344E37F);
    assert(p206_beta_GET(pack) == (float) -1.1307029E38F);
    assert(p206_alpha_GET(pack) == (float)2.1177805E38F);
    assert(p206_WindDir_GET(pack) == (float) -1.0262408E38F);
    assert(p206_timestamp_GET(pack) == (uint64_t)2684332372044194070L);
    assert(p206_Windspeed_GET(pack) == (float)1.653101E38F);
    assert(p206_WindZ_GET(pack) == (float) -2.5826677E38F);
};


void c_CommunicationChannel_on_ASL_OBCTRL_207(Bounds_Inside * ph, Pack * pack)
{
    assert(p207_uAilL_GET(pack) == (float) -2.8655869E38F);
    assert(p207_timestamp_GET(pack) == (uint64_t)2557465763835125436L);
    assert(p207_uThrot_GET(pack) == (float)2.6145963E38F);
    assert(p207_uThrot2_GET(pack) == (float) -3.2844178E38F);
    assert(p207_uAilR_GET(pack) == (float)3.031416E38F);
    assert(p207_uElev_GET(pack) == (float)2.5518677E38F);
    assert(p207_obctrl_status_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p207_uRud_GET(pack) == (float) -1.1719743E38F);
};


void c_CommunicationChannel_on_SENS_ATMOS_208(Bounds_Inside * ph, Pack * pack)
{
    assert(p208_TempAmbient_GET(pack) == (float) -2.4332206E38F);
    assert(p208_Humidity_GET(pack) == (float) -2.661444E38F);
};


void c_CommunicationChannel_on_SENS_BATMON_209(Bounds_Inside * ph, Pack * pack)
{
    assert(p209_cellvoltage4_GET(pack) == (uint16_t)(uint16_t)47642);
    assert(p209_temperature_GET(pack) == (float) -2.4388914E38F);
    assert(p209_cellvoltage3_GET(pack) == (uint16_t)(uint16_t)42733);
    assert(p209_serialnumber_GET(pack) == (uint16_t)(uint16_t)40260);
    assert(p209_SoC_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p209_current_GET(pack) == (int16_t)(int16_t)23345);
    assert(p209_cellvoltage6_GET(pack) == (uint16_t)(uint16_t)29344);
    assert(p209_batterystatus_GET(pack) == (uint16_t)(uint16_t)11940);
    assert(p209_cellvoltage5_GET(pack) == (uint16_t)(uint16_t)36948);
    assert(p209_voltage_GET(pack) == (uint16_t)(uint16_t)34228);
    assert(p209_hostfetcontrol_GET(pack) == (uint16_t)(uint16_t)53224);
    assert(p209_cellvoltage2_GET(pack) == (uint16_t)(uint16_t)26793);
    assert(p209_cellvoltage1_GET(pack) == (uint16_t)(uint16_t)55579);
};


void c_CommunicationChannel_on_FW_SOARING_DATA_210(Bounds_Inside * ph, Pack * pack)
{
    assert(p210_z2_DeltaRoll_GET(pack) == (float)3.8276794E37F);
    assert(p210_xLat_GET(pack) == (float)2.644896E38F);
    assert(p210_valid_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p210_ControlMode_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p210_LoiterDirection_GET(pack) == (float) -1.4252063E38F);
    assert(p210_z1_exp_GET(pack) == (float)3.0498718E38F);
    assert(p210_z1_LocalUpdraftSpeed_GET(pack) == (float) -3.2846594E38F);
    assert(p210_vSinkExp_GET(pack) == (float)3.1512313E38F);
    assert(p210_z2_exp_GET(pack) == (float) -5.518926E37F);
    assert(p210_xW_GET(pack) == (float)2.204834E38F);
    assert(p210_DebugVar1_GET(pack) == (float)4.788306E37F);
    assert(p210_DebugVar2_GET(pack) == (float)2.6787867E38F);
    assert(p210_VarLat_GET(pack) == (float) -1.0889321E35F);
    assert(p210_ThermalGSNorth_GET(pack) == (float) -5.55634E37F);
    assert(p210_timestamp_GET(pack) == (uint64_t)2773514599753803499L);
    assert(p210_VarR_GET(pack) == (float) -2.6221537E38F);
    assert(p210_VarW_GET(pack) == (float) -2.2712289E38F);
    assert(p210_DistToSoarPoint_GET(pack) == (float)2.653032E38F);
    assert(p210_xR_GET(pack) == (float) -2.1299524E38F);
    assert(p210_VarLon_GET(pack) == (float)3.548662E37F);
    assert(p210_timestampModeChanged_GET(pack) == (uint64_t)2607393097584597889L);
    assert(p210_ThermalGSEast_GET(pack) == (float) -2.6215055E37F);
    assert(p210_TSE_dot_GET(pack) == (float) -1.8207186E38F);
    assert(p210_LoiterRadius_GET(pack) == (float) -2.5514837E38F);
    assert(p210_xLon_GET(pack) == (float) -4.4732986E37F);
};


void c_CommunicationChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * ph, Pack * pack)
{
    assert(p211_cpu_temp_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p211_visensor_rate_3_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p211_timestamp_GET(pack) == (uint64_t)6059200872289593965L);
    assert(p211_recording_nodes_count_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p211_visensor_rate_4_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p211_free_space_GET(pack) == (uint16_t)(uint16_t)29073);
    assert(p211_visensor_rate_1_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p211_visensor_rate_2_GET(pack) == (uint8_t)(uint8_t)165);
};


void c_CommunicationChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * ph, Pack * pack)
{
    assert(p212_timestamp_GET(pack) == (uint64_t)4093727713564333L);
    assert(p212_pwr_brd_mot_r_amp_GET(pack) == (float) -1.802026E38F);
    assert(p212_pwr_brd_system_volt_GET(pack) == (float)3.3753335E38F);
    assert(p212_pwr_brd_servo_3_amp_GET(pack) == (float) -1.4919936E38F);
    assert(p212_pwr_brd_servo_4_amp_GET(pack) == (float) -3.0171104E38F);
    assert(p212_pwr_brd_servo_volt_GET(pack) == (float)1.35046E38F);
    assert(p212_pwr_brd_servo_2_amp_GET(pack) == (float) -8.713115E37F);
    assert(p212_pwr_brd_led_status_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p212_pwr_brd_status_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p212_pwr_brd_mot_l_amp_GET(pack) == (float) -1.9959605E38F);
    assert(p212_pwr_brd_aux_amp_GET(pack) == (float) -4.999581E37F);
    assert(p212_pwr_brd_servo_1_amp_GET(pack) == (float)1.7011138E38F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -2.3975277E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -1.4829601E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)3.0636748E38F);
    assert(p230_tas_ratio_GET(pack) == (float)2.9261096E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)2.7059195E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.4785463E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)1.7424616E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.0491469E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)2524065423633256584L);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS));
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_time_usec_GET(pack) == (uint64_t)7769729986774889362L);
    assert(p231_wind_y_GET(pack) == (float)4.0044809E37F);
    assert(p231_vert_accuracy_GET(pack) == (float)1.8539617E38F);
    assert(p231_wind_z_GET(pack) == (float)1.5258766E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)2.1901777E38F);
    assert(p231_wind_x_GET(pack) == (float)2.6094592E38F);
    assert(p231_var_horiz_GET(pack) == (float)9.7859704E36F);
    assert(p231_wind_alt_GET(pack) == (float) -3.2494158E38F);
    assert(p231_var_vert_GET(pack) == (float) -2.5857267E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)29581);
    assert(p232_time_usec_GET(pack) == (uint64_t)8294112524983618193L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p232_vert_accuracy_GET(pack) == (float)2.436052E38F);
    assert(p232_ve_GET(pack) == (float) -9.156242E37F);
    assert(p232_vn_GET(pack) == (float) -1.30638E37F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p232_alt_GET(pack) == (float)1.5771093E38F);
    assert(p232_vdop_GET(pack) == (float) -1.961859E38F);
    assert(p232_lat_GET(pack) == (int32_t)1358345590);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)3563689822L);
    assert(p232_speed_accuracy_GET(pack) == (float) -3.1363511E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -3.8020372E36F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p232_hdop_GET(pack) == (float)3.9833613E37F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT));
    assert(p232_vd_GET(pack) == (float) -2.1109603E38F);
    assert(p232_lon_GET(pack) == (int32_t)414490353);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)37);
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)134, (uint8_t)165, (uint8_t)129, (uint8_t)28, (uint8_t)95, (uint8_t)247, (uint8_t)47, (uint8_t)238, (uint8_t)173, (uint8_t)149, (uint8_t)179, (uint8_t)187, (uint8_t)101, (uint8_t)163, (uint8_t)192, (uint8_t)218, (uint8_t)104, (uint8_t)154, (uint8_t)242, (uint8_t)164, (uint8_t)211, (uint8_t)68, (uint8_t)93, (uint8_t)46, (uint8_t)112, (uint8_t)216, (uint8_t)118, (uint8_t)233, (uint8_t)249, (uint8_t)219, (uint8_t)91, (uint8_t)247, (uint8_t)87, (uint8_t)80, (uint8_t)94, (uint8_t)225, (uint8_t)6, (uint8_t)35, (uint8_t)98, (uint8_t)97, (uint8_t)39, (uint8_t)111, (uint8_t)68, (uint8_t)180, (uint8_t)240, (uint8_t)2, (uint8_t)246, (uint8_t)154, (uint8_t)2, (uint8_t)27, (uint8_t)252, (uint8_t)97, (uint8_t)121, (uint8_t)161, (uint8_t)98, (uint8_t)175, (uint8_t)230, (uint8_t)187, (uint8_t)144, (uint8_t)89, (uint8_t)193, (uint8_t)41, (uint8_t)186, (uint8_t)74, (uint8_t)98, (uint8_t)163, (uint8_t)240, (uint8_t)142, (uint8_t)186, (uint8_t)220, (uint8_t)178, (uint8_t)42, (uint8_t)198, (uint8_t)57, (uint8_t)111, (uint8_t)80, (uint8_t)14, (uint8_t)219, (uint8_t)29, (uint8_t)228, (uint8_t)210, (uint8_t)157, (uint8_t)126, (uint8_t)220, (uint8_t)177, (uint8_t)29, (uint8_t)59, (uint8_t)221, (uint8_t)190, (uint8_t)214, (uint8_t)146, (uint8_t)16, (uint8_t)68, (uint8_t)182, (uint8_t)140, (uint8_t)61, (uint8_t)76, (uint8_t)203, (uint8_t)221, (uint8_t)8, (uint8_t)248, (uint8_t)191, (uint8_t)11, (uint8_t)137, (uint8_t)60, (uint8_t)179, (uint8_t)21, (uint8_t)153, (uint8_t)100, (uint8_t)15, (uint8_t)211, (uint8_t)82, (uint8_t)25, (uint8_t)35, (uint8_t)22, (uint8_t)38, (uint8_t)97, (uint8_t)218, (uint8_t)234, (uint8_t)98, (uint8_t)18, (uint8_t)202, (uint8_t)60, (uint8_t)183, (uint8_t)205, (uint8_t)99, (uint8_t)48, (uint8_t)74, (uint8_t)71, (uint8_t)0, (uint8_t)225, (uint8_t)125, (uint8_t)154, (uint8_t)90, (uint8_t)217, (uint8_t)171, (uint8_t)192, (uint8_t)173, (uint8_t)126, (uint8_t)246, (uint8_t)68, (uint8_t)127, (uint8_t)70, (uint8_t)133, (uint8_t)209, (uint8_t)33, (uint8_t)180, (uint8_t)111, (uint8_t)225, (uint8_t)142, (uint8_t)38, (uint8_t)129, (uint8_t)173, (uint8_t)192, (uint8_t)154, (uint8_t)154, (uint8_t)79, (uint8_t)68, (uint8_t)185, (uint8_t)133, (uint8_t)31, (uint8_t)213, (uint8_t)212, (uint8_t)9, (uint8_t)119, (uint8_t)93, (uint8_t)149, (uint8_t)36, (uint8_t)157, (uint8_t)144, (uint8_t)86, (uint8_t)6, (uint8_t)217, (uint8_t)116, (uint8_t)11, (uint8_t)93, (uint8_t)24, (uint8_t)152, (uint8_t)77} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -3129);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -30417);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)25568);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED));
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p234_longitude_GET(pack) == (int32_t)1208554210);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -47);
    assert(p234_custom_mode_GET(pack) == (uint32_t)171625830L);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)32711);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p234_latitude_GET(pack) == (int32_t)385162917);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -30000);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)9);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)45714);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)52);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)63);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)49498);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_x_GET(pack) == (float) -1.9628302E38F);
    assert(p241_vibration_z_GET(pack) == (float) -3.0472197E38F);
    assert(p241_vibration_y_GET(pack) == (float)5.671594E37F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3966562888L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)1666651833L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2740700348L);
    assert(p241_time_usec_GET(pack) == (uint64_t)1300520494146345422L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_altitude_GET(pack) == (int32_t) -649148916);
    {
        float exemplary[] =  {2.0004384E38F, 4.478343E37F, 3.3484204E38F, -2.0006607E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t)906994020);
    assert(p242_approach_y_GET(pack) == (float)2.9124589E38F);
    assert(p242_z_GET(pack) == (float)3.4000878E38F);
    assert(p242_x_GET(pack) == (float) -2.3406479E38F);
    assert(p242_approach_x_GET(pack) == (float) -3.2342199E38F);
    assert(p242_latitude_GET(pack) == (int32_t)1381063527);
    assert(p242_approach_z_GET(pack) == (float) -1.748249E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)6802636397191708908L);
    assert(p242_y_GET(pack) == (float) -1.521178E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_z_GET(pack) == (float)1.0197115E38F);
    assert(p243_approach_x_GET(pack) == (float)1.1935076E37F);
    assert(p243_z_GET(pack) == (float) -2.5981996E38F);
    assert(p243_x_GET(pack) == (float) -2.1744915E38F);
    assert(p243_time_usec_TRY(ph) == (uint64_t)4251498827834386883L);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p243_y_GET(pack) == (float) -2.4765933E37F);
    assert(p243_altitude_GET(pack) == (int32_t)676404980);
    assert(p243_latitude_GET(pack) == (int32_t) -1179617773);
    assert(p243_longitude_GET(pack) == (int32_t) -878627601);
    {
        float exemplary[] =  {-1.0148291E38F, -5.1825223E37F, -2.7102761E38F, 2.4851288E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_y_GET(pack) == (float)3.0804688E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -1623233171);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)24757);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p246_altitude_GET(pack) == (int32_t)287418753);
    assert(p246_lat_GET(pack) == (int32_t)779559442);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)45907);
    assert(p246_lon_GET(pack) == (int32_t)317354840);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)1066831238L);
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"Ejgfr";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY));
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)14413);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)1099);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)29737);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)4.577144E37F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -6.353918E37F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE);
    assert(p247_id_GET(pack) == (uint32_t)71641241L);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -3.2725623E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)233, (uint8_t)207, (uint8_t)34, (uint8_t)207, (uint8_t)199, (uint8_t)17, (uint8_t)158, (uint8_t)50, (uint8_t)86, (uint8_t)244, (uint8_t)152, (uint8_t)100, (uint8_t)67, (uint8_t)1, (uint8_t)91, (uint8_t)113, (uint8_t)23, (uint8_t)199, (uint8_t)49, (uint8_t)182, (uint8_t)247, (uint8_t)243, (uint8_t)152, (uint8_t)194, (uint8_t)65, (uint8_t)153, (uint8_t)178, (uint8_t)142, (uint8_t)46, (uint8_t)38, (uint8_t)252, (uint8_t)163, (uint8_t)3, (uint8_t)97, (uint8_t)210, (uint8_t)200, (uint8_t)12, (uint8_t)232, (uint8_t)205, (uint8_t)225, (uint8_t)38, (uint8_t)183, (uint8_t)247, (uint8_t)85, (uint8_t)218, (uint8_t)211, (uint8_t)65, (uint8_t)245, (uint8_t)77, (uint8_t)71, (uint8_t)76, (uint8_t)70, (uint8_t)44, (uint8_t)88, (uint8_t)115, (uint8_t)202, (uint8_t)220, (uint8_t)116, (uint8_t)54, (uint8_t)153, (uint8_t)10, (uint8_t)148, (uint8_t)224, (uint8_t)127, (uint8_t)55, (uint8_t)1, (uint8_t)67, (uint8_t)204, (uint8_t)119, (uint8_t)130, (uint8_t)92, (uint8_t)219, (uint8_t)228, (uint8_t)199, (uint8_t)96, (uint8_t)110, (uint8_t)192, (uint8_t)240, (uint8_t)97, (uint8_t)79, (uint8_t)169, (uint8_t)245, (uint8_t)15, (uint8_t)58, (uint8_t)39, (uint8_t)229, (uint8_t)79, (uint8_t)145, (uint8_t)117, (uint8_t)204, (uint8_t)54, (uint8_t)80, (uint8_t)168, (uint8_t)179, (uint8_t)12, (uint8_t)76, (uint8_t)238, (uint8_t)47, (uint8_t)186, (uint8_t)155, (uint8_t)249, (uint8_t)80, (uint8_t)68, (uint8_t)97, (uint8_t)228, (uint8_t)155, (uint8_t)171, (uint8_t)5, (uint8_t)205, (uint8_t)14, (uint8_t)226, (uint8_t)196, (uint8_t)134, (uint8_t)103, (uint8_t)199, (uint8_t)34, (uint8_t)98, (uint8_t)79, (uint8_t)65, (uint8_t)164, (uint8_t)182, (uint8_t)89, (uint8_t)194, (uint8_t)104, (uint8_t)194, (uint8_t)193, (uint8_t)81, (uint8_t)228, (uint8_t)135, (uint8_t)206, (uint8_t)37, (uint8_t)134, (uint8_t)110, (uint8_t)231, (uint8_t)73, (uint8_t)178, (uint8_t)228, (uint8_t)111, (uint8_t)136, (uint8_t)102, (uint8_t)81, (uint8_t)112, (uint8_t)128, (uint8_t)129, (uint8_t)247, (uint8_t)134, (uint8_t)87, (uint8_t)220, (uint8_t)116, (uint8_t)28, (uint8_t)15, (uint8_t)92, (uint8_t)64, (uint8_t)105, (uint8_t)71, (uint8_t)105, (uint8_t)234, (uint8_t)73, (uint8_t)225, (uint8_t)73, (uint8_t)201, (uint8_t)42, (uint8_t)1, (uint8_t)92, (uint8_t)118, (uint8_t)168, (uint8_t)203, (uint8_t)180, (uint8_t)95, (uint8_t)30, (uint8_t)171, (uint8_t)50, (uint8_t)62, (uint8_t)132, (uint8_t)184, (uint8_t)196, (uint8_t)60, (uint8_t)204, (uint8_t)32, (uint8_t)123, (uint8_t)189, (uint8_t)97, (uint8_t)200, (uint8_t)97, (uint8_t)14, (uint8_t)237, (uint8_t)11, (uint8_t)174, (uint8_t)25, (uint8_t)255, (uint8_t)139, (uint8_t)83, (uint8_t)205, (uint8_t)213, (uint8_t)29, (uint8_t)93, (uint8_t)135, (uint8_t)152, (uint8_t)225, (uint8_t)181, (uint8_t)247, (uint8_t)70, (uint8_t)220, (uint8_t)65, (uint8_t)78, (uint8_t)86, (uint8_t)165, (uint8_t)73, (uint8_t)230, (uint8_t)51, (uint8_t)81, (uint8_t)99, (uint8_t)41, (uint8_t)106, (uint8_t)111, (uint8_t)182, (uint8_t)26, (uint8_t)250, (uint8_t)72, (uint8_t)153, (uint8_t)122, (uint8_t)157, (uint8_t)153, (uint8_t)129, (uint8_t)151, (uint8_t)165, (uint8_t)107, (uint8_t)158, (uint8_t)177, (uint8_t)179, (uint8_t)186, (uint8_t)31, (uint8_t)41, (uint8_t)231, (uint8_t)127, (uint8_t)37, (uint8_t)101, (uint8_t)250, (uint8_t)194, (uint8_t)57, (uint8_t)130, (uint8_t)57, (uint8_t)224, (uint8_t)77, (uint8_t)62, (uint8_t)185, (uint8_t)12, (uint8_t)100, (uint8_t)37} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)42892);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)89);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)32398);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)29);
    {
        int8_t exemplary[] =  {(int8_t) -88, (int8_t)121, (int8_t) -24, (int8_t) -43, (int8_t) -8, (int8_t)123, (int8_t) -63, (int8_t)37, (int8_t) -7, (int8_t) -112, (int8_t) -26, (int8_t)52, (int8_t)125, (int8_t)36, (int8_t)4, (int8_t)85, (int8_t) -117, (int8_t) -83, (int8_t)110, (int8_t) -104, (int8_t) -47, (int8_t) -78, (int8_t)14, (int8_t)97, (int8_t)68, (int8_t)43, (int8_t) -56, (int8_t)36, (int8_t)112, (int8_t)38, (int8_t) -25, (int8_t)97} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)20);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_time_usec_GET(pack) == (uint64_t)6123168229078066334L);
    assert(p250_x_GET(pack) == (float)3.1708004E38F);
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"Tallz";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_z_GET(pack) == (float) -2.6506793E38F);
    assert(p250_y_GET(pack) == (float)1.0010957E37F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2226984583L);
    assert(p251_value_GET(pack) == (float)8.930372E37F);
    assert(p251_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"cbzg";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t)2069173558);
    assert(p252_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"t";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3808247146L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 25);
    {
        char16_t * exemplary = u"zvGhxsmAqRmfhrxzePEuffcjf";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ERROR);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p254_value_GET(pack) == (float)1.2237344E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3538000375L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)64, (uint8_t)70, (uint8_t)239, (uint8_t)86, (uint8_t)251, (uint8_t)166, (uint8_t)132, (uint8_t)78, (uint8_t)6, (uint8_t)204, (uint8_t)12, (uint8_t)122, (uint8_t)57, (uint8_t)71, (uint8_t)147, (uint8_t)101, (uint8_t)72, (uint8_t)187, (uint8_t)208, (uint8_t)227, (uint8_t)167, (uint8_t)67, (uint8_t)65, (uint8_t)210, (uint8_t)191, (uint8_t)25, (uint8_t)149, (uint8_t)189, (uint8_t)238, (uint8_t)113, (uint8_t)254} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)5725248189427489857L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3766544226L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)568994889L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p258_tune_LEN(ph) == 10);
    {
        char16_t * exemplary = u"tQmhqvyXcm";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_h_GET(pack) == (float) -8.637447E37F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)34053);
    assert(p259_sensor_size_v_GET(pack) == (float)2.5127249E38F);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3959237267L);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)49422);
    assert(p259_firmware_version_GET(pack) == (uint32_t)735082356L);
    {
        uint8_t exemplary[] =  {(uint8_t)238, (uint8_t)203, (uint8_t)124, (uint8_t)165, (uint8_t)131, (uint8_t)31, (uint8_t)2, (uint8_t)235, (uint8_t)252, (uint8_t)211, (uint8_t)187, (uint8_t)112, (uint8_t)22, (uint8_t)133, (uint8_t)57, (uint8_t)60, (uint8_t)209, (uint8_t)123, (uint8_t)24, (uint8_t)56, (uint8_t)26, (uint8_t)186, (uint8_t)57, (uint8_t)116, (uint8_t)24, (uint8_t)248, (uint8_t)131, (uint8_t)176, (uint8_t)8, (uint8_t)241, (uint8_t)192, (uint8_t)149} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float) -1.5111609E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)39, (uint8_t)125, (uint8_t)188, (uint8_t)23, (uint8_t)95, (uint8_t)132, (uint8_t)197, (uint8_t)172, (uint8_t)106, (uint8_t)131, (uint8_t)80, (uint8_t)11, (uint8_t)16, (uint8_t)110, (uint8_t)218, (uint8_t)156, (uint8_t)188, (uint8_t)209, (uint8_t)59, (uint8_t)108, (uint8_t)188, (uint8_t)244, (uint8_t)59, (uint8_t)163, (uint8_t)254, (uint8_t)209, (uint8_t)85, (uint8_t)104, (uint8_t)146, (uint8_t)252, (uint8_t)71, (uint8_t)229} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)56177);
    assert(p259_cam_definition_uri_LEN(ph) == 122);
    {
        char16_t * exemplary = u"pgywitvbnysluZcoacmqlcnsWnhagjwmtpcdofczileqjlfyujbhianwxzaNcdyxiaarqqtcvhkghsuetsdfocfJrvtdJlDzyLfHDsrmuwtxtmvrshpygyuwmo";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 244);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3097219262L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_total_capacity_GET(pack) == (float)2.710221E37F);
    assert(p261_used_capacity_GET(pack) == (float) -3.2368923E38F);
    assert(p261_available_capacity_GET(pack) == (float) -2.747017E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p261_write_speed_GET(pack) == (float) -8.0691736E37F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p261_read_speed_GET(pack) == (float) -1.2033737E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1765955992L);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p262_available_capacity_GET(pack) == (float) -1.616006E38F);
    assert(p262_image_interval_GET(pack) == (float) -5.2828685E37F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1143610562L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3524219870L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_image_index_GET(pack) == (int32_t)1673363567);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p263_lat_GET(pack) == (int32_t) -1802123384);
    assert(p263_file_url_LEN(ph) == 189);
    {
        char16_t * exemplary = u"XloooGbhamimetWxQnctJyrMgwcseksxksokvZzptoipoolvgtrvuyshdcyvtzdlryyzdztjssxufvoflzOaanbgrmExxVfrykwezqxskzxugqbdrmlcolrdsmsbfjAmuoocrcbfrcmbahuyzaqwayykqzdpnadaUlbhcwagRdhqihgqytksvyzecohub";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 378);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t) -780236504);
    assert(p263_relative_alt_GET(pack) == (int32_t) -80975482);
    assert(p263_time_utc_GET(pack) == (uint64_t)1108674768072043193L);
    assert(p263_lon_GET(pack) == (int32_t) -1452102215);
    {
        float exemplary[] =  {9.193425E37F, 1.0792307E38F, 4.771927E37F, 9.627906E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2382079625L);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -125);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)133502443L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)1470937663765612509L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7097306137198587484L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)3612401160507255168L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2072776420L);
    assert(p265_roll_GET(pack) == (float)2.2092197E38F);
    assert(p265_pitch_GET(pack) == (float) -4.666561E37F);
    assert(p265_yaw_GET(pack) == (float)1.3293881E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)57434);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)119);
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)8, (uint8_t)246, (uint8_t)49, (uint8_t)171, (uint8_t)224, (uint8_t)251, (uint8_t)221, (uint8_t)21, (uint8_t)30, (uint8_t)52, (uint8_t)54, (uint8_t)199, (uint8_t)88, (uint8_t)100, (uint8_t)165, (uint8_t)10, (uint8_t)194, (uint8_t)180, (uint8_t)108, (uint8_t)86, (uint8_t)117, (uint8_t)189, (uint8_t)40, (uint8_t)154, (uint8_t)78, (uint8_t)170, (uint8_t)230, (uint8_t)209, (uint8_t)41, (uint8_t)84, (uint8_t)63, (uint8_t)120, (uint8_t)133, (uint8_t)151, (uint8_t)247, (uint8_t)82, (uint8_t)209, (uint8_t)245, (uint8_t)157, (uint8_t)182, (uint8_t)230, (uint8_t)97, (uint8_t)77, (uint8_t)172, (uint8_t)84, (uint8_t)184, (uint8_t)117, (uint8_t)27, (uint8_t)136, (uint8_t)113, (uint8_t)101, (uint8_t)32, (uint8_t)243, (uint8_t)42, (uint8_t)202, (uint8_t)65, (uint8_t)74, (uint8_t)88, (uint8_t)115, (uint8_t)128, (uint8_t)122, (uint8_t)68, (uint8_t)126, (uint8_t)238, (uint8_t)237, (uint8_t)20, (uint8_t)85, (uint8_t)52, (uint8_t)5, (uint8_t)161, (uint8_t)78, (uint8_t)104, (uint8_t)94, (uint8_t)61, (uint8_t)196, (uint8_t)193, (uint8_t)248, (uint8_t)71, (uint8_t)11, (uint8_t)206, (uint8_t)152, (uint8_t)181, (uint8_t)45, (uint8_t)118, (uint8_t)56, (uint8_t)124, (uint8_t)210, (uint8_t)148, (uint8_t)63, (uint8_t)104, (uint8_t)180, (uint8_t)37, (uint8_t)107, (uint8_t)201, (uint8_t)164, (uint8_t)51, (uint8_t)33, (uint8_t)193, (uint8_t)86, (uint8_t)177, (uint8_t)21, (uint8_t)98, (uint8_t)186, (uint8_t)227, (uint8_t)179, (uint8_t)41, (uint8_t)220, (uint8_t)101, (uint8_t)96, (uint8_t)204, (uint8_t)155, (uint8_t)95, (uint8_t)122, (uint8_t)5, (uint8_t)44, (uint8_t)101, (uint8_t)47, (uint8_t)240, (uint8_t)185, (uint8_t)192, (uint8_t)208, (uint8_t)186, (uint8_t)220, (uint8_t)114, (uint8_t)157, (uint8_t)200, (uint8_t)230, (uint8_t)83, (uint8_t)102, (uint8_t)132, (uint8_t)37, (uint8_t)1, (uint8_t)82, (uint8_t)253, (uint8_t)154, (uint8_t)42, (uint8_t)21, (uint8_t)212, (uint8_t)107, (uint8_t)56, (uint8_t)232, (uint8_t)129, (uint8_t)5, (uint8_t)43, (uint8_t)238, (uint8_t)190, (uint8_t)39, (uint8_t)192, (uint8_t)211, (uint8_t)11, (uint8_t)70, (uint8_t)133, (uint8_t)203, (uint8_t)193, (uint8_t)217, (uint8_t)82, (uint8_t)218, (uint8_t)158, (uint8_t)41, (uint8_t)9, (uint8_t)48, (uint8_t)116, (uint8_t)222, (uint8_t)167, (uint8_t)45, (uint8_t)179, (uint8_t)30, (uint8_t)101, (uint8_t)37, (uint8_t)242, (uint8_t)159, (uint8_t)232, (uint8_t)186, (uint8_t)86, (uint8_t)102, (uint8_t)146, (uint8_t)86, (uint8_t)214, (uint8_t)129, (uint8_t)227, (uint8_t)139, (uint8_t)134, (uint8_t)27, (uint8_t)166, (uint8_t)174, (uint8_t)121, (uint8_t)61, (uint8_t)180, (uint8_t)166, (uint8_t)56, (uint8_t)26, (uint8_t)73, (uint8_t)75, (uint8_t)18, (uint8_t)5, (uint8_t)244, (uint8_t)77, (uint8_t)82, (uint8_t)157, (uint8_t)43, (uint8_t)193, (uint8_t)222, (uint8_t)84, (uint8_t)92, (uint8_t)215, (uint8_t)237, (uint8_t)180, (uint8_t)94, (uint8_t)110, (uint8_t)8, (uint8_t)29, (uint8_t)238, (uint8_t)21, (uint8_t)231, (uint8_t)190, (uint8_t)112, (uint8_t)28, (uint8_t)201, (uint8_t)57, (uint8_t)28, (uint8_t)70, (uint8_t)11, (uint8_t)223, (uint8_t)174, (uint8_t)28, (uint8_t)84, (uint8_t)169, (uint8_t)0, (uint8_t)120, (uint8_t)130, (uint8_t)238, (uint8_t)168, (uint8_t)1, (uint8_t)21, (uint8_t)68, (uint8_t)181, (uint8_t)109, (uint8_t)196, (uint8_t)193, (uint8_t)67, (uint8_t)12, (uint8_t)231, (uint8_t)50, (uint8_t)91, (uint8_t)63, (uint8_t)245, (uint8_t)147, (uint8_t)148} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)30);
    {
        uint8_t exemplary[] =  {(uint8_t)220, (uint8_t)209, (uint8_t)58, (uint8_t)185, (uint8_t)224, (uint8_t)253, (uint8_t)184, (uint8_t)52, (uint8_t)245, (uint8_t)168, (uint8_t)215, (uint8_t)234, (uint8_t)85, (uint8_t)200, (uint8_t)10, (uint8_t)213, (uint8_t)119, (uint8_t)194, (uint8_t)153, (uint8_t)214, (uint8_t)44, (uint8_t)124, (uint8_t)233, (uint8_t)76, (uint8_t)152, (uint8_t)211, (uint8_t)120, (uint8_t)73, (uint8_t)225, (uint8_t)252, (uint8_t)206, (uint8_t)9, (uint8_t)112, (uint8_t)146, (uint8_t)86, (uint8_t)7, (uint8_t)106, (uint8_t)11, (uint8_t)208, (uint8_t)64, (uint8_t)135, (uint8_t)116, (uint8_t)137, (uint8_t)51, (uint8_t)64, (uint8_t)251, (uint8_t)79, (uint8_t)207, (uint8_t)73, (uint8_t)239, (uint8_t)153, (uint8_t)26, (uint8_t)175, (uint8_t)252, (uint8_t)55, (uint8_t)61, (uint8_t)164, (uint8_t)62, (uint8_t)55, (uint8_t)1, (uint8_t)72, (uint8_t)16, (uint8_t)165, (uint8_t)251, (uint8_t)59, (uint8_t)230, (uint8_t)134, (uint8_t)33, (uint8_t)215, (uint8_t)130, (uint8_t)159, (uint8_t)180, (uint8_t)180, (uint8_t)144, (uint8_t)8, (uint8_t)86, (uint8_t)147, (uint8_t)201, (uint8_t)235, (uint8_t)141, (uint8_t)133, (uint8_t)166, (uint8_t)251, (uint8_t)2, (uint8_t)239, (uint8_t)3, (uint8_t)251, (uint8_t)178, (uint8_t)121, (uint8_t)55, (uint8_t)37, (uint8_t)3, (uint8_t)224, (uint8_t)22, (uint8_t)240, (uint8_t)226, (uint8_t)31, (uint8_t)42, (uint8_t)30, (uint8_t)188, (uint8_t)127, (uint8_t)76, (uint8_t)29, (uint8_t)47, (uint8_t)142, (uint8_t)0, (uint8_t)112, (uint8_t)154, (uint8_t)109, (uint8_t)42, (uint8_t)151, (uint8_t)174, (uint8_t)255, (uint8_t)77, (uint8_t)95, (uint8_t)66, (uint8_t)69, (uint8_t)181, (uint8_t)219, (uint8_t)150, (uint8_t)133, (uint8_t)11, (uint8_t)37, (uint8_t)234, (uint8_t)36, (uint8_t)140, (uint8_t)60, (uint8_t)169, (uint8_t)207, (uint8_t)121, (uint8_t)200, (uint8_t)7, (uint8_t)30, (uint8_t)87, (uint8_t)237, (uint8_t)253, (uint8_t)101, (uint8_t)89, (uint8_t)227, (uint8_t)150, (uint8_t)138, (uint8_t)68, (uint8_t)110, (uint8_t)249, (uint8_t)35, (uint8_t)228, (uint8_t)196, (uint8_t)5, (uint8_t)221, (uint8_t)174, (uint8_t)170, (uint8_t)14, (uint8_t)118, (uint8_t)216, (uint8_t)235, (uint8_t)63, (uint8_t)54, (uint8_t)13, (uint8_t)5, (uint8_t)189, (uint8_t)79, (uint8_t)20, (uint8_t)162, (uint8_t)240, (uint8_t)33, (uint8_t)97, (uint8_t)176, (uint8_t)180, (uint8_t)165, (uint8_t)195, (uint8_t)17, (uint8_t)155, (uint8_t)219, (uint8_t)204, (uint8_t)236, (uint8_t)185, (uint8_t)241, (uint8_t)58, (uint8_t)4, (uint8_t)126, (uint8_t)122, (uint8_t)99, (uint8_t)182, (uint8_t)221, (uint8_t)98, (uint8_t)75, (uint8_t)165, (uint8_t)74, (uint8_t)89, (uint8_t)126, (uint8_t)82, (uint8_t)50, (uint8_t)96, (uint8_t)139, (uint8_t)57, (uint8_t)105, (uint8_t)229, (uint8_t)247, (uint8_t)10, (uint8_t)36, (uint8_t)62, (uint8_t)206, (uint8_t)173, (uint8_t)95, (uint8_t)248, (uint8_t)156, (uint8_t)45, (uint8_t)176, (uint8_t)96, (uint8_t)155, (uint8_t)26, (uint8_t)59, (uint8_t)78, (uint8_t)120, (uint8_t)116, (uint8_t)16, (uint8_t)144, (uint8_t)44, (uint8_t)142, (uint8_t)121, (uint8_t)130, (uint8_t)144, (uint8_t)244, (uint8_t)43, (uint8_t)252, (uint8_t)183, (uint8_t)196, (uint8_t)18, (uint8_t)150, (uint8_t)113, (uint8_t)83, (uint8_t)100, (uint8_t)228, (uint8_t)7, (uint8_t)57, (uint8_t)161, (uint8_t)107, (uint8_t)124, (uint8_t)152, (uint8_t)53, (uint8_t)113, (uint8_t)5, (uint8_t)1, (uint8_t)231, (uint8_t)252, (uint8_t)111, (uint8_t)222, (uint8_t)21, (uint8_t)161} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)39398);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)44322);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)17665);
    assert(p269_framerate_GET(pack) == (float) -7.688928E37F);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p269_uri_LEN(ph) == 42);
    {
        char16_t * exemplary = u"zsmEfkPpgaPDuahhgUjydyueozmcdPfoSnptbftjzk";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 84);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)7597);
    assert(p269_bitrate_GET(pack) == (uint32_t)32247602L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)24356);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p270_uri_LEN(ph) == 103);
    {
        char16_t * exemplary = u"qeglumqjkfdzoakikwifojhstcrxkobbtoMnssnYuecwjqsduofySnOfionijmfczmpfvdxninxaRwbyfspkiolkoueelyumjobomgg";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 206);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p270_bitrate_GET(pack) == (uint32_t)1414570100L);
    assert(p270_framerate_GET(pack) == (float) -2.163217E38F);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)61022);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)7535);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)36019);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 25);
    {
        char16_t * exemplary = u"EsahcfgyrcfvxqmfwcaxwEmeT";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 11);
    {
        char16_t * exemplary = u"jtcnvsmwkox";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)31073);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)52494);
    {
        uint8_t exemplary[] =  {(uint8_t)75, (uint8_t)125, (uint8_t)195, (uint8_t)216, (uint8_t)247, (uint8_t)57, (uint8_t)9, (uint8_t)100} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)122, (uint8_t)83, (uint8_t)149, (uint8_t)21, (uint8_t)148, (uint8_t)110, (uint8_t)190, (uint8_t)252} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)5980);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)2820785682L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_time_usec_GET(pack) == (uint64_t)2091706697018370340L);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)49303);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)204);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_time_usec_GET(pack) == (uint64_t)6693162421961716222L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3969457753L);
    assert(p311_name_LEN(ph) == 13);
    {
        char16_t * exemplary = u"olgysIxnesyfh";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)3056618840L);
    {
        uint8_t exemplary[] =  {(uint8_t)118, (uint8_t)72, (uint8_t)67, (uint8_t)183, (uint8_t)55, (uint8_t)98, (uint8_t)64, (uint8_t)23, (uint8_t)223, (uint8_t)166, (uint8_t)242, (uint8_t)227, (uint8_t)215, (uint8_t)113, (uint8_t)96, (uint8_t)221} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"uuw";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -32115);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)225);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 94);
    {
        char16_t * exemplary = u"ANvnvdgvitehfznqgufHiypcYxixoygittimqxnLlcSxgjFznykugrnMmhbjtwyRbkztalskjftWbuzgjnkLmthvispfpu";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 188);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"fmj";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)5004);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)38912);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 28);
    {
        char16_t * exemplary = u"ZOBUsqhuslqbnjdbEwtvggkpbopx";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p323_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"ro";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)179);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_value_LEN(ph) == 122);
    {
        char16_t * exemplary = u"zxoojkmjmesiHqpWkyjipYUZwaoqnrGQVifYyyjlfihrgmQcAjgarxefowiveonnZoqnfdxaavtcgyqvrnkceipqtovaowbechsvfzukdpTibvvvkvntymbksf";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 244);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"wuy";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)32103);
    assert(p330_time_usec_GET(pack) == (uint64_t)8439960605934931700L);
    {
        uint16_t exemplary[] =  {(uint16_t)171, (uint16_t)37937, (uint16_t)16589, (uint16_t)42419, (uint16_t)8838, (uint16_t)58978, (uint16_t)18491, (uint16_t)1150, (uint16_t)54738, (uint16_t)43603, (uint16_t)12310, (uint16_t)27548, (uint16_t)32697, (uint16_t)53435, (uint16_t)38194, (uint16_t)62787, (uint16_t)5690, (uint16_t)16856, (uint16_t)29125, (uint16_t)53868, (uint16_t)33883, (uint16_t)5286, (uint16_t)5504, (uint16_t)21238, (uint16_t)14943, (uint16_t)10647, (uint16_t)20367, (uint16_t)51027, (uint16_t)63649, (uint16_t)4687, (uint16_t)10210, (uint16_t)13805, (uint16_t)43335, (uint16_t)40771, (uint16_t)63471, (uint16_t)37288, (uint16_t)43728, (uint16_t)5115, (uint16_t)35754, (uint16_t)37882, (uint16_t)52708, (uint16_t)28610, (uint16_t)47219, (uint16_t)14438, (uint16_t)43095, (uint16_t)41531, (uint16_t)38369, (uint16_t)65090, (uint16_t)43006, (uint16_t)55386, (uint16_t)29517, (uint16_t)3129, (uint16_t)7754, (uint16_t)10695, (uint16_t)48755, (uint16_t)7479, (uint16_t)25269, (uint16_t)43557, (uint16_t)60236, (uint16_t)49178, (uint16_t)62601, (uint16_t)5954, (uint16_t)54347, (uint16_t)35535, (uint16_t)45526, (uint16_t)34299, (uint16_t)9805, (uint16_t)21052, (uint16_t)35254, (uint16_t)1986, (uint16_t)42050, (uint16_t)12890} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)33736);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)254);
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
        p0_custom_mode_SET((uint32_t)2765560595L, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED), PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_DUOROTOR, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_current_battery_SET((int16_t)(int16_t) -17940, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)57141, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)71, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)50154, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)59, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)22888, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)41235, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)6691, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)34483, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)12110, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2904164346115591784L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)389453065L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_x_SET((float)1.5294025E38F, PH.base.pack) ;
        p3_afy_SET((float) -1.1907058E38F, PH.base.pack) ;
        p3_vx_SET((float) -2.7560558E38F, PH.base.pack) ;
        p3_vz_SET((float)1.1184797E37F, PH.base.pack) ;
        p3_y_SET((float) -1.5284626E38F, PH.base.pack) ;
        p3_z_SET((float) -2.8835187E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3687649025L, PH.base.pack) ;
        p3_vy_SET((float)3.0810749E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)29615, PH.base.pack) ;
        p3_yaw_SET((float)1.8057346E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.7340803E38F, PH.base.pack) ;
        p3_afx_SET((float)1.5692594E38F, PH.base.pack) ;
        p3_afz_SET((float)2.28606E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p4_seq_SET((uint32_t)1508622707L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)4300620714996981620L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"Fbmflercsrbgpgtn";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_control_request_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"cyLlxofkutacvrtcwimlzw";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)2945153450L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -21370, PH.base.pack) ;
        {
            char16_t* param_id = u"r";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_value_SET((float) -2.791541E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)31439, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)62353, PH.base.pack) ;
        {
            char16_t* param_id = u"uKykosap";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p23_param_value_SET((float) -2.9887598E36F, PH.base.pack) ;
        {
            char16_t* param_id = u"fXlfhw";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_vel_SET((uint16_t)(uint16_t)61369, PH.base.pack) ;
        p24_alt_SET((int32_t)480573365, PH.base.pack) ;
        p24_lon_SET((int32_t)537885413, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)790189333L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)801040295L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)429879186L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)41855, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)48332, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)502445952L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)26241, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -1165129026, &PH) ;
        p24_time_usec_SET((uint64_t)548337702172422829L, PH.base.pack) ;
        p24_lat_SET((int32_t)818619306, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)35, (uint8_t)181, (uint8_t)88, (uint8_t)172, (uint8_t)27, (uint8_t)188, (uint8_t)243, (uint8_t)220, (uint8_t)216, (uint8_t)202, (uint8_t)105, (uint8_t)107, (uint8_t)235, (uint8_t)166, (uint8_t)235, (uint8_t)234, (uint8_t)136, (uint8_t)163, (uint8_t)182, (uint8_t)124};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)250, (uint8_t)163, (uint8_t)214, (uint8_t)154, (uint8_t)50, (uint8_t)245, (uint8_t)130, (uint8_t)50, (uint8_t)136, (uint8_t)39, (uint8_t)69, (uint8_t)116, (uint8_t)146, (uint8_t)109, (uint8_t)121, (uint8_t)108, (uint8_t)75, (uint8_t)40, (uint8_t)123, (uint8_t)150};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)88, (uint8_t)30, (uint8_t)65, (uint8_t)6, (uint8_t)103, (uint8_t)55, (uint8_t)243, (uint8_t)213, (uint8_t)240, (uint8_t)27, (uint8_t)124, (uint8_t)39, (uint8_t)138, (uint8_t)210, (uint8_t)66, (uint8_t)46, (uint8_t)59, (uint8_t)81, (uint8_t)27, (uint8_t)22};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)132, (uint8_t)221, (uint8_t)35, (uint8_t)12, (uint8_t)240, (uint8_t)110, (uint8_t)94, (uint8_t)233, (uint8_t)24, (uint8_t)171, (uint8_t)238, (uint8_t)72, (uint8_t)64, (uint8_t)219, (uint8_t)190, (uint8_t)162, (uint8_t)12, (uint8_t)131, (uint8_t)55, (uint8_t)72};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)25, (uint8_t)52, (uint8_t)74, (uint8_t)112, (uint8_t)240, (uint8_t)241, (uint8_t)244, (uint8_t)205, (uint8_t)49, (uint8_t)94, (uint8_t)214, (uint8_t)82, (uint8_t)52, (uint8_t)62, (uint8_t)228, (uint8_t)235, (uint8_t)133, (uint8_t)164, (uint8_t)29, (uint8_t)220};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)2022357312L, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)30435, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -9127, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -9055, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)8699, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)29652, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)22823, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -16718, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)5228, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)20867, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zmag_SET((int16_t)(int16_t)11868, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)4802, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -95, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)32237, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)20769, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -14048, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)11187, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)12889, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)26801, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)3018704613867489834L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t) -26596, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)135, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)4929, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)8262, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)4824776670802958611L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_time_boot_ms_SET((uint32_t)1258189388L, PH.base.pack) ;
        p29_press_diff_SET((float) -2.4906438E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -2543, PH.base.pack) ;
        p29_press_abs_SET((float) -1.1724734E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitch_SET((float)2.723118E37F, PH.base.pack) ;
        p30_roll_SET((float) -9.858389E37F, PH.base.pack) ;
        p30_yawspeed_SET((float) -1.5910043E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)1078058709L, PH.base.pack) ;
        p30_pitchspeed_SET((float) -3.206644E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)2.0982719E38F, PH.base.pack) ;
        p30_yaw_SET((float) -2.6713252E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -2.5997522E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -1.0160632E38F, PH.base.pack) ;
        p31_q4_SET((float) -3.435712E37F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -2.1011183E38F, PH.base.pack) ;
        p31_q2_SET((float) -2.3637392E38F, PH.base.pack) ;
        p31_q1_SET((float)2.3889117E38F, PH.base.pack) ;
        p31_q3_SET((float) -2.4475597E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2866446519L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_time_boot_ms_SET((uint32_t)2642280173L, PH.base.pack) ;
        p32_y_SET((float) -1.9428575E38F, PH.base.pack) ;
        p32_z_SET((float)3.2133255E38F, PH.base.pack) ;
        p32_vy_SET((float)2.7353186E38F, PH.base.pack) ;
        p32_vx_SET((float)2.827804E38F, PH.base.pack) ;
        p32_vz_SET((float) -3.3085784E38F, PH.base.pack) ;
        p32_x_SET((float) -5.5791057E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_hdg_SET((uint16_t)(uint16_t)60207, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)9228, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)4114887637L, PH.base.pack) ;
        p33_lon_SET((int32_t)411384470, PH.base.pack) ;
        p33_alt_SET((int32_t) -1801273062, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1497693465, PH.base.pack) ;
        p33_lat_SET((int32_t) -212468275, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)13637, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)14369, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_time_boot_ms_SET((uint32_t)2225287753L, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)8547, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -831, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)5999, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -13375, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)29835, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -12276, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -17827, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)17586, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan2_raw_SET((uint16_t)(uint16_t)30068, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)13043, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)3542, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3199596468L, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)16200, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)30198, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)36863, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)31381, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)14644, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo5_raw_SET((uint16_t)(uint16_t)39368, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)56874, &PH) ;
        p36_time_usec_SET((uint32_t)2559541550L, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)39925, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)46686, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)37365, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)56631, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)46632, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)31285, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)16978, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)17194, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)15996, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)12339, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)26119, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)35003, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)21138, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)16147, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t) -29822, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)18893, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_start_index_SET((int16_t)(int16_t)7690, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -15543, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_autocontinue_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p39_param1_SET((float)1.413518E38F, PH.base.pack) ;
        p39_param3_SET((float)1.9861726E38F, PH.base.pack) ;
        p39_x_SET((float) -2.1482076E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p39_z_SET((float)6.821716E36F, PH.base.pack) ;
        p39_y_SET((float)3.2811053E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)24642, PH.base.pack) ;
        p39_param2_SET((float)2.3775986E38F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p39_param4_SET((float) -2.1585352E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)17071, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)4764, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)39991, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)26445, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)54535, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_DENIED, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -87210337, PH.base.pack) ;
        p48_longitude_SET((int32_t)737923821, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1221526684, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)5554771672005870911L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)3771774416740675194L, &PH) ;
        p49_altitude_SET((int32_t)1593725200, PH.base.pack) ;
        p49_latitude_SET((int32_t)1391041402, PH.base.pack) ;
        p49_longitude_SET((int32_t) -1152321568, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        {
            char16_t* param_id = u"lltdlf";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_min_SET((float) -9.74519E37F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p50_scale_SET((float)1.529815E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p50_param_value_max_SET((float) -1.5813724E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -5179, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p50_param_value0_SET((float) -3.3710417E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)35866, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p54_p1z_SET((float) -2.9351482E38F, PH.base.pack) ;
        p54_p2z_SET((float)2.3098728E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p54_p1x_SET((float)2.3615428E38F, PH.base.pack) ;
        p54_p2y_SET((float) -2.985116E38F, PH.base.pack) ;
        p54_p2x_SET((float) -1.717018E38F, PH.base.pack) ;
        p54_p1y_SET((float)2.3971222E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float) -1.3836933E38F, PH.base.pack) ;
        p55_p1x_SET((float)1.1878581E38F, PH.base.pack) ;
        p55_p1y_SET((float) -2.7469124E38F, PH.base.pack) ;
        p55_p1z_SET((float) -3.1616534E38F, PH.base.pack) ;
        p55_p2y_SET((float)2.687833E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p55_p2z_SET((float)1.6697206E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)504932440565637652L, PH.base.pack) ;
        p61_rollspeed_SET((float) -8.660185E37F, PH.base.pack) ;
        p61_yawspeed_SET((float)8.641544E37F, PH.base.pack) ;
        {
            float covariance[] =  {3.2484912E38F, 1.6568845E38F, -1.3075093E37F, -2.1894364E38F, -4.7509334E37F, -1.3305802E38F, 1.58605E37F, 2.6180847E38F, 3.1964242E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)8.883747E37F, PH.base.pack) ;
        {
            float q[] =  {6.705601E37F, 6.082038E37F, 2.4419015E38F, 3.1459472E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_roll_SET((float)2.0906606E37F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)56258, PH.base.pack) ;
        p62_nav_pitch_SET((float) -2.49632E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -98, PH.base.pack) ;
        p62_aspd_error_SET((float) -2.6977704E38F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.0374413E38F, PH.base.pack) ;
        p62_alt_error_SET((float)7.20684E37F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)32148, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_alt_SET((int32_t)714640761, PH.base.pack) ;
        p63_lon_SET((int32_t)531195949, PH.base.pack) ;
        p63_vx_SET((float)1.461029E38F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        p63_vy_SET((float) -1.5260762E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.6336284E38F, -1.7525947E38F, -8.3366007E37F, 1.0797073E38F, -1.4674999E38F, -1.699904E38F, 4.2340278E37F, -1.7932378E38F, 7.481944E37F, -1.7216403E38F, -6.75903E37F, -1.061725E37F, 1.0108357E38F, 1.1923779E38F, -1.1735329E38F, -1.3566328E38F, -1.8322679E38F, 1.756308E38F, 1.826495E38F, -3.283705E38F, -7.6467975E37F, -1.733567E38F, 1.9427897E38F, -1.7305278E38F, -1.9478543E38F, -9.953078E37F, -2.7793424E38F, 3.3665092E38F, 2.1310438E38F, -3.4844972E37F, 3.3343046E38F, -1.9570002E38F, 2.6099772E37F, -3.1705189E38F, 3.3979622E38F, -3.3707967E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float)5.189075E37F, PH.base.pack) ;
        p63_lat_SET((int32_t)1935886934, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -1734102324, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)6910023968277355277L, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        {
            float covariance[] =  {-1.2414482E38F, 1.7600872E38F, -2.2954213E38F, -1.856652E38F, -2.3349522E38F, 1.1506343E38F, -2.6191166E38F, 1.2032473E38F, -2.3101904E38F, 1.6458957E37F, -1.711168E37F, -2.5143716E38F, -7.123413E37F, 1.3240676E38F, -7.707169E37F, 2.0442754E38F, -2.4997328E38F, -3.2127655E38F, -2.6852233E38F, -2.2803432E38F, -2.1297232E38F, 1.3315088E38F, -1.958259E38F, -1.9939935E38F, 1.9013222E38F, -2.7606388E38F, -1.796513E38F, -1.6648603E38F, 1.1853338E38F, 2.0207054E38F, -1.9865953E38F, 2.9566202E38F, -1.0853504E38F, -8.869235E37F, 1.6102348E38F, -2.412652E38F, 1.0875981E38F, 1.8218368E38F, -1.081966E38F, 2.5017867E38F, 2.1683575E38F, 8.836392E37F, 2.4132358E38F, -2.4604905E38F, -2.908512E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vy_SET((float) -3.1453405E38F, PH.base.pack) ;
        p64_y_SET((float) -6.5168006E37F, PH.base.pack) ;
        p64_vx_SET((float)1.4072036E38F, PH.base.pack) ;
        p64_ax_SET((float) -1.1272131E38F, PH.base.pack) ;
        p64_x_SET((float)2.569028E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)1243273728801440186L, PH.base.pack) ;
        p64_az_SET((float)2.2328884E38F, PH.base.pack) ;
        p64_z_SET((float) -2.899734E38F, PH.base.pack) ;
        p64_vz_SET((float) -2.784719E38F, PH.base.pack) ;
        p64_ay_SET((float) -2.993234E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan12_raw_SET((uint16_t)(uint16_t)22954, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)61254, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)56292, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)42076, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)43344, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)59751, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)5804, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)55975, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)29873, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)11260, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)3365, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)822729750L, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)50789, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)3759, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)46329, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)61332, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)44728, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)44669, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)26364, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)43377, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)42571, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_x_SET((int16_t)(int16_t)5815, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)19256, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)25896, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -17081, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)58188, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan8_raw_SET((uint16_t)(uint16_t)2838, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)38093, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)24467, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)43240, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)23167, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)62049, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)22386, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)39296, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_target_component_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p73_param3_SET((float) -2.7265112E38F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)212, PH.base.pack) ;
        p73_param2_SET((float)1.4410962E38F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_FLIGHT_INFORMATION, PH.base.pack) ;
        p73_x_SET((int32_t)1966636293, PH.base.pack) ;
        p73_param1_SET((float) -4.834003E37F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p73_z_SET((float)3.2321985E38F, PH.base.pack) ;
        p73_param4_SET((float) -1.6046381E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p73_y_SET((int32_t) -1627602531, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_heading_SET((int16_t)(int16_t)6954, PH.base.pack) ;
        p74_groundspeed_SET((float) -1.0442035E38F, PH.base.pack) ;
        p74_climb_SET((float) -3.987933E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)58823, PH.base.pack) ;
        p74_airspeed_SET((float)7.044281E37F, PH.base.pack) ;
        p74_alt_SET((float) -3.9373114E37F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_current_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p75_param1_SET((float) -2.4005228E38F, PH.base.pack) ;
        p75_x_SET((int32_t)888939576, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p75_param2_SET((float)7.7799287E37F, PH.base.pack) ;
        p75_param4_SET((float) -7.78055E37F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_RELAY, PH.base.pack) ;
        p75_y_SET((int32_t)932515132, PH.base.pack) ;
        p75_param3_SET((float)2.3381805E38F, PH.base.pack) ;
        p75_z_SET((float) -2.9496735E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float)2.4021202E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_CHANGE_SPEED, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p76_param2_SET((float)1.2146495E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p76_param4_SET((float)1.8624874E38F, PH.base.pack) ;
        p76_param1_SET((float) -8.7984244E36F, PH.base.pack) ;
        p76_param5_SET((float)2.7341126E38F, PH.base.pack) ;
        p76_param7_SET((float)1.898853E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p76_param3_SET((float)3.2084841E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_system_SET((uint8_t)(uint8_t)63, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_WAYPOINT, PH.base.pack) ;
        p77_result_param2_SET((int32_t) -733126142, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)206, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)6, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_thrust_SET((float)2.5057286E38F, PH.base.pack) ;
        p81_yaw_SET((float)5.493314E37F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p81_roll_SET((float) -2.3932769E38F, PH.base.pack) ;
        p81_pitch_SET((float) -1.1149528E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1121550506L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_type_mask_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        {
            float q[] =  {-1.3508732E38F, 2.4452937E38F, -9.714557E36F, 6.1933215E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float)3.2751331E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)4023736381L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p82_thrust_SET((float) -4.1609665E37F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)2.4651131E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)7.137042E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_pitch_rate_SET((float)1.4508214E38F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)2.9337958E38F, PH.base.pack) ;
        {
            float q[] =  {1.8296695E38F, -1.5013688E38F, -6.303796E37F, -1.7825145E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_time_boot_ms_SET((uint32_t)630258726L, PH.base.pack) ;
        p83_thrust_SET((float) -2.330182E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)2.1823798E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vy_SET((float) -3.9861849E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p84_yaw_SET((float)1.9134636E38F, PH.base.pack) ;
        p84_x_SET((float)2.957576E38F, PH.base.pack) ;
        p84_afy_SET((float) -3.0100029E38F, PH.base.pack) ;
        p84_y_SET((float)3.1753842E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -3.221492E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3496519776L, PH.base.pack) ;
        p84_vz_SET((float)8.648285E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)63551, PH.base.pack) ;
        p84_vx_SET((float)2.156836E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p84_afx_SET((float)3.0877853E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p84_afz_SET((float) -1.6416033E38F, PH.base.pack) ;
        p84_z_SET((float)1.53507E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_alt_SET((float)1.3151898E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2029955245L, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -2079461291, PH.base.pack) ;
        p86_afy_SET((float)2.188726E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p86_afx_SET((float) -1.1747156E37F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -2.335574E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p86_yaw_SET((float)1.3802159E38F, PH.base.pack) ;
        p86_afz_SET((float) -2.0598783E38F, PH.base.pack) ;
        p86_vz_SET((float)2.3835884E38F, PH.base.pack) ;
        p86_vx_SET((float) -1.6808125E37F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)37300, PH.base.pack) ;
        p86_lon_int_SET((int32_t)169839176, PH.base.pack) ;
        p86_vy_SET((float) -2.4878156E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vz_SET((float) -2.2817204E38F, PH.base.pack) ;
        p87_alt_SET((float)1.2891385E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)35757, PH.base.pack) ;
        p87_afx_SET((float)2.0580547E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.0139975E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)306592126L, PH.base.pack) ;
        p87_afy_SET((float)1.6131639E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)724419513, PH.base.pack) ;
        p87_afz_SET((float) -1.9577375E38F, PH.base.pack) ;
        p87_vx_SET((float)2.8481606E38F, PH.base.pack) ;
        p87_yaw_SET((float)3.2404664E38F, PH.base.pack) ;
        p87_vy_SET((float) -1.5180126E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1400318430, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_pitch_SET((float)1.378589E38F, PH.base.pack) ;
        p89_x_SET((float) -2.4288751E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)737846300L, PH.base.pack) ;
        p89_z_SET((float)1.6132474E37F, PH.base.pack) ;
        p89_yaw_SET((float)8.93284E37F, PH.base.pack) ;
        p89_y_SET((float)1.442257E38F, PH.base.pack) ;
        p89_roll_SET((float) -3.2615492E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_xacc_SET((int16_t)(int16_t)9634, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)28953, PH.base.pack) ;
        p90_pitchspeed_SET((float) -3.2211527E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -17870, PH.base.pack) ;
        p90_yawspeed_SET((float)2.2436404E38F, PH.base.pack) ;
        p90_alt_SET((int32_t)605984641, PH.base.pack) ;
        p90_pitch_SET((float) -2.6513219E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)19836, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -21496, PH.base.pack) ;
        p90_rollspeed_SET((float)1.9714812E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -9829, PH.base.pack) ;
        p90_roll_SET((float) -3.0624136E38F, PH.base.pack) ;
        p90_lon_SET((int32_t) -2071187641, PH.base.pack) ;
        p90_lat_SET((int32_t) -313568902, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)440135096532141811L, PH.base.pack) ;
        p90_yaw_SET((float)2.397357E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux3_SET((float) -3.2733004E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p91_throttle_SET((float)3.0359544E38F, PH.base.pack) ;
        p91_aux2_SET((float) -1.2189926E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p91_roll_ailerons_SET((float)2.4230853E38F, PH.base.pack) ;
        p91_aux1_SET((float) -2.5489237E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -1.5529365E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)7732800735633922047L, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -2.6084298E38F, PH.base.pack) ;
        p91_aux4_SET((float)2.1467632E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan8_raw_SET((uint16_t)(uint16_t)54238, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)35377, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)21744, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)15003, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)31443, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)9490, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)4990373399804635633L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)55756, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)50812, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)33528, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)61294, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)58620, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)42475, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)5255576608208803422L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        {
            float controls[] =  {1.9051563E38F, -1.061897E38F, -2.6617436E38F, 2.5953696E38F, 1.8149421E37F, -1.0928376E38F, -2.8882524E37F, -1.0977422E37F, 7.59596E37F, -4.703896E37F, -3.0071296E37F, -2.244871E38F, 3.126668E38F, 9.458305E37F, 5.298124E37F, 3.1206327E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_flags_SET((uint64_t)6453685364167101055L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_time_usec_SET((uint64_t)5385234053433603527L, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.3243757E38F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.9603462E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)12300, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -1.2604758E36F, &PH) ;
        p100_flow_comp_m_y_SET((float)2.8918485E38F, PH.base.pack) ;
        p100_ground_distance_SET((float) -5.9977626E37F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)18641, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_pitch_SET((float) -1.5756092E38F, PH.base.pack) ;
        p101_z_SET((float)3.0806999E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)865265721480554490L, PH.base.pack) ;
        p101_roll_SET((float) -3.1647627E38F, PH.base.pack) ;
        p101_x_SET((float)7.909868E37F, PH.base.pack) ;
        p101_y_SET((float)2.1137256E38F, PH.base.pack) ;
        p101_yaw_SET((float)3.112193E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float)1.3414545E37F, PH.base.pack) ;
        p102_yaw_SET((float) -6.2354805E37F, PH.base.pack) ;
        p102_y_SET((float) -2.716056E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)7619618762669387886L, PH.base.pack) ;
        p102_z_SET((float)2.3860598E38F, PH.base.pack) ;
        p102_roll_SET((float)1.6781606E38F, PH.base.pack) ;
        p102_pitch_SET((float)1.881858E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float) -7.395469E36F, PH.base.pack) ;
        p103_z_SET((float)7.934822E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)4168804493007514797L, PH.base.pack) ;
        p103_y_SET((float) -1.083757E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)8795994295406990599L, PH.base.pack) ;
        p104_pitch_SET((float)6.876075E37F, PH.base.pack) ;
        p104_x_SET((float) -1.9784783E38F, PH.base.pack) ;
        p104_y_SET((float)2.2089268E38F, PH.base.pack) ;
        p104_yaw_SET((float)8.0956984E37F, PH.base.pack) ;
        p104_z_SET((float)2.68513E38F, PH.base.pack) ;
        p104_roll_SET((float)3.016782E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_xgyro_SET((float)2.1798473E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)25291, PH.base.pack) ;
        p105_pressure_alt_SET((float)3.2928403E38F, PH.base.pack) ;
        p105_zacc_SET((float)2.1635766E38F, PH.base.pack) ;
        p105_xacc_SET((float) -2.1815214E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)2692725512311767966L, PH.base.pack) ;
        p105_ygyro_SET((float)2.1448283E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.1180095E38F, PH.base.pack) ;
        p105_zgyro_SET((float)2.6750229E38F, PH.base.pack) ;
        p105_zmag_SET((float) -3.084696E38F, PH.base.pack) ;
        p105_yacc_SET((float)2.0091737E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)9.315963E37F, PH.base.pack) ;
        p105_ymag_SET((float)1.4772264E38F, PH.base.pack) ;
        p105_temperature_SET((float)1.0782369E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.6629901E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integration_time_us_SET((uint32_t)1684602428L, PH.base.pack) ;
        p106_integrated_y_SET((float)2.4044766E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3673300684L, PH.base.pack) ;
        p106_distance_SET((float)2.2292686E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)6.0287796E36F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.4337514E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -7.2757425E37F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -1.5611486E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)3432953820719864489L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -18683, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_temperature_SET((float)4.5618805E37F, PH.base.pack) ;
        p107_zgyro_SET((float)1.5649051E38F, PH.base.pack) ;
        p107_xacc_SET((float) -2.0921774E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -4.9913773E37F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -3.2416627E38F, PH.base.pack) ;
        p107_yacc_SET((float)2.1935493E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2238306458L, PH.base.pack) ;
        p107_zacc_SET((float)2.0385364E37F, PH.base.pack) ;
        p107_xmag_SET((float) -3.0129824E38F, PH.base.pack) ;
        p107_zmag_SET((float)1.6663808E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -2.7990575E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)8408069849837580862L, PH.base.pack) ;
        p107_ygyro_SET((float)1.1394049E38F, PH.base.pack) ;
        p107_ymag_SET((float)1.4388346E38F, PH.base.pack) ;
        p107_xgyro_SET((float) -1.0606056E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_q1_SET((float)2.7665337E38F, PH.base.pack) ;
        p108_q3_SET((float)1.8930585E38F, PH.base.pack) ;
        p108_vd_SET((float) -7.965017E37F, PH.base.pack) ;
        p108_zacc_SET((float)1.0138871E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -7.4874863E37F, PH.base.pack) ;
        p108_roll_SET((float) -8.592887E37F, PH.base.pack) ;
        p108_xacc_SET((float)1.9671895E38F, PH.base.pack) ;
        p108_alt_SET((float) -1.8948588E38F, PH.base.pack) ;
        p108_vn_SET((float)1.8324407E38F, PH.base.pack) ;
        p108_q4_SET((float)1.3751737E38F, PH.base.pack) ;
        p108_zgyro_SET((float)1.0522486E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.0444782E38F, PH.base.pack) ;
        p108_lon_SET((float)2.0981658E38F, PH.base.pack) ;
        p108_lat_SET((float) -2.7244079E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)1.331162E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.0802463E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -5.0784355E37F, PH.base.pack) ;
        p108_ve_SET((float)6.3651977E37F, PH.base.pack) ;
        p108_pitch_SET((float) -2.6886354E38F, PH.base.pack) ;
        p108_yaw_SET((float)9.408274E37F, PH.base.pack) ;
        p108_yacc_SET((float)1.9577557E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_noise_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)26162, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)6490, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)202, (uint8_t)19, (uint8_t)216, (uint8_t)10, (uint8_t)205, (uint8_t)132, (uint8_t)155, (uint8_t)157, (uint8_t)181, (uint8_t)138, (uint8_t)111, (uint8_t)190, (uint8_t)120, (uint8_t)68, (uint8_t)107, (uint8_t)91, (uint8_t)58, (uint8_t)203, (uint8_t)106, (uint8_t)221, (uint8_t)178, (uint8_t)40, (uint8_t)164, (uint8_t)53, (uint8_t)55, (uint8_t)88, (uint8_t)7, (uint8_t)181, (uint8_t)219, (uint8_t)175, (uint8_t)22, (uint8_t)39, (uint8_t)65, (uint8_t)1, (uint8_t)114, (uint8_t)161, (uint8_t)179, (uint8_t)166, (uint8_t)93, (uint8_t)146, (uint8_t)146, (uint8_t)194, (uint8_t)238, (uint8_t)129, (uint8_t)145, (uint8_t)149, (uint8_t)177, (uint8_t)98, (uint8_t)200, (uint8_t)224, (uint8_t)1, (uint8_t)229, (uint8_t)30, (uint8_t)108, (uint8_t)135, (uint8_t)192, (uint8_t)215, (uint8_t)107, (uint8_t)245, (uint8_t)90, (uint8_t)23, (uint8_t)235, (uint8_t)12, (uint8_t)16, (uint8_t)216, (uint8_t)41, (uint8_t)138, (uint8_t)99, (uint8_t)12, (uint8_t)202, (uint8_t)150, (uint8_t)189, (uint8_t)187, (uint8_t)119, (uint8_t)73, (uint8_t)134, (uint8_t)224, (uint8_t)224, (uint8_t)34, (uint8_t)11, (uint8_t)160, (uint8_t)17, (uint8_t)253, (uint8_t)45, (uint8_t)235, (uint8_t)252, (uint8_t)172, (uint8_t)188, (uint8_t)212, (uint8_t)57, (uint8_t)127, (uint8_t)33, (uint8_t)61, (uint8_t)246, (uint8_t)120, (uint8_t)134, (uint8_t)165, (uint8_t)30, (uint8_t)32, (uint8_t)88, (uint8_t)48, (uint8_t)84, (uint8_t)152, (uint8_t)53, (uint8_t)180, (uint8_t)160, (uint8_t)178, (uint8_t)66, (uint8_t)130, (uint8_t)203, (uint8_t)30, (uint8_t)70, (uint8_t)36, (uint8_t)224, (uint8_t)252, (uint8_t)133, (uint8_t)54, (uint8_t)109, (uint8_t)136, (uint8_t)17, (uint8_t)178, (uint8_t)148, (uint8_t)199, (uint8_t)253, (uint8_t)85, (uint8_t)250, (uint8_t)138, (uint8_t)35, (uint8_t)199, (uint8_t)80, (uint8_t)96, (uint8_t)88, (uint8_t)71, (uint8_t)0, (uint8_t)225, (uint8_t)189, (uint8_t)216, (uint8_t)152, (uint8_t)171, (uint8_t)235, (uint8_t)115, (uint8_t)226, (uint8_t)2, (uint8_t)34, (uint8_t)127, (uint8_t)123, (uint8_t)97, (uint8_t)95, (uint8_t)252, (uint8_t)88, (uint8_t)140, (uint8_t)108, (uint8_t)131, (uint8_t)62, (uint8_t)233, (uint8_t)207, (uint8_t)13, (uint8_t)181, (uint8_t)169, (uint8_t)251, (uint8_t)73, (uint8_t)147, (uint8_t)192, (uint8_t)129, (uint8_t)248, (uint8_t)127, (uint8_t)43, (uint8_t)10, (uint8_t)117, (uint8_t)89, (uint8_t)67, (uint8_t)55, (uint8_t)51, (uint8_t)234, (uint8_t)77, (uint8_t)223, (uint8_t)169, (uint8_t)146, (uint8_t)84, (uint8_t)196, (uint8_t)252, (uint8_t)41, (uint8_t)104, (uint8_t)110, (uint8_t)91, (uint8_t)46, (uint8_t)251, (uint8_t)74, (uint8_t)32, (uint8_t)241, (uint8_t)195, (uint8_t)176, (uint8_t)157, (uint8_t)68, (uint8_t)222, (uint8_t)145, (uint8_t)57, (uint8_t)234, (uint8_t)167, (uint8_t)4, (uint8_t)89, (uint8_t)231, (uint8_t)105, (uint8_t)129, (uint8_t)46, (uint8_t)33, (uint8_t)65, (uint8_t)192, (uint8_t)165, (uint8_t)109, (uint8_t)83, (uint8_t)206, (uint8_t)43, (uint8_t)29, (uint8_t)196, (uint8_t)159, (uint8_t)206, (uint8_t)33, (uint8_t)40, (uint8_t)232, (uint8_t)231, (uint8_t)176, (uint8_t)120, (uint8_t)176, (uint8_t)53, (uint8_t)234, (uint8_t)38, (uint8_t)87, (uint8_t)59, (uint8_t)173, (uint8_t)59, (uint8_t)161, (uint8_t)232, (uint8_t)230, (uint8_t)60, (uint8_t)49, (uint8_t)139, (uint8_t)221, (uint8_t)116, (uint8_t)48, (uint8_t)27, (uint8_t)168, (uint8_t)227, (uint8_t)147, (uint8_t)69, (uint8_t)33, (uint8_t)224, (uint8_t)42, (uint8_t)171, (uint8_t)27, (uint8_t)109};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t) -3519508074601737126L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -5661291639675183456L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)1581092729751413986L, PH.base.pack) ;
        p112_seq_SET((uint32_t)306545998L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_eph_SET((uint16_t)(uint16_t)51739, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)2489, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -31751, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)10689, PH.base.pack) ;
        p113_alt_SET((int32_t) -1418259808, PH.base.pack) ;
        p113_lon_SET((int32_t) -1489340241, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)6112075186638870857L, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p113_lat_SET((int32_t) -1551286042, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)17477, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)59138, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)4844, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_ygyro_SET((float)1.847266E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)3685658382347545169L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4274085146L, PH.base.pack) ;
        p114_distance_SET((float) -3.006049E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)1947027844L, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)6.899169E37F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)10435, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)3.1476292E38F, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.007425E38F, PH.base.pack) ;
        p114_integrated_y_SET((float) -1.9330055E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_vx_SET((int16_t)(int16_t) -1504, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)37757, PH.base.pack) ;
        p115_pitchspeed_SET((float) -1.195181E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -15803, PH.base.pack) ;
        p115_alt_SET((int32_t) -1149005288, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -10836, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)3573070863460337349L, PH.base.pack) ;
        p115_rollspeed_SET((float) -3.2248945E37F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-8.9060694E35F, -2.4381008E38F, 3.3206032E38F, -2.5177543E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_zacc_SET((int16_t)(int16_t)15526, PH.base.pack) ;
        p115_lat_SET((int32_t)1996842973, PH.base.pack) ;
        p115_lon_SET((int32_t)70388355, PH.base.pack) ;
        p115_yawspeed_SET((float)3.0364414E38F, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)7161, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)28308, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)13901, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zmag_SET((int16_t)(int16_t)3931, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)7935, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -23561, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)3961, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -8463, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -10219, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2315676151L, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)29945, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -32064, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)18244, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)9714, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)61703, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)2202937657L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)47048, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)52904, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1868786887L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)19270, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)26792, PH.base.pack) ;
        p119_count_SET((uint32_t)3042034430L, PH.base.pack) ;
        p119_ofs_SET((uint32_t)4261066505L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)4251109799L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)11938, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)83, (uint8_t)79, (uint8_t)79, (uint8_t)189, (uint8_t)100, (uint8_t)243, (uint8_t)10, (uint8_t)145, (uint8_t)95, (uint8_t)49, (uint8_t)96, (uint8_t)244, (uint8_t)51, (uint8_t)172, (uint8_t)193, (uint8_t)161, (uint8_t)26, (uint8_t)37, (uint8_t)131, (uint8_t)130, (uint8_t)253, (uint8_t)141, (uint8_t)87, (uint8_t)14, (uint8_t)253, (uint8_t)123, (uint8_t)221, (uint8_t)3, (uint8_t)91, (uint8_t)184, (uint8_t)189, (uint8_t)135, (uint8_t)153, (uint8_t)120, (uint8_t)236, (uint8_t)113, (uint8_t)179, (uint8_t)15, (uint8_t)67, (uint8_t)236, (uint8_t)42, (uint8_t)27, (uint8_t)185, (uint8_t)70, (uint8_t)103, (uint8_t)204, (uint8_t)227, (uint8_t)105, (uint8_t)128, (uint8_t)36, (uint8_t)251, (uint8_t)233, (uint8_t)16, (uint8_t)141, (uint8_t)203, (uint8_t)97, (uint8_t)112, (uint8_t)232, (uint8_t)33, (uint8_t)19, (uint8_t)188, (uint8_t)246, (uint8_t)162, (uint8_t)94, (uint8_t)197, (uint8_t)23, (uint8_t)138, (uint8_t)136, (uint8_t)96, (uint8_t)98, (uint8_t)196, (uint8_t)174, (uint8_t)220, (uint8_t)87, (uint8_t)132, (uint8_t)96, (uint8_t)101, (uint8_t)21, (uint8_t)47, (uint8_t)178, (uint8_t)170, (uint8_t)35, (uint8_t)63, (uint8_t)154, (uint8_t)7, (uint8_t)4, (uint8_t)101, (uint8_t)164, (uint8_t)6, (uint8_t)200};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)211, (uint8_t)154, (uint8_t)152, (uint8_t)111, (uint8_t)253, (uint8_t)225, (uint8_t)154, (uint8_t)97, (uint8_t)254, (uint8_t)227, (uint8_t)181, (uint8_t)189, (uint8_t)183, (uint8_t)250, (uint8_t)203, (uint8_t)206, (uint8_t)200, (uint8_t)249, (uint8_t)226, (uint8_t)82, (uint8_t)171, (uint8_t)97, (uint8_t)170, (uint8_t)212, (uint8_t)78, (uint8_t)252, (uint8_t)231, (uint8_t)5, (uint8_t)130, (uint8_t)165, (uint8_t)147, (uint8_t)60, (uint8_t)184, (uint8_t)67, (uint8_t)102, (uint8_t)87, (uint8_t)170, (uint8_t)51, (uint8_t)251, (uint8_t)196, (uint8_t)125, (uint8_t)10, (uint8_t)163, (uint8_t)37, (uint8_t)145, (uint8_t)243, (uint8_t)91, (uint8_t)13, (uint8_t)67, (uint8_t)180, (uint8_t)185, (uint8_t)178, (uint8_t)139, (uint8_t)248, (uint8_t)204, (uint8_t)247, (uint8_t)80, (uint8_t)203, (uint8_t)238, (uint8_t)124, (uint8_t)132, (uint8_t)15, (uint8_t)138, (uint8_t)25, (uint8_t)60, (uint8_t)91, (uint8_t)202, (uint8_t)150, (uint8_t)119, (uint8_t)188, (uint8_t)13, (uint8_t)110, (uint8_t)6, (uint8_t)23, (uint8_t)159, (uint8_t)107, (uint8_t)30, (uint8_t)111, (uint8_t)203, (uint8_t)71, (uint8_t)237, (uint8_t)208, (uint8_t)239, (uint8_t)77, (uint8_t)192, (uint8_t)51, (uint8_t)80, (uint8_t)107, (uint8_t)245, (uint8_t)66, (uint8_t)98, (uint8_t)81, (uint8_t)49, (uint8_t)155, (uint8_t)167, (uint8_t)64, (uint8_t)188, (uint8_t)140, (uint8_t)118, (uint8_t)200, (uint8_t)86, (uint8_t)220, (uint8_t)117, (uint8_t)147, (uint8_t)43, (uint8_t)166, (uint8_t)219, (uint8_t)129, (uint8_t)165, (uint8_t)99};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p124_lon_SET((int32_t)639066999, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)3174395424994094556L, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)26856, PH.base.pack) ;
        p124_alt_SET((int32_t)154152148, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)32809, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)656, PH.base.pack) ;
        p124_lat_SET((int32_t) -1673918017, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)30096, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1080131957L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)62451, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)62378, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED), PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)53108, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1020505612L, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)143, (uint8_t)37, (uint8_t)57, (uint8_t)159, (uint8_t)140, (uint8_t)32, (uint8_t)141, (uint8_t)83, (uint8_t)138, (uint8_t)61, (uint8_t)239, (uint8_t)110, (uint8_t)7, (uint8_t)0, (uint8_t)8, (uint8_t)160, (uint8_t)61, (uint8_t)70, (uint8_t)196, (uint8_t)118, (uint8_t)76, (uint8_t)49, (uint8_t)2, (uint8_t)167, (uint8_t)65, (uint8_t)196, (uint8_t)137, (uint8_t)94, (uint8_t)252, (uint8_t)17, (uint8_t)131, (uint8_t)184, (uint8_t)159, (uint8_t)78, (uint8_t)235, (uint8_t)180, (uint8_t)201, (uint8_t)236, (uint8_t)97, (uint8_t)30, (uint8_t)224, (uint8_t)227, (uint8_t)72, (uint8_t)26, (uint8_t)249, (uint8_t)132, (uint8_t)213, (uint8_t)73, (uint8_t)101, (uint8_t)90, (uint8_t)227, (uint8_t)143, (uint8_t)1, (uint8_t)112, (uint8_t)195, (uint8_t)81, (uint8_t)87, (uint8_t)237, (uint8_t)91, (uint8_t)223, (uint8_t)163, (uint8_t)24, (uint8_t)4, (uint8_t)171, (uint8_t)6, (uint8_t)51, (uint8_t)120, (uint8_t)35, (uint8_t)213, (uint8_t)213};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_accuracy_SET((uint32_t)1735505008L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)8839, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -2079477454, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)106988358L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -162489806, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1745317382, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)868218239, PH.base.pack) ;
        p127_tow_SET((uint32_t)2147003435L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_iar_num_hypotheses_SET((int32_t) -1225865703, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -127357716, PH.base.pack) ;
        p128_tow_SET((uint32_t)470932486L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)4230312724L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)33559, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)2663007119L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)1809751482, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)1351442447, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_zgyro_SET((int16_t)(int16_t)26814, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)797, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)11193, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1548431394L, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)31363, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -16304, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -13471, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)29361, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)891, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -10537, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_width_SET((uint16_t)(uint16_t)37523, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)29150, PH.base.pack) ;
        p130_size_SET((uint32_t)2080673138L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)33212, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)12609, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)112, (uint8_t)32, (uint8_t)56, (uint8_t)237, (uint8_t)166, (uint8_t)149, (uint8_t)46, (uint8_t)118, (uint8_t)180, (uint8_t)169, (uint8_t)129, (uint8_t)16, (uint8_t)76, (uint8_t)104, (uint8_t)124, (uint8_t)189, (uint8_t)212, (uint8_t)232, (uint8_t)64, (uint8_t)128, (uint8_t)89, (uint8_t)110, (uint8_t)2, (uint8_t)239, (uint8_t)137, (uint8_t)245, (uint8_t)81, (uint8_t)41, (uint8_t)226, (uint8_t)172, (uint8_t)211, (uint8_t)16, (uint8_t)67, (uint8_t)120, (uint8_t)207, (uint8_t)43, (uint8_t)8, (uint8_t)206, (uint8_t)221, (uint8_t)156, (uint8_t)18, (uint8_t)179, (uint8_t)159, (uint8_t)92, (uint8_t)123, (uint8_t)135, (uint8_t)143, (uint8_t)237, (uint8_t)27, (uint8_t)218, (uint8_t)42, (uint8_t)246, (uint8_t)58, (uint8_t)230, (uint8_t)9, (uint8_t)108, (uint8_t)98, (uint8_t)111, (uint8_t)104, (uint8_t)197, (uint8_t)113, (uint8_t)74, (uint8_t)46, (uint8_t)58, (uint8_t)146, (uint8_t)0, (uint8_t)243, (uint8_t)3, (uint8_t)193, (uint8_t)120, (uint8_t)220, (uint8_t)236, (uint8_t)95, (uint8_t)67, (uint8_t)131, (uint8_t)17, (uint8_t)120, (uint8_t)47, (uint8_t)254, (uint8_t)173, (uint8_t)242, (uint8_t)181, (uint8_t)160, (uint8_t)199, (uint8_t)39, (uint8_t)201, (uint8_t)124, (uint8_t)112, (uint8_t)35, (uint8_t)170, (uint8_t)111, (uint8_t)72, (uint8_t)159, (uint8_t)14, (uint8_t)87, (uint8_t)101, (uint8_t)171, (uint8_t)214, (uint8_t)222, (uint8_t)30, (uint8_t)198, (uint8_t)45, (uint8_t)180, (uint8_t)210, (uint8_t)204, (uint8_t)246, (uint8_t)244, (uint8_t)143, (uint8_t)47, (uint8_t)218, (uint8_t)20, (uint8_t)135, (uint8_t)143, (uint8_t)86, (uint8_t)239, (uint8_t)147, (uint8_t)89, (uint8_t)20, (uint8_t)1, (uint8_t)186, (uint8_t)23, (uint8_t)82, (uint8_t)9, (uint8_t)113, (uint8_t)18, (uint8_t)127, (uint8_t)175, (uint8_t)34, (uint8_t)8, (uint8_t)8, (uint8_t)239, (uint8_t)200, (uint8_t)92, (uint8_t)109, (uint8_t)82, (uint8_t)104, (uint8_t)52, (uint8_t)161, (uint8_t)247, (uint8_t)154, (uint8_t)114, (uint8_t)165, (uint8_t)22, (uint8_t)134, (uint8_t)34, (uint8_t)53, (uint8_t)178, (uint8_t)24, (uint8_t)84, (uint8_t)81, (uint8_t)92, (uint8_t)24, (uint8_t)21, (uint8_t)48, (uint8_t)49, (uint8_t)46, (uint8_t)41, (uint8_t)210, (uint8_t)210, (uint8_t)253, (uint8_t)57, (uint8_t)47, (uint8_t)30, (uint8_t)62, (uint8_t)225, (uint8_t)108, (uint8_t)14, (uint8_t)255, (uint8_t)16, (uint8_t)142, (uint8_t)66, (uint8_t)124, (uint8_t)176, (uint8_t)228, (uint8_t)162, (uint8_t)85, (uint8_t)226, (uint8_t)62, (uint8_t)139, (uint8_t)243, (uint8_t)135, (uint8_t)0, (uint8_t)215, (uint8_t)195, (uint8_t)89, (uint8_t)149, (uint8_t)118, (uint8_t)20, (uint8_t)76, (uint8_t)105, (uint8_t)232, (uint8_t)244, (uint8_t)124, (uint8_t)153, (uint8_t)25, (uint8_t)41, (uint8_t)242, (uint8_t)73, (uint8_t)161, (uint8_t)148, (uint8_t)81, (uint8_t)181, (uint8_t)250, (uint8_t)68, (uint8_t)152, (uint8_t)64, (uint8_t)168, (uint8_t)193, (uint8_t)53, (uint8_t)122, (uint8_t)141, (uint8_t)187, (uint8_t)13, (uint8_t)234, (uint8_t)115, (uint8_t)238, (uint8_t)109, (uint8_t)141, (uint8_t)202, (uint8_t)8, (uint8_t)66, (uint8_t)106, (uint8_t)234, (uint8_t)70, (uint8_t)102, (uint8_t)75, (uint8_t)79, (uint8_t)20, (uint8_t)63, (uint8_t)41, (uint8_t)229, (uint8_t)97, (uint8_t)60, (uint8_t)158, (uint8_t)249, (uint8_t)255, (uint8_t)159, (uint8_t)47, (uint8_t)111, (uint8_t)131, (uint8_t)35, (uint8_t)199, (uint8_t)228, (uint8_t)173, (uint8_t)6, (uint8_t)201, (uint8_t)74, (uint8_t)245, (uint8_t)129, (uint8_t)230, (uint8_t)231, (uint8_t)206, (uint8_t)153};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_current_distance_SET((uint16_t)(uint16_t)47447, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)57588, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3929050721L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)47487, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)16068, PH.base.pack) ;
        p133_mask_SET((uint64_t)7142430660652634481L, PH.base.pack) ;
        p133_lat_SET((int32_t) -1650183395, PH.base.pack) ;
        p133_lon_SET((int32_t)860656203, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t) -185220575, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)12329, (int16_t)25304, (int16_t)11329, (int16_t) -18567, (int16_t) -8631, (int16_t)2801, (int16_t) -3771, (int16_t) -29410, (int16_t) -12931, (int16_t)25475, (int16_t) -23582, (int16_t) -30593, (int16_t)23704, (int16_t) -2204, (int16_t)3009, (int16_t)30347};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)55374, PH.base.pack) ;
        p134_lat_SET((int32_t) -1307708020, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t)1541502441, PH.base.pack) ;
        p135_lat_SET((int32_t)553852956, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)18692, PH.base.pack) ;
        p136_lat_SET((int32_t)1124509925, PH.base.pack) ;
        p136_current_height_SET((float)2.3808515E38F, PH.base.pack) ;
        p136_terrain_height_SET((float)2.3446822E38F, PH.base.pack) ;
        p136_lon_SET((int32_t) -433166983, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)61132, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)30168, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float) -1.750134E38F, PH.base.pack) ;
        p137_press_diff_SET((float)2.6170963E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)1625246619L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)7676, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)3655874965958432471L, PH.base.pack) ;
        p138_z_SET((float)2.0006983E38F, PH.base.pack) ;
        p138_x_SET((float) -7.2635746E37F, PH.base.pack) ;
        {
            float q[] =  {-3.1664385E38F, 2.933916E38F, 2.0031352E38F, -2.5617482E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_y_SET((float) -1.4543669E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_time_usec_SET((uint64_t)4098417378326167877L, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        {
            float controls[] =  {-1.4454737E38F, -2.450314E38F, -5.0044843E37F, 2.7733589E38F, -2.912193E38F, 3.2973227E38F, -2.0633103E38F, -3.1971316E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)8787075891538174162L, PH.base.pack) ;
        {
            float controls[] =  {2.7731066E38F, -2.3891387E38F, -2.9487419E38F, 3.2484407E38F, -1.219474E36F, 2.744994E38F, -3.0853652E38F, 6.876433E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float)2.4050125E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)2603011888920540343L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)3.891034E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.6917523E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.3911744E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.2287382E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.6785081E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)242, (uint8_t)190, (uint8_t)56, (uint8_t)137, (uint8_t)16, (uint8_t)89, (uint8_t)92, (uint8_t)140, (uint8_t)85, (uint8_t)88, (uint8_t)251, (uint8_t)3, (uint8_t)218, (uint8_t)6, (uint8_t)185, (uint8_t)199, (uint8_t)199, (uint8_t)154, (uint8_t)206, (uint8_t)77, (uint8_t)200, (uint8_t)182, (uint8_t)142, (uint8_t)73, (uint8_t)162, (uint8_t)126, (uint8_t)15, (uint8_t)42, (uint8_t)16, (uint8_t)46, (uint8_t)137, (uint8_t)210, (uint8_t)116, (uint8_t)52, (uint8_t)219, (uint8_t)51, (uint8_t)169, (uint8_t)132, (uint8_t)213, (uint8_t)216, (uint8_t)22, (uint8_t)205, (uint8_t)148, (uint8_t)211, (uint8_t)15, (uint8_t)124, (uint8_t)9, (uint8_t)44, (uint8_t)201, (uint8_t)171, (uint8_t)108, (uint8_t)207, (uint8_t)46, (uint8_t)44, (uint8_t)144, (uint8_t)109, (uint8_t)82, (uint8_t)200, (uint8_t)108, (uint8_t)229, (uint8_t)221, (uint8_t)14, (uint8_t)121, (uint8_t)241, (uint8_t)60, (uint8_t)144, (uint8_t)179, (uint8_t)89, (uint8_t)92, (uint8_t)136, (uint8_t)2, (uint8_t)94, (uint8_t)158, (uint8_t)221, (uint8_t)123, (uint8_t)176, (uint8_t)142, (uint8_t)138, (uint8_t)117, (uint8_t)239, (uint8_t)3, (uint8_t)86, (uint8_t)238, (uint8_t)138, (uint8_t)233, (uint8_t)9, (uint8_t)245, (uint8_t)237, (uint8_t)213, (uint8_t)119, (uint8_t)96, (uint8_t)22, (uint8_t)31, (uint8_t)179, (uint8_t)246, (uint8_t)46, (uint8_t)180, (uint8_t)250, (uint8_t)223, (uint8_t)180, (uint8_t)103, (uint8_t)196, (uint8_t)53, (uint8_t)127, (uint8_t)196, (uint8_t)7, (uint8_t)40, (uint8_t)230, (uint8_t)255, (uint8_t)72, (uint8_t)211, (uint8_t)52, (uint8_t)216, (uint8_t)239, (uint8_t)88, (uint8_t)129, (uint8_t)221, (uint8_t)23, (uint8_t)230, (uint8_t)128};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        {
            uint8_t uri[] =  {(uint8_t)57, (uint8_t)29, (uint8_t)53, (uint8_t)216, (uint8_t)239, (uint8_t)91, (uint8_t)73, (uint8_t)188, (uint8_t)218, (uint8_t)35, (uint8_t)176, (uint8_t)12, (uint8_t)106, (uint8_t)129, (uint8_t)254, (uint8_t)52, (uint8_t)46, (uint8_t)159, (uint8_t)215, (uint8_t)57, (uint8_t)84, (uint8_t)214, (uint8_t)141, (uint8_t)152, (uint8_t)21, (uint8_t)94, (uint8_t)0, (uint8_t)196, (uint8_t)73, (uint8_t)126, (uint8_t)12, (uint8_t)3, (uint8_t)32, (uint8_t)14, (uint8_t)3, (uint8_t)244, (uint8_t)102, (uint8_t)66, (uint8_t)113, (uint8_t)136, (uint8_t)254, (uint8_t)188, (uint8_t)226, (uint8_t)207, (uint8_t)176, (uint8_t)45, (uint8_t)12, (uint8_t)131, (uint8_t)150, (uint8_t)186, (uint8_t)191, (uint8_t)206, (uint8_t)131, (uint8_t)44, (uint8_t)93, (uint8_t)219, (uint8_t)70, (uint8_t)81, (uint8_t)212, (uint8_t)247, (uint8_t)41, (uint8_t)215, (uint8_t)218, (uint8_t)217, (uint8_t)177, (uint8_t)63, (uint8_t)194, (uint8_t)16, (uint8_t)227, (uint8_t)116, (uint8_t)137, (uint8_t)88, (uint8_t)226, (uint8_t)174, (uint8_t)242, (uint8_t)39, (uint8_t)241, (uint8_t)62, (uint8_t)40, (uint8_t)228, (uint8_t)159, (uint8_t)179, (uint8_t)104, (uint8_t)201, (uint8_t)222, (uint8_t)164, (uint8_t)147, (uint8_t)183, (uint8_t)240, (uint8_t)45, (uint8_t)169, (uint8_t)195, (uint8_t)7, (uint8_t)164, (uint8_t)25, (uint8_t)163, (uint8_t)244, (uint8_t)90, (uint8_t)32, (uint8_t)164, (uint8_t)165, (uint8_t)6, (uint8_t)110, (uint8_t)93, (uint8_t)232, (uint8_t)90, (uint8_t)118, (uint8_t)71, (uint8_t)240, (uint8_t)33, (uint8_t)180, (uint8_t)53, (uint8_t)226, (uint8_t)107, (uint8_t)144, (uint8_t)164, (uint8_t)84, (uint8_t)100, (uint8_t)142, (uint8_t)134};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)2.7894392E38F, PH.base.pack) ;
        p143_press_diff_SET((float)1.9175905E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3491459089L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -21256, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float attitude_q[] =  {2.8645123E38F, -1.7121443E38F, -2.8425497E38F, -3.1939696E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)2959187900089460293L, PH.base.pack) ;
        {
            float rates[] =  {-3.3280743E38F, -3.0572748E37F, -4.236031E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-2.4172198E37F, -7.505534E37F, 1.6607099E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {-6.6749826E37F, -2.2842161E38F, -1.3761412E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -1729436600, PH.base.pack) ;
        p144_alt_SET((float) -1.0555975E38F, PH.base.pack) ;
        p144_lon_SET((int32_t) -230821665, PH.base.pack) ;
        {
            float vel[] =  {4.0510914E37F, 3.2270232E38F, -2.1487009E36F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)5179437851203008076L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_vel_SET((float)9.952708E37F, PH.base.pack) ;
        p146_x_pos_SET((float)1.0118824E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)1.7113772E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.6377846E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {1.4162652E38F, 1.0901792E38F, -2.0308732E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_z_pos_SET((float) -2.1294031E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -1.4291175E38F, PH.base.pack) ;
        p146_y_pos_SET((float) -2.2296485E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -6.0889386E37F, PH.base.pack) ;
        p146_y_vel_SET((float) -2.3504583E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -1.0725178E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -8.978267E37F, PH.base.pack) ;
        {
            float q[] =  {-3.223272E36F, -2.9693133E38F, 1.1791592E38F, -2.1449784E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float)1.5158082E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)5232233134099404015L, PH.base.pack) ;
        p146_airspeed_SET((float)2.9858946E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-2.76961E38F, 2.5937857E38F, -2.0718735E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_id_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)25608, (uint16_t)34307, (uint16_t)17935, (uint16_t)40849, (uint16_t)24566, (uint16_t)23190, (uint16_t)29393, (uint16_t)42848, (uint16_t)21341, (uint16_t)32006};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_consumed_SET((int32_t) -616319523, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -29600, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -106, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -44783661, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)7324, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_flight_sw_version_SET((uint32_t)3453013498L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)189, (uint8_t)105, (uint8_t)203, (uint8_t)46, (uint8_t)4, (uint8_t)210, (uint8_t)42, (uint8_t)68};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)14783, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)39221, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)17, (uint8_t)92, (uint8_t)120, (uint8_t)159, (uint8_t)25, (uint8_t)146, (uint8_t)89, (uint8_t)207};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)31, (uint8_t)238, (uint8_t)125, (uint8_t)152, (uint8_t)131, (uint8_t)80, (uint8_t)64, (uint8_t)192, (uint8_t)71, (uint8_t)210, (uint8_t)57, (uint8_t)48, (uint8_t)34, (uint8_t)202, (uint8_t)150, (uint8_t)118, (uint8_t)141, (uint8_t)39};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_uid_SET((uint64_t)8761676970266768747L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)103, (uint8_t)234, (uint8_t)97, (uint8_t)255, (uint8_t)61, (uint8_t)49, (uint8_t)172, (uint8_t)182};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY), PH.base.pack) ;
        p148_board_version_SET((uint32_t)686299430L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)3124264619L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)3336507060L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)137, &PH) ;
        p149_size_y_SET((float) -3.1350679E38F, PH.base.pack) ;
        p149_size_x_SET((float)2.0522669E38F, PH.base.pack) ;
        p149_z_SET((float)2.7459586E38F, &PH) ;
        p149_angle_x_SET((float) -2.0980995E38F, PH.base.pack) ;
        p149_distance_SET((float) -5.8678786E37F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_x_SET((float) -2.446685E37F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        {
            float q[] =  {5.2795056E37F, -1.69894E37F, -2.577135E38F, -1.8414152E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_angle_y_SET((float) -2.1411052E37F, PH.base.pack) ;
        p149_y_SET((float) -1.8297983E38F, &PH) ;
        p149_time_usec_SET((uint64_t)4485267712437206597L, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_POWER_201(), &PH);
        p201_adc121_cs2_amp_SET((float)3.3988816E38F, PH.base.pack) ;
        p201_adc121_cs1_amp_SET((float)1.0825496E38F, PH.base.pack) ;
        p201_adc121_vspb_volt_SET((float) -2.6967977E38F, PH.base.pack) ;
        p201_adc121_cspb_amp_SET((float)1.0282093E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_POWER_201(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_MPPT_202(), &PH);
        p202_mppt1_status_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p202_mppt1_volt_SET((float)6.981507E37F, PH.base.pack) ;
        p202_mppt_timestamp_SET((uint64_t)9190535815262464431L, PH.base.pack) ;
        p202_mppt3_status_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p202_mppt1_pwm_SET((uint16_t)(uint16_t)39525, PH.base.pack) ;
        p202_mppt1_amp_SET((float) -1.1424517E38F, PH.base.pack) ;
        p202_mppt3_volt_SET((float)2.964E38F, PH.base.pack) ;
        p202_mppt3_pwm_SET((uint16_t)(uint16_t)37452, PH.base.pack) ;
        p202_mppt2_status_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p202_mppt3_amp_SET((float) -1.8708063E38F, PH.base.pack) ;
        p202_mppt2_amp_SET((float) -2.2082907E38F, PH.base.pack) ;
        p202_mppt2_pwm_SET((uint16_t)(uint16_t)25716, PH.base.pack) ;
        p202_mppt2_volt_SET((float)1.1836987E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_MPPT_202(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLCTRL_DATA_203(), &PH);
        p203_p_SET((float) -1.3254494E38F, PH.base.pack) ;
        p203_YawAngle_SET((float) -1.0809843E38F, PH.base.pack) ;
        p203_YawAngleRef_SET((float)3.3649002E38F, PH.base.pack) ;
        p203_uAil_SET((float)3.1332264E38F, PH.base.pack) ;
        p203_aslctrl_mode_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p203_nZ_SET((float) -2.099024E37F, PH.base.pack) ;
        p203_pRef_SET((float)1.2630461E38F, PH.base.pack) ;
        p203_uThrot_SET((float)1.5428256E38F, PH.base.pack) ;
        p203_PitchAngle_SET((float)2.1484739E38F, PH.base.pack) ;
        p203_uRud_SET((float) -2.3189929E37F, PH.base.pack) ;
        p203_SpoilersEngaged_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p203_q_SET((float) -2.960124E38F, PH.base.pack) ;
        p203_hRef_t_SET((float)2.1744495E38F, PH.base.pack) ;
        p203_h_SET((float) -1.6295831E38F, PH.base.pack) ;
        p203_timestamp_SET((uint64_t)4491710795493874348L, PH.base.pack) ;
        p203_RollAngleRef_SET((float) -5.140219E37F, PH.base.pack) ;
        p203_uElev_SET((float) -9.325526E37F, PH.base.pack) ;
        p203_RollAngle_SET((float)3.1133615E37F, PH.base.pack) ;
        p203_PitchAngleRef_SET((float)3.3580436E38F, PH.base.pack) ;
        p203_qRef_SET((float)1.6768969E38F, PH.base.pack) ;
        p203_uThrot2_SET((float)2.3287058E38F, PH.base.pack) ;
        p203_hRef_SET((float)2.3183382E37F, PH.base.pack) ;
        p203_rRef_SET((float)1.0298218E38F, PH.base.pack) ;
        p203_AirspeedRef_SET((float)9.083275E37F, PH.base.pack) ;
        p203_r_SET((float) -1.4112538E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ASLCTRL_DATA_203(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLCTRL_DEBUG_204(), &PH);
        p204_f_4_SET((float)2.111506E38F, PH.base.pack) ;
        p204_f_1_SET((float)3.3193883E38F, PH.base.pack) ;
        p204_f_8_SET((float)1.0079533E38F, PH.base.pack) ;
        p204_f_7_SET((float) -1.7406388E38F, PH.base.pack) ;
        p204_f_5_SET((float)7.4846234E37F, PH.base.pack) ;
        p204_f_6_SET((float)2.3557244E38F, PH.base.pack) ;
        p204_i8_1_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p204_f_2_SET((float)2.1151032E38F, PH.base.pack) ;
        p204_i8_2_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p204_f_3_SET((float) -2.8096646E38F, PH.base.pack) ;
        p204_i32_1_SET((uint32_t)412052952L, PH.base.pack) ;
        c_CommunicationChannel_on_ASLCTRL_DEBUG_204(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLUAV_STATUS_205(), &PH);
        p205_Motor_rpm_SET((float)2.5038503E38F, PH.base.pack) ;
        p205_SATCOM_status_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            uint8_t Servo_status[] =  {(uint8_t)134, (uint8_t)185, (uint8_t)108, (uint8_t)15, (uint8_t)188, (uint8_t)117, (uint8_t)38, (uint8_t)106};
            p205_Servo_status_SET(&Servo_status, 0, PH.base.pack) ;
        }
        p205_LED_status_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        c_CommunicationChannel_on_ASLUAV_STATUS_205(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EKF_EXT_206(), &PH);
        p206_WindDir_SET((float) -1.0262408E38F, PH.base.pack) ;
        p206_Windspeed_SET((float)1.653101E38F, PH.base.pack) ;
        p206_WindZ_SET((float) -2.5826677E38F, PH.base.pack) ;
        p206_alpha_SET((float)2.1177805E38F, PH.base.pack) ;
        p206_beta_SET((float) -1.1307029E38F, PH.base.pack) ;
        p206_Airspeed_SET((float)3.095344E37F, PH.base.pack) ;
        p206_timestamp_SET((uint64_t)2684332372044194070L, PH.base.pack) ;
        c_CommunicationChannel_on_EKF_EXT_206(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASL_OBCTRL_207(), &PH);
        p207_uRud_SET((float) -1.1719743E38F, PH.base.pack) ;
        p207_uThrot_SET((float)2.6145963E38F, PH.base.pack) ;
        p207_obctrl_status_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p207_uElev_SET((float)2.5518677E38F, PH.base.pack) ;
        p207_uAilL_SET((float) -2.8655869E38F, PH.base.pack) ;
        p207_uAilR_SET((float)3.031416E38F, PH.base.pack) ;
        p207_uThrot2_SET((float) -3.2844178E38F, PH.base.pack) ;
        p207_timestamp_SET((uint64_t)2557465763835125436L, PH.base.pack) ;
        c_CommunicationChannel_on_ASL_OBCTRL_207(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_ATMOS_208(), &PH);
        p208_Humidity_SET((float) -2.661444E38F, PH.base.pack) ;
        p208_TempAmbient_SET((float) -2.4332206E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_ATMOS_208(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_BATMON_209(), &PH);
        p209_SoC_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p209_hostfetcontrol_SET((uint16_t)(uint16_t)53224, PH.base.pack) ;
        p209_cellvoltage3_SET((uint16_t)(uint16_t)42733, PH.base.pack) ;
        p209_cellvoltage1_SET((uint16_t)(uint16_t)55579, PH.base.pack) ;
        p209_temperature_SET((float) -2.4388914E38F, PH.base.pack) ;
        p209_cellvoltage6_SET((uint16_t)(uint16_t)29344, PH.base.pack) ;
        p209_serialnumber_SET((uint16_t)(uint16_t)40260, PH.base.pack) ;
        p209_current_SET((int16_t)(int16_t)23345, PH.base.pack) ;
        p209_cellvoltage5_SET((uint16_t)(uint16_t)36948, PH.base.pack) ;
        p209_batterystatus_SET((uint16_t)(uint16_t)11940, PH.base.pack) ;
        p209_cellvoltage2_SET((uint16_t)(uint16_t)26793, PH.base.pack) ;
        p209_cellvoltage4_SET((uint16_t)(uint16_t)47642, PH.base.pack) ;
        p209_voltage_SET((uint16_t)(uint16_t)34228, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_BATMON_209(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FW_SOARING_DATA_210(), &PH);
        p210_xR_SET((float) -2.1299524E38F, PH.base.pack) ;
        p210_xLon_SET((float) -4.4732986E37F, PH.base.pack) ;
        p210_z2_DeltaRoll_SET((float)3.8276794E37F, PH.base.pack) ;
        p210_valid_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p210_DebugVar1_SET((float)4.788306E37F, PH.base.pack) ;
        p210_VarLon_SET((float)3.548662E37F, PH.base.pack) ;
        p210_vSinkExp_SET((float)3.1512313E38F, PH.base.pack) ;
        p210_xW_SET((float)2.204834E38F, PH.base.pack) ;
        p210_DistToSoarPoint_SET((float)2.653032E38F, PH.base.pack) ;
        p210_z1_exp_SET((float)3.0498718E38F, PH.base.pack) ;
        p210_ThermalGSNorth_SET((float) -5.55634E37F, PH.base.pack) ;
        p210_timestampModeChanged_SET((uint64_t)2607393097584597889L, PH.base.pack) ;
        p210_LoiterRadius_SET((float) -2.5514837E38F, PH.base.pack) ;
        p210_xLat_SET((float)2.644896E38F, PH.base.pack) ;
        p210_VarLat_SET((float) -1.0889321E35F, PH.base.pack) ;
        p210_VarR_SET((float) -2.6221537E38F, PH.base.pack) ;
        p210_DebugVar2_SET((float)2.6787867E38F, PH.base.pack) ;
        p210_z2_exp_SET((float) -5.518926E37F, PH.base.pack) ;
        p210_ControlMode_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p210_TSE_dot_SET((float) -1.8207186E38F, PH.base.pack) ;
        p210_ThermalGSEast_SET((float) -2.6215055E37F, PH.base.pack) ;
        p210_z1_LocalUpdraftSpeed_SET((float) -3.2846594E38F, PH.base.pack) ;
        p210_timestamp_SET((uint64_t)2773514599753803499L, PH.base.pack) ;
        p210_VarW_SET((float) -2.2712289E38F, PH.base.pack) ;
        p210_LoiterDirection_SET((float) -1.4252063E38F, PH.base.pack) ;
        c_CommunicationChannel_on_FW_SOARING_DATA_210(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSORPOD_STATUS_211(), &PH);
        p211_recording_nodes_count_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p211_timestamp_SET((uint64_t)6059200872289593965L, PH.base.pack) ;
        p211_visensor_rate_4_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p211_visensor_rate_2_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p211_visensor_rate_3_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p211_cpu_temp_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p211_free_space_SET((uint16_t)(uint16_t)29073, PH.base.pack) ;
        p211_visensor_rate_1_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        c_CommunicationChannel_on_SENSORPOD_STATUS_211(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_POWER_BOARD_212(), &PH);
        p212_pwr_brd_status_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p212_pwr_brd_aux_amp_SET((float) -4.999581E37F, PH.base.pack) ;
        p212_pwr_brd_mot_r_amp_SET((float) -1.802026E38F, PH.base.pack) ;
        p212_pwr_brd_servo_1_amp_SET((float)1.7011138E38F, PH.base.pack) ;
        p212_pwr_brd_system_volt_SET((float)3.3753335E38F, PH.base.pack) ;
        p212_timestamp_SET((uint64_t)4093727713564333L, PH.base.pack) ;
        p212_pwr_brd_mot_l_amp_SET((float) -1.9959605E38F, PH.base.pack) ;
        p212_pwr_brd_servo_4_amp_SET((float) -3.0171104E38F, PH.base.pack) ;
        p212_pwr_brd_led_status_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p212_pwr_brd_servo_3_amp_SET((float) -1.4919936E38F, PH.base.pack) ;
        p212_pwr_brd_servo_2_amp_SET((float) -8.713115E37F, PH.base.pack) ;
        p212_pwr_brd_servo_volt_SET((float)1.35046E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_POWER_BOARD_212(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_tas_ratio_SET((float)2.9261096E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)2.7059195E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)2524065423633256584L, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -1.4829601E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -2.3975277E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)3.0636748E38F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS), PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.4785463E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.0491469E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)1.7424616E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float) -2.5857267E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.1901777E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.2494158E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)7769729986774889362L, PH.base.pack) ;
        p231_vert_accuracy_SET((float)1.8539617E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)9.7859704E36F, PH.base.pack) ;
        p231_wind_z_SET((float)1.5258766E38F, PH.base.pack) ;
        p231_wind_y_SET((float)4.0044809E37F, PH.base.pack) ;
        p231_wind_x_SET((float)2.6094592E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT), PH.base.pack) ;
        p232_alt_SET((float)1.5771093E38F, PH.base.pack) ;
        p232_vn_SET((float) -1.30638E37F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)8294112524983618193L, PH.base.pack) ;
        p232_vdop_SET((float) -1.961859E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -3.1363511E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -3.8020372E36F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)3563689822L, PH.base.pack) ;
        p232_lat_SET((int32_t)1358345590, PH.base.pack) ;
        p232_ve_SET((float) -9.156242E37F, PH.base.pack) ;
        p232_vd_SET((float) -2.1109603E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)2.436052E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p232_hdop_SET((float)3.9833613E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)29581, PH.base.pack) ;
        p232_lon_SET((int32_t)414490353, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)73, (uint8_t)134, (uint8_t)165, (uint8_t)129, (uint8_t)28, (uint8_t)95, (uint8_t)247, (uint8_t)47, (uint8_t)238, (uint8_t)173, (uint8_t)149, (uint8_t)179, (uint8_t)187, (uint8_t)101, (uint8_t)163, (uint8_t)192, (uint8_t)218, (uint8_t)104, (uint8_t)154, (uint8_t)242, (uint8_t)164, (uint8_t)211, (uint8_t)68, (uint8_t)93, (uint8_t)46, (uint8_t)112, (uint8_t)216, (uint8_t)118, (uint8_t)233, (uint8_t)249, (uint8_t)219, (uint8_t)91, (uint8_t)247, (uint8_t)87, (uint8_t)80, (uint8_t)94, (uint8_t)225, (uint8_t)6, (uint8_t)35, (uint8_t)98, (uint8_t)97, (uint8_t)39, (uint8_t)111, (uint8_t)68, (uint8_t)180, (uint8_t)240, (uint8_t)2, (uint8_t)246, (uint8_t)154, (uint8_t)2, (uint8_t)27, (uint8_t)252, (uint8_t)97, (uint8_t)121, (uint8_t)161, (uint8_t)98, (uint8_t)175, (uint8_t)230, (uint8_t)187, (uint8_t)144, (uint8_t)89, (uint8_t)193, (uint8_t)41, (uint8_t)186, (uint8_t)74, (uint8_t)98, (uint8_t)163, (uint8_t)240, (uint8_t)142, (uint8_t)186, (uint8_t)220, (uint8_t)178, (uint8_t)42, (uint8_t)198, (uint8_t)57, (uint8_t)111, (uint8_t)80, (uint8_t)14, (uint8_t)219, (uint8_t)29, (uint8_t)228, (uint8_t)210, (uint8_t)157, (uint8_t)126, (uint8_t)220, (uint8_t)177, (uint8_t)29, (uint8_t)59, (uint8_t)221, (uint8_t)190, (uint8_t)214, (uint8_t)146, (uint8_t)16, (uint8_t)68, (uint8_t)182, (uint8_t)140, (uint8_t)61, (uint8_t)76, (uint8_t)203, (uint8_t)221, (uint8_t)8, (uint8_t)248, (uint8_t)191, (uint8_t)11, (uint8_t)137, (uint8_t)60, (uint8_t)179, (uint8_t)21, (uint8_t)153, (uint8_t)100, (uint8_t)15, (uint8_t)211, (uint8_t)82, (uint8_t)25, (uint8_t)35, (uint8_t)22, (uint8_t)38, (uint8_t)97, (uint8_t)218, (uint8_t)234, (uint8_t)98, (uint8_t)18, (uint8_t)202, (uint8_t)60, (uint8_t)183, (uint8_t)205, (uint8_t)99, (uint8_t)48, (uint8_t)74, (uint8_t)71, (uint8_t)0, (uint8_t)225, (uint8_t)125, (uint8_t)154, (uint8_t)90, (uint8_t)217, (uint8_t)171, (uint8_t)192, (uint8_t)173, (uint8_t)126, (uint8_t)246, (uint8_t)68, (uint8_t)127, (uint8_t)70, (uint8_t)133, (uint8_t)209, (uint8_t)33, (uint8_t)180, (uint8_t)111, (uint8_t)225, (uint8_t)142, (uint8_t)38, (uint8_t)129, (uint8_t)173, (uint8_t)192, (uint8_t)154, (uint8_t)154, (uint8_t)79, (uint8_t)68, (uint8_t)185, (uint8_t)133, (uint8_t)31, (uint8_t)213, (uint8_t)212, (uint8_t)9, (uint8_t)119, (uint8_t)93, (uint8_t)149, (uint8_t)36, (uint8_t)157, (uint8_t)144, (uint8_t)86, (uint8_t)6, (uint8_t)217, (uint8_t)116, (uint8_t)11, (uint8_t)93, (uint8_t)24, (uint8_t)152, (uint8_t)77};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_wp_num_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -47, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)45714, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)32711, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -30417, PH.base.pack) ;
        p234_latitude_SET((int32_t)385162917, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED), PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)25568, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)63, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p234_longitude_SET((int32_t)1208554210, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)49498, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)52, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -3129, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)9, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)171625830L, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -30000, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)1300520494146345422L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)1666651833L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.9628302E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2740700348L, PH.base.pack) ;
        p241_vibration_z_SET((float) -3.0472197E38F, PH.base.pack) ;
        p241_vibration_y_SET((float)5.671594E37F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3966562888L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {2.0004384E38F, 4.478343E37F, 3.3484204E38F, -2.0006607E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_x_SET((float) -3.2342199E38F, PH.base.pack) ;
        p242_y_SET((float) -1.521178E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)6802636397191708908L, &PH) ;
        p242_approach_y_SET((float)2.9124589E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)1381063527, PH.base.pack) ;
        p242_longitude_SET((int32_t)906994020, PH.base.pack) ;
        p242_x_SET((float) -2.3406479E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.748249E38F, PH.base.pack) ;
        p242_z_SET((float)3.4000878E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -649148916, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_latitude_SET((int32_t) -1179617773, PH.base.pack) ;
        p243_z_SET((float) -2.5981996E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -878627601, PH.base.pack) ;
        p243_approach_x_SET((float)1.1935076E37F, PH.base.pack) ;
        p243_y_SET((float) -2.4765933E37F, PH.base.pack) ;
        {
            float q[] =  {-1.0148291E38F, -5.1825223E37F, -2.7102761E38F, 2.4851288E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_z_SET((float)1.0197115E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)4251498827834386883L, &PH) ;
        p243_x_SET((float) -2.1744915E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)676404980, PH.base.pack) ;
        p243_approach_y_SET((float)3.0804688E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)24757, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -1623233171, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_heading_SET((uint16_t)(uint16_t)14413, PH.base.pack) ;
        p246_lat_SET((int32_t)779559442, PH.base.pack) ;
        p246_altitude_SET((int32_t)287418753, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY), PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)1066831238L, PH.base.pack) ;
        p246_lon_SET((int32_t)317354840, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)45907, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)1099, PH.base.pack) ;
        {
            char16_t* callsign = u"Ejgfr";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)29737, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -6.353918E37F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -3.2725623E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)71641241L, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)4.577144E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)233, (uint8_t)207, (uint8_t)34, (uint8_t)207, (uint8_t)199, (uint8_t)17, (uint8_t)158, (uint8_t)50, (uint8_t)86, (uint8_t)244, (uint8_t)152, (uint8_t)100, (uint8_t)67, (uint8_t)1, (uint8_t)91, (uint8_t)113, (uint8_t)23, (uint8_t)199, (uint8_t)49, (uint8_t)182, (uint8_t)247, (uint8_t)243, (uint8_t)152, (uint8_t)194, (uint8_t)65, (uint8_t)153, (uint8_t)178, (uint8_t)142, (uint8_t)46, (uint8_t)38, (uint8_t)252, (uint8_t)163, (uint8_t)3, (uint8_t)97, (uint8_t)210, (uint8_t)200, (uint8_t)12, (uint8_t)232, (uint8_t)205, (uint8_t)225, (uint8_t)38, (uint8_t)183, (uint8_t)247, (uint8_t)85, (uint8_t)218, (uint8_t)211, (uint8_t)65, (uint8_t)245, (uint8_t)77, (uint8_t)71, (uint8_t)76, (uint8_t)70, (uint8_t)44, (uint8_t)88, (uint8_t)115, (uint8_t)202, (uint8_t)220, (uint8_t)116, (uint8_t)54, (uint8_t)153, (uint8_t)10, (uint8_t)148, (uint8_t)224, (uint8_t)127, (uint8_t)55, (uint8_t)1, (uint8_t)67, (uint8_t)204, (uint8_t)119, (uint8_t)130, (uint8_t)92, (uint8_t)219, (uint8_t)228, (uint8_t)199, (uint8_t)96, (uint8_t)110, (uint8_t)192, (uint8_t)240, (uint8_t)97, (uint8_t)79, (uint8_t)169, (uint8_t)245, (uint8_t)15, (uint8_t)58, (uint8_t)39, (uint8_t)229, (uint8_t)79, (uint8_t)145, (uint8_t)117, (uint8_t)204, (uint8_t)54, (uint8_t)80, (uint8_t)168, (uint8_t)179, (uint8_t)12, (uint8_t)76, (uint8_t)238, (uint8_t)47, (uint8_t)186, (uint8_t)155, (uint8_t)249, (uint8_t)80, (uint8_t)68, (uint8_t)97, (uint8_t)228, (uint8_t)155, (uint8_t)171, (uint8_t)5, (uint8_t)205, (uint8_t)14, (uint8_t)226, (uint8_t)196, (uint8_t)134, (uint8_t)103, (uint8_t)199, (uint8_t)34, (uint8_t)98, (uint8_t)79, (uint8_t)65, (uint8_t)164, (uint8_t)182, (uint8_t)89, (uint8_t)194, (uint8_t)104, (uint8_t)194, (uint8_t)193, (uint8_t)81, (uint8_t)228, (uint8_t)135, (uint8_t)206, (uint8_t)37, (uint8_t)134, (uint8_t)110, (uint8_t)231, (uint8_t)73, (uint8_t)178, (uint8_t)228, (uint8_t)111, (uint8_t)136, (uint8_t)102, (uint8_t)81, (uint8_t)112, (uint8_t)128, (uint8_t)129, (uint8_t)247, (uint8_t)134, (uint8_t)87, (uint8_t)220, (uint8_t)116, (uint8_t)28, (uint8_t)15, (uint8_t)92, (uint8_t)64, (uint8_t)105, (uint8_t)71, (uint8_t)105, (uint8_t)234, (uint8_t)73, (uint8_t)225, (uint8_t)73, (uint8_t)201, (uint8_t)42, (uint8_t)1, (uint8_t)92, (uint8_t)118, (uint8_t)168, (uint8_t)203, (uint8_t)180, (uint8_t)95, (uint8_t)30, (uint8_t)171, (uint8_t)50, (uint8_t)62, (uint8_t)132, (uint8_t)184, (uint8_t)196, (uint8_t)60, (uint8_t)204, (uint8_t)32, (uint8_t)123, (uint8_t)189, (uint8_t)97, (uint8_t)200, (uint8_t)97, (uint8_t)14, (uint8_t)237, (uint8_t)11, (uint8_t)174, (uint8_t)25, (uint8_t)255, (uint8_t)139, (uint8_t)83, (uint8_t)205, (uint8_t)213, (uint8_t)29, (uint8_t)93, (uint8_t)135, (uint8_t)152, (uint8_t)225, (uint8_t)181, (uint8_t)247, (uint8_t)70, (uint8_t)220, (uint8_t)65, (uint8_t)78, (uint8_t)86, (uint8_t)165, (uint8_t)73, (uint8_t)230, (uint8_t)51, (uint8_t)81, (uint8_t)99, (uint8_t)41, (uint8_t)106, (uint8_t)111, (uint8_t)182, (uint8_t)26, (uint8_t)250, (uint8_t)72, (uint8_t)153, (uint8_t)122, (uint8_t)157, (uint8_t)153, (uint8_t)129, (uint8_t)151, (uint8_t)165, (uint8_t)107, (uint8_t)158, (uint8_t)177, (uint8_t)179, (uint8_t)186, (uint8_t)31, (uint8_t)41, (uint8_t)231, (uint8_t)127, (uint8_t)37, (uint8_t)101, (uint8_t)250, (uint8_t)194, (uint8_t)57, (uint8_t)130, (uint8_t)57, (uint8_t)224, (uint8_t)77, (uint8_t)62, (uint8_t)185, (uint8_t)12, (uint8_t)100, (uint8_t)37};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)42892, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)32398, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -88, (int8_t)121, (int8_t) -24, (int8_t) -43, (int8_t) -8, (int8_t)123, (int8_t) -63, (int8_t)37, (int8_t) -7, (int8_t) -112, (int8_t) -26, (int8_t)52, (int8_t)125, (int8_t)36, (int8_t)4, (int8_t)85, (int8_t) -117, (int8_t) -83, (int8_t)110, (int8_t) -104, (int8_t) -47, (int8_t) -78, (int8_t)14, (int8_t)97, (int8_t)68, (int8_t)43, (int8_t) -56, (int8_t)36, (int8_t)112, (int8_t)38, (int8_t) -25, (int8_t)97};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_time_usec_SET((uint64_t)6123168229078066334L, PH.base.pack) ;
        {
            char16_t* name = u"Tallz";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -2.6506793E38F, PH.base.pack) ;
        p250_y_SET((float)1.0010957E37F, PH.base.pack) ;
        p250_x_SET((float)3.1708004E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)2226984583L, PH.base.pack) ;
        p251_value_SET((float)8.930372E37F, PH.base.pack) ;
        {
            char16_t* name = u"cbzg";
            p251_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)2069173558, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)3808247146L, PH.base.pack) ;
        {
            char16_t* name = u"t";
            p252_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"zvGhxsmAqRmfhrxzePEuffcjf";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ERROR, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p254_value_SET((float)1.2237344E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)3538000375L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)5725248189427489857L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)101, (uint8_t)64, (uint8_t)70, (uint8_t)239, (uint8_t)86, (uint8_t)251, (uint8_t)166, (uint8_t)132, (uint8_t)78, (uint8_t)6, (uint8_t)204, (uint8_t)12, (uint8_t)122, (uint8_t)57, (uint8_t)71, (uint8_t)147, (uint8_t)101, (uint8_t)72, (uint8_t)187, (uint8_t)208, (uint8_t)227, (uint8_t)167, (uint8_t)67, (uint8_t)65, (uint8_t)210, (uint8_t)191, (uint8_t)25, (uint8_t)149, (uint8_t)189, (uint8_t)238, (uint8_t)113, (uint8_t)254};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)568994889L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)3766544226L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"tQmhqvyXcm";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        {
            uint8_t model_name[] =  {(uint8_t)238, (uint8_t)203, (uint8_t)124, (uint8_t)165, (uint8_t)131, (uint8_t)31, (uint8_t)2, (uint8_t)235, (uint8_t)252, (uint8_t)211, (uint8_t)187, (uint8_t)112, (uint8_t)22, (uint8_t)133, (uint8_t)57, (uint8_t)60, (uint8_t)209, (uint8_t)123, (uint8_t)24, (uint8_t)56, (uint8_t)26, (uint8_t)186, (uint8_t)57, (uint8_t)116, (uint8_t)24, (uint8_t)248, (uint8_t)131, (uint8_t)176, (uint8_t)8, (uint8_t)241, (uint8_t)192, (uint8_t)149};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)2.5127249E38F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)735082356L, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -8.637447E37F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)34053, PH.base.pack) ;
        p259_focal_length_SET((float) -1.5111609E38F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)39, (uint8_t)125, (uint8_t)188, (uint8_t)23, (uint8_t)95, (uint8_t)132, (uint8_t)197, (uint8_t)172, (uint8_t)106, (uint8_t)131, (uint8_t)80, (uint8_t)11, (uint8_t)16, (uint8_t)110, (uint8_t)218, (uint8_t)156, (uint8_t)188, (uint8_t)209, (uint8_t)59, (uint8_t)108, (uint8_t)188, (uint8_t)244, (uint8_t)59, (uint8_t)163, (uint8_t)254, (uint8_t)209, (uint8_t)85, (uint8_t)104, (uint8_t)146, (uint8_t)252, (uint8_t)71, (uint8_t)229};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)56177, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)3959237267L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"pgywitvbnysluZcoacmqlcnsWnhagjwmtpcdofczileqjlfyujbhianwxzaNcdyxiaarqqtcvhkghsuetsdfocfJrvtdJlDzyLfHDsrmuwtxtmvrshpygyuwmo";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE), PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)49422, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)3097219262L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_write_speed_SET((float) -8.0691736E37F, PH.base.pack) ;
        p261_read_speed_SET((float) -1.2033737E38F, PH.base.pack) ;
        p261_used_capacity_SET((float) -3.2368923E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p261_total_capacity_SET((float)2.710221E37F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p261_available_capacity_SET((float) -2.747017E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1765955992L, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float) -5.2828685E37F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3524219870L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)1143610562L, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.616006E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lat_SET((int32_t) -1802123384, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)1108674768072043193L, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2382079625L, PH.base.pack) ;
        {
            char16_t* file_url = u"XloooGbhamimetWxQnctJyrMgwcseksxksokvZzptoipoolvgtrvuyshdcyvtzdlryyzdztjssxufvoflzOaanbgrmExxVfrykwezqxskzxugqbdrmlcolrdsmsbfjAmuoocrcbfrcmbahuyzaqwayykqzdpnadaUlbhcwagRdhqihgqytksvyzecohub";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_lon_SET((int32_t) -1452102215, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -125, PH.base.pack) ;
        {
            float q[] =  {9.193425E37F, 1.0792307E38F, 4.771927E37F, 9.627906E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_image_index_SET((int32_t)1673363567, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -80975482, PH.base.pack) ;
        p263_alt_SET((int32_t) -780236504, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)1470937663765612509L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)3612401160507255168L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)133502443L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)7097306137198587484L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float)1.3293881E38F, PH.base.pack) ;
        p265_roll_SET((float)2.2092197E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)2072776420L, PH.base.pack) ;
        p265_pitch_SET((float) -4.666561E37F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)57434, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)40, (uint8_t)8, (uint8_t)246, (uint8_t)49, (uint8_t)171, (uint8_t)224, (uint8_t)251, (uint8_t)221, (uint8_t)21, (uint8_t)30, (uint8_t)52, (uint8_t)54, (uint8_t)199, (uint8_t)88, (uint8_t)100, (uint8_t)165, (uint8_t)10, (uint8_t)194, (uint8_t)180, (uint8_t)108, (uint8_t)86, (uint8_t)117, (uint8_t)189, (uint8_t)40, (uint8_t)154, (uint8_t)78, (uint8_t)170, (uint8_t)230, (uint8_t)209, (uint8_t)41, (uint8_t)84, (uint8_t)63, (uint8_t)120, (uint8_t)133, (uint8_t)151, (uint8_t)247, (uint8_t)82, (uint8_t)209, (uint8_t)245, (uint8_t)157, (uint8_t)182, (uint8_t)230, (uint8_t)97, (uint8_t)77, (uint8_t)172, (uint8_t)84, (uint8_t)184, (uint8_t)117, (uint8_t)27, (uint8_t)136, (uint8_t)113, (uint8_t)101, (uint8_t)32, (uint8_t)243, (uint8_t)42, (uint8_t)202, (uint8_t)65, (uint8_t)74, (uint8_t)88, (uint8_t)115, (uint8_t)128, (uint8_t)122, (uint8_t)68, (uint8_t)126, (uint8_t)238, (uint8_t)237, (uint8_t)20, (uint8_t)85, (uint8_t)52, (uint8_t)5, (uint8_t)161, (uint8_t)78, (uint8_t)104, (uint8_t)94, (uint8_t)61, (uint8_t)196, (uint8_t)193, (uint8_t)248, (uint8_t)71, (uint8_t)11, (uint8_t)206, (uint8_t)152, (uint8_t)181, (uint8_t)45, (uint8_t)118, (uint8_t)56, (uint8_t)124, (uint8_t)210, (uint8_t)148, (uint8_t)63, (uint8_t)104, (uint8_t)180, (uint8_t)37, (uint8_t)107, (uint8_t)201, (uint8_t)164, (uint8_t)51, (uint8_t)33, (uint8_t)193, (uint8_t)86, (uint8_t)177, (uint8_t)21, (uint8_t)98, (uint8_t)186, (uint8_t)227, (uint8_t)179, (uint8_t)41, (uint8_t)220, (uint8_t)101, (uint8_t)96, (uint8_t)204, (uint8_t)155, (uint8_t)95, (uint8_t)122, (uint8_t)5, (uint8_t)44, (uint8_t)101, (uint8_t)47, (uint8_t)240, (uint8_t)185, (uint8_t)192, (uint8_t)208, (uint8_t)186, (uint8_t)220, (uint8_t)114, (uint8_t)157, (uint8_t)200, (uint8_t)230, (uint8_t)83, (uint8_t)102, (uint8_t)132, (uint8_t)37, (uint8_t)1, (uint8_t)82, (uint8_t)253, (uint8_t)154, (uint8_t)42, (uint8_t)21, (uint8_t)212, (uint8_t)107, (uint8_t)56, (uint8_t)232, (uint8_t)129, (uint8_t)5, (uint8_t)43, (uint8_t)238, (uint8_t)190, (uint8_t)39, (uint8_t)192, (uint8_t)211, (uint8_t)11, (uint8_t)70, (uint8_t)133, (uint8_t)203, (uint8_t)193, (uint8_t)217, (uint8_t)82, (uint8_t)218, (uint8_t)158, (uint8_t)41, (uint8_t)9, (uint8_t)48, (uint8_t)116, (uint8_t)222, (uint8_t)167, (uint8_t)45, (uint8_t)179, (uint8_t)30, (uint8_t)101, (uint8_t)37, (uint8_t)242, (uint8_t)159, (uint8_t)232, (uint8_t)186, (uint8_t)86, (uint8_t)102, (uint8_t)146, (uint8_t)86, (uint8_t)214, (uint8_t)129, (uint8_t)227, (uint8_t)139, (uint8_t)134, (uint8_t)27, (uint8_t)166, (uint8_t)174, (uint8_t)121, (uint8_t)61, (uint8_t)180, (uint8_t)166, (uint8_t)56, (uint8_t)26, (uint8_t)73, (uint8_t)75, (uint8_t)18, (uint8_t)5, (uint8_t)244, (uint8_t)77, (uint8_t)82, (uint8_t)157, (uint8_t)43, (uint8_t)193, (uint8_t)222, (uint8_t)84, (uint8_t)92, (uint8_t)215, (uint8_t)237, (uint8_t)180, (uint8_t)94, (uint8_t)110, (uint8_t)8, (uint8_t)29, (uint8_t)238, (uint8_t)21, (uint8_t)231, (uint8_t)190, (uint8_t)112, (uint8_t)28, (uint8_t)201, (uint8_t)57, (uint8_t)28, (uint8_t)70, (uint8_t)11, (uint8_t)223, (uint8_t)174, (uint8_t)28, (uint8_t)84, (uint8_t)169, (uint8_t)0, (uint8_t)120, (uint8_t)130, (uint8_t)238, (uint8_t)168, (uint8_t)1, (uint8_t)21, (uint8_t)68, (uint8_t)181, (uint8_t)109, (uint8_t)196, (uint8_t)193, (uint8_t)67, (uint8_t)12, (uint8_t)231, (uint8_t)50, (uint8_t)91, (uint8_t)63, (uint8_t)245, (uint8_t)147, (uint8_t)148};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)39398, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)220, (uint8_t)209, (uint8_t)58, (uint8_t)185, (uint8_t)224, (uint8_t)253, (uint8_t)184, (uint8_t)52, (uint8_t)245, (uint8_t)168, (uint8_t)215, (uint8_t)234, (uint8_t)85, (uint8_t)200, (uint8_t)10, (uint8_t)213, (uint8_t)119, (uint8_t)194, (uint8_t)153, (uint8_t)214, (uint8_t)44, (uint8_t)124, (uint8_t)233, (uint8_t)76, (uint8_t)152, (uint8_t)211, (uint8_t)120, (uint8_t)73, (uint8_t)225, (uint8_t)252, (uint8_t)206, (uint8_t)9, (uint8_t)112, (uint8_t)146, (uint8_t)86, (uint8_t)7, (uint8_t)106, (uint8_t)11, (uint8_t)208, (uint8_t)64, (uint8_t)135, (uint8_t)116, (uint8_t)137, (uint8_t)51, (uint8_t)64, (uint8_t)251, (uint8_t)79, (uint8_t)207, (uint8_t)73, (uint8_t)239, (uint8_t)153, (uint8_t)26, (uint8_t)175, (uint8_t)252, (uint8_t)55, (uint8_t)61, (uint8_t)164, (uint8_t)62, (uint8_t)55, (uint8_t)1, (uint8_t)72, (uint8_t)16, (uint8_t)165, (uint8_t)251, (uint8_t)59, (uint8_t)230, (uint8_t)134, (uint8_t)33, (uint8_t)215, (uint8_t)130, (uint8_t)159, (uint8_t)180, (uint8_t)180, (uint8_t)144, (uint8_t)8, (uint8_t)86, (uint8_t)147, (uint8_t)201, (uint8_t)235, (uint8_t)141, (uint8_t)133, (uint8_t)166, (uint8_t)251, (uint8_t)2, (uint8_t)239, (uint8_t)3, (uint8_t)251, (uint8_t)178, (uint8_t)121, (uint8_t)55, (uint8_t)37, (uint8_t)3, (uint8_t)224, (uint8_t)22, (uint8_t)240, (uint8_t)226, (uint8_t)31, (uint8_t)42, (uint8_t)30, (uint8_t)188, (uint8_t)127, (uint8_t)76, (uint8_t)29, (uint8_t)47, (uint8_t)142, (uint8_t)0, (uint8_t)112, (uint8_t)154, (uint8_t)109, (uint8_t)42, (uint8_t)151, (uint8_t)174, (uint8_t)255, (uint8_t)77, (uint8_t)95, (uint8_t)66, (uint8_t)69, (uint8_t)181, (uint8_t)219, (uint8_t)150, (uint8_t)133, (uint8_t)11, (uint8_t)37, (uint8_t)234, (uint8_t)36, (uint8_t)140, (uint8_t)60, (uint8_t)169, (uint8_t)207, (uint8_t)121, (uint8_t)200, (uint8_t)7, (uint8_t)30, (uint8_t)87, (uint8_t)237, (uint8_t)253, (uint8_t)101, (uint8_t)89, (uint8_t)227, (uint8_t)150, (uint8_t)138, (uint8_t)68, (uint8_t)110, (uint8_t)249, (uint8_t)35, (uint8_t)228, (uint8_t)196, (uint8_t)5, (uint8_t)221, (uint8_t)174, (uint8_t)170, (uint8_t)14, (uint8_t)118, (uint8_t)216, (uint8_t)235, (uint8_t)63, (uint8_t)54, (uint8_t)13, (uint8_t)5, (uint8_t)189, (uint8_t)79, (uint8_t)20, (uint8_t)162, (uint8_t)240, (uint8_t)33, (uint8_t)97, (uint8_t)176, (uint8_t)180, (uint8_t)165, (uint8_t)195, (uint8_t)17, (uint8_t)155, (uint8_t)219, (uint8_t)204, (uint8_t)236, (uint8_t)185, (uint8_t)241, (uint8_t)58, (uint8_t)4, (uint8_t)126, (uint8_t)122, (uint8_t)99, (uint8_t)182, (uint8_t)221, (uint8_t)98, (uint8_t)75, (uint8_t)165, (uint8_t)74, (uint8_t)89, (uint8_t)126, (uint8_t)82, (uint8_t)50, (uint8_t)96, (uint8_t)139, (uint8_t)57, (uint8_t)105, (uint8_t)229, (uint8_t)247, (uint8_t)10, (uint8_t)36, (uint8_t)62, (uint8_t)206, (uint8_t)173, (uint8_t)95, (uint8_t)248, (uint8_t)156, (uint8_t)45, (uint8_t)176, (uint8_t)96, (uint8_t)155, (uint8_t)26, (uint8_t)59, (uint8_t)78, (uint8_t)120, (uint8_t)116, (uint8_t)16, (uint8_t)144, (uint8_t)44, (uint8_t)142, (uint8_t)121, (uint8_t)130, (uint8_t)144, (uint8_t)244, (uint8_t)43, (uint8_t)252, (uint8_t)183, (uint8_t)196, (uint8_t)18, (uint8_t)150, (uint8_t)113, (uint8_t)83, (uint8_t)100, (uint8_t)228, (uint8_t)7, (uint8_t)57, (uint8_t)161, (uint8_t)107, (uint8_t)124, (uint8_t)152, (uint8_t)53, (uint8_t)113, (uint8_t)5, (uint8_t)1, (uint8_t)231, (uint8_t)252, (uint8_t)111, (uint8_t)222, (uint8_t)21, (uint8_t)161};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)44322, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_status_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)24356, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)7597, PH.base.pack) ;
        p269_framerate_SET((float) -7.688928E37F, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)32247602L, PH.base.pack) ;
        {
            char16_t* uri = u"zsmEfkPpgaPDuahhgUjydyueozmcdPfoSnptbftjzk";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_v_SET((uint16_t)(uint16_t)17665, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_rotation_SET((uint16_t)(uint16_t)36019, PH.base.pack) ;
        p270_framerate_SET((float) -2.163217E38F, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)1414570100L, PH.base.pack) ;
        {
            char16_t* uri = u"qeglumqjkfdzoakikwifojhstcrxkobbtoMnssnYuecwjqsduofySnOfionijmfczmpfvdxninxaRwbyfspkiolkoueelyumjobomgg";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_resolution_v_SET((uint16_t)(uint16_t)7535, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)61022, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"jtcnvsmwkox";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"EsahcfgyrcfvxqmfwcaxwEmeT";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)5980, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)31073, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)122, (uint8_t)83, (uint8_t)149, (uint8_t)21, (uint8_t)148, (uint8_t)110, (uint8_t)190, (uint8_t)252};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t library_version_hash[] =  {(uint8_t)75, (uint8_t)125, (uint8_t)195, (uint8_t)216, (uint8_t)247, (uint8_t)57, (uint8_t)9, (uint8_t)100};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)52494, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)2091706697018370340L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)49303, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)2820785682L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)3056618840L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)6693162421961716222L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)118, (uint8_t)72, (uint8_t)67, (uint8_t)183, (uint8_t)55, (uint8_t)98, (uint8_t)64, (uint8_t)23, (uint8_t)223, (uint8_t)166, (uint8_t)242, (uint8_t)227, (uint8_t)215, (uint8_t)113, (uint8_t)96, (uint8_t)221};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        {
            char16_t* name = u"olgysIxnesyfh";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3969457753L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        {
            char16_t* param_id = u"uuw";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_component_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t) -32115, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)5004, PH.base.pack) ;
        {
            char16_t* param_value = u"ANvnvdgvitehfznqgufHiypcYxixoygittimqxnLlcSxgjFznykugrnMmhbjtwyRbkztalskjftWbuzgjnkLmthvispfpu";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)38912, PH.base.pack) ;
        {
            char16_t* param_id = u"fmj";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_value = u"ZOBUsqhuslqbnjdbEwtvggkpbopx";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        {
            char16_t* param_id = u"ro";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT16, PH.base.pack) ;
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"wuy";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"zxoojkmjmesiHqpWkyjipYUZwaoqnrGQVifYyyjlfihrgmQcAjgarxefowiveonnZoqnfdxaavtcgyqvrnkceipqtovaowbechsvfzukdpTibvvvkvntymbksf";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_increment_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)33736, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)171, (uint16_t)37937, (uint16_t)16589, (uint16_t)42419, (uint16_t)8838, (uint16_t)58978, (uint16_t)18491, (uint16_t)1150, (uint16_t)54738, (uint16_t)43603, (uint16_t)12310, (uint16_t)27548, (uint16_t)32697, (uint16_t)53435, (uint16_t)38194, (uint16_t)62787, (uint16_t)5690, (uint16_t)16856, (uint16_t)29125, (uint16_t)53868, (uint16_t)33883, (uint16_t)5286, (uint16_t)5504, (uint16_t)21238, (uint16_t)14943, (uint16_t)10647, (uint16_t)20367, (uint16_t)51027, (uint16_t)63649, (uint16_t)4687, (uint16_t)10210, (uint16_t)13805, (uint16_t)43335, (uint16_t)40771, (uint16_t)63471, (uint16_t)37288, (uint16_t)43728, (uint16_t)5115, (uint16_t)35754, (uint16_t)37882, (uint16_t)52708, (uint16_t)28610, (uint16_t)47219, (uint16_t)14438, (uint16_t)43095, (uint16_t)41531, (uint16_t)38369, (uint16_t)65090, (uint16_t)43006, (uint16_t)55386, (uint16_t)29517, (uint16_t)3129, (uint16_t)7754, (uint16_t)10695, (uint16_t)48755, (uint16_t)7479, (uint16_t)25269, (uint16_t)43557, (uint16_t)60236, (uint16_t)49178, (uint16_t)62601, (uint16_t)5954, (uint16_t)54347, (uint16_t)35535, (uint16_t)45526, (uint16_t)34299, (uint16_t)9805, (uint16_t)21052, (uint16_t)35254, (uint16_t)1986, (uint16_t)42050, (uint16_t)12890};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)8439960605934931700L, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)32103, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}



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
    return  _en__v(get_bits(data, 40, 4));
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
    switch(get_bits(data, 276, 7))
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
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_MISSION_TYPE p39_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 283, 3))
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
    switch(get_bits(data, 276, 7))
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
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_MISSION_TYPE p73_mission_type_GET(Pack * src)//Mission type, see MAV_MISSION_TYPE
{
    uint8_t * data = src->data;
    switch(get_bits(data, 283, 3))
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
    switch(get_bits(data, 260, 7))
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
    switch(get_bits(data, 248, 7))
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
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
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
*	should have the UINT16_MAX value*/
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
*	energy consumption estimat*/
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
INLINER void p150_ar_u16_SET(uint16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p150_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p150_v1_SET(uint8_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  24);
}
INLINER void p150_ar_i8_SET(int8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  25, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
INLINER void p150_ar_u8_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  29, src_max = pos + 4; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
Pack * c_TEST_Channel_new_ARRAY_TEST_0_150()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 150));
};
INLINER void p151_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
Pack * c_TEST_Channel_new_ARRAY_TEST_1_151()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 151));
};
INLINER void p153_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p153_v_SET(uint8_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
Pack * c_TEST_Channel_new_ARRAY_TEST_3_153()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 153));
};
INLINER void p154_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p154_v_SET(uint8_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  16);
}
Pack * c_TEST_Channel_new_ARRAY_TEST_4_154()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 154));
};
INLINER void p155_c1_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Value array
{
    if(dst->base.field_bit != 0 && insert_field(dst, 0, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p155_c1_SET_(char16_t*  src, Bounds_Inside * dst) {p155_c1_SET(src, 0, strlen16(src), dst);}
INLINER void p155_c2_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Value array
{
    if(dst->base.field_bit != 1 && insert_field(dst, 1, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p155_c2_SET_(char16_t*  src, Bounds_Inside * dst) {p155_c2_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_ARRAY_TEST_5_155()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 155));
};
INLINER void p156_v2_SET(uint16_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p156_ar_u16_SET(uint16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  2, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p156_v3_SET(uint32_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  6);
}
INLINER void p156_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  10, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p156_v1_SET(uint8_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  18);
}
INLINER void p156_ar_i32_SET(int32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  19, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes((uint32_t)(src[pos]), 4, data,  BYTE);
}
INLINER void p156_ar_i16_SET(int16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  27, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p156_ar_u8_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  31, src_max = pos + 2; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p156_ar_i8_SET(int8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  33, src_max = pos + 2; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
INLINER void p156_ar_d_SET(double*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  35, src_max = pos + 2; pos < src_max; pos++, BYTE += 8)
        set_bytes(doubleToLongBits(src[pos]), 8, data,  BYTE);
}
INLINER void p156_ar_f_SET(float*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  51, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p156_ar_c_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Value array
{
    if(dst->base.field_bit != 472 && insert_field(dst, 472, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p156_ar_c_SET_(char16_t*  src, Bounds_Inside * dst) {p156_ar_c_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_ARRAY_TEST_6_156()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 156));
};
INLINER void p157_ar_u16_SET(uint16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p157_ar_u32_SET(uint32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  4, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes((src[pos]), 4, data,  BYTE);
}
INLINER void p157_ar_d_SET(double*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  12, src_max = pos + 2; pos < src_max; pos++, BYTE += 8)
        set_bytes(doubleToLongBits(src[pos]), 8, data,  BYTE);
}
INLINER void p157_ar_f_SET(float*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  28, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
INLINER void p157_ar_i32_SET(int32_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  36, src_max = pos + 2; pos < src_max; pos++, BYTE += 4)
        set_bytes((uint32_t)(src[pos]), 4, data,  BYTE);
}
INLINER void p157_ar_i16_SET(int16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  44, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p157_ar_u8_SET(uint8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  48, src_max = pos + 2; pos < src_max; pos++, BYTE += 1)
        set_bytes((src[pos]), 1, data,  BYTE);
}
INLINER void p157_ar_i8_SET(int8_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  50, src_max = pos + 2; pos < src_max; pos++, BYTE += 1)
        set_bytes((uint8_t)(src[pos]), 1, data,  BYTE);
}
INLINER void p157_ar_c_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Value array
{
    if(dst->base.field_bit != 416 && insert_field(dst, 416, items) ||
            ! try_visit_item(dst, 0)) insert_item(dst, 0, items);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + dst->items; pos < src_max; pos++, BYTE += 2)
        set_bytes((uint16_t)(src[pos]), 2, data,  BYTE);
}
INLINER void p157_ar_c_SET_(char16_t*  src, Bounds_Inside * dst) {p157_ar_c_SET(src, 0, strlen16(src), dst);}
Pack * c_TEST_Channel_new_ARRAY_TEST_7_157()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 157));
};
INLINER void p158_ar_u16_SET(uint16_t*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  0, src_max = pos + 2; pos < src_max; pos++, BYTE += 2)
        set_bytes((src[pos]), 2, data,  BYTE);
}
INLINER void p158_v3_SET(uint32_t  src, Pack * dst)//Stub field
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  4);
}
INLINER void p158_ar_d_SET(double*  src, int32_t pos, Pack * dst) //Value array
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  8, src_max = pos + 2; pos < src_max; pos++, BYTE += 8)
        set_bytes(doubleToLongBits(src[pos]), 8, data,  BYTE);
}
Pack * c_TEST_Channel_new_ARRAY_TEST_8_158()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 158));
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED));
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ);
    assert(p0_custom_mode_GET(pack) == (uint32_t)871436993L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)61017);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)34040);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)52855);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)55297);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)25366);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -38);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)53654);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)38721);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)36025);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)6785);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)6611651006765643611L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)492467128L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_y_GET(pack) == (float) -2.67447E38F);
    assert(p3_vx_GET(pack) == (float)1.8723781E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1731220167L);
    assert(p3_afx_GET(pack) == (float) -2.982289E37F);
    assert(p3_afz_GET(pack) == (float) -1.6305881E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)53900);
    assert(p3_afy_GET(pack) == (float) -3.8991173E37F);
    assert(p3_x_GET(pack) == (float)1.5823821E38F);
    assert(p3_yaw_rate_GET(pack) == (float)9.202929E37F);
    assert(p3_vy_GET(pack) == (float) -1.4382453E37F);
    assert(p3_z_GET(pack) == (float) -3.1055298E38F);
    assert(p3_yaw_GET(pack) == (float) -2.8443485E38F);
    assert(p3_vz_GET(pack) == (float) -2.1527105E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p4_seq_GET(pack) == (uint32_t)365994858L);
    assert(p4_time_usec_GET(pack) == (uint64_t)6302494446634245873L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)54);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p5_passkey_LEN(ph) == 21);
    {
        char16_t * exemplary = u"dRrymaoiejzceznzoymts";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)166);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 23);
    {
        char16_t * exemplary = u"teWzaygolyhlhnaqiuepivW";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 46);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2633885312L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -13192);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p20_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"haeLtuysbjushN";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"w";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_value_GET(pack) == (float)3.8603812E37F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)63096);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)17462);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"yikbkmrpn";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p23_param_value_GET(pack) == (float) -1.5280107E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)102);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_h_acc_TRY(ph) == (uint32_t)3636253548L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)13436);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)28283);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1066124706L);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)3321982060L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)304625170L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)12619);
    assert(p24_time_usec_GET(pack) == (uint64_t)5655149024254006198L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p24_alt_GET(pack) == (int32_t) -1654815279);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -749806192);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)16161);
    assert(p24_lon_GET(pack) == (int32_t)670684767);
    assert(p24_lat_GET(pack) == (int32_t) -75896297);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)205);
    {
        uint8_t exemplary[] =  {(uint8_t)116, (uint8_t)166, (uint8_t)95, (uint8_t)228, (uint8_t)31, (uint8_t)66, (uint8_t)154, (uint8_t)235, (uint8_t)63, (uint8_t)13, (uint8_t)121, (uint8_t)11, (uint8_t)103, (uint8_t)5, (uint8_t)59, (uint8_t)238, (uint8_t)122, (uint8_t)0, (uint8_t)199, (uint8_t)202} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)89, (uint8_t)81, (uint8_t)18, (uint8_t)255, (uint8_t)47, (uint8_t)57, (uint8_t)0, (uint8_t)1, (uint8_t)211, (uint8_t)47, (uint8_t)197, (uint8_t)77, (uint8_t)157, (uint8_t)167, (uint8_t)196, (uint8_t)12, (uint8_t)158, (uint8_t)142, (uint8_t)238} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)180, (uint8_t)13, (uint8_t)121, (uint8_t)31, (uint8_t)28, (uint8_t)15, (uint8_t)41, (uint8_t)225, (uint8_t)38, (uint8_t)13, (uint8_t)174, (uint8_t)194, (uint8_t)167, (uint8_t)254, (uint8_t)53, (uint8_t)236, (uint8_t)241, (uint8_t)51, (uint8_t)89, (uint8_t)187} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)232, (uint8_t)132, (uint8_t)46, (uint8_t)30, (uint8_t)102, (uint8_t)186, (uint8_t)190, (uint8_t)198, (uint8_t)121, (uint8_t)234, (uint8_t)38, (uint8_t)178, (uint8_t)99, (uint8_t)47, (uint8_t)32, (uint8_t)224, (uint8_t)127, (uint8_t)0, (uint8_t)247} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)171, (uint8_t)141, (uint8_t)57, (uint8_t)164, (uint8_t)98, (uint8_t)151, (uint8_t)48, (uint8_t)142, (uint8_t)237, (uint8_t)251, (uint8_t)219, (uint8_t)138, (uint8_t)2, (uint8_t)60, (uint8_t)242, (uint8_t)224, (uint8_t)230, (uint8_t)106, (uint8_t)110, (uint8_t)187} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)18976);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)21559);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)1653395348L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -13291);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -11369);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -14856);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -10669);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)3155);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -8576);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)24922);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -18541);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -2595);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -31036);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)27002);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)25097);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)29694);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -3647);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -13496);
    assert(p27_time_usec_GET(pack) == (uint64_t)9218885039736564971L);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -29934);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)6968413614037378389L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -31157);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -24136);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -15840);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -1939);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -32235);
    assert(p29_press_diff_GET(pack) == (float)1.4225954E38F);
    assert(p29_press_abs_GET(pack) == (float)6.5931547E37F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1398711674L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float)3.1727424E38F);
    assert(p30_yawspeed_GET(pack) == (float) -3.5494932E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3629138493L);
    assert(p30_pitchspeed_GET(pack) == (float) -2.6078903E38F);
    assert(p30_roll_GET(pack) == (float) -1.0697225E38F);
    assert(p30_yaw_GET(pack) == (float)1.7154736E38F);
    assert(p30_pitch_GET(pack) == (float) -2.528399E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float)6.212208E37F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3664188803L);
    assert(p31_yawspeed_GET(pack) == (float)9.612121E37F);
    assert(p31_q2_GET(pack) == (float) -1.0652034E38F);
    assert(p31_q4_GET(pack) == (float)3.079741E37F);
    assert(p31_rollspeed_GET(pack) == (float)1.9504926E38F);
    assert(p31_pitchspeed_GET(pack) == (float)6.2292046E37F);
    assert(p31_q1_GET(pack) == (float)8.880396E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vy_GET(pack) == (float) -2.4418857E38F);
    assert(p32_y_GET(pack) == (float)1.7047377E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)3328979060L);
    assert(p32_vz_GET(pack) == (float)1.9234693E38F);
    assert(p32_x_GET(pack) == (float) -3.0304686E38F);
    assert(p32_vx_GET(pack) == (float) -2.3212966E38F);
    assert(p32_z_GET(pack) == (float)2.7826148E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -19444);
    assert(p33_relative_alt_GET(pack) == (int32_t) -1122643787);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)17566);
    assert(p33_alt_GET(pack) == (int32_t)496298056);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)4083379058L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)38056);
    assert(p33_lon_GET(pack) == (int32_t) -2005968202);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)31865);
    assert(p33_lat_GET(pack) == (int32_t)1296900327);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)18886);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)14630);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -32339);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)13333);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3095055223L);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -26966);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -30619);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)3463);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -24190);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)1300);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3736555819L);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)54270);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)23514);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)65405);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)39625);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)28900);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)31114);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)3204);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)1685);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)3061);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)49082);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)17365);
    assert(p36_time_usec_GET(pack) == (uint32_t)2927463588L);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)17954);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)15080);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)47199);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)27165);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)34576);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)63174);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)56765);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)58984);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)51969);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)64980);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)64890);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)56609);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)28773);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -17098);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -27014);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)5375);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p39_param1_GET(pack) == (float)1.9016E38F);
    assert(p39_y_GET(pack) == (float) -2.8709791E38F);
    assert(p39_param4_GET(pack) == (float)2.731169E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p39_param3_GET(pack) == (float)3.304267E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p39_param2_GET(pack) == (float) -2.6794315E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION);
    assert(p39_x_GET(pack) == (float)3.1507624E38F);
    assert(p39_z_GET(pack) == (float)1.5647951E38F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)37854);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)42534);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)17453);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)87);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)26825);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)245);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)49284);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)20);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)20197);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t) -368294850);
    assert(p48_longitude_GET(pack) == (int32_t) -868355018);
    assert(p48_time_usec_TRY(ph) == (uint64_t)3346985133315847267L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p48_latitude_GET(pack) == (int32_t) -1471809522);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_latitude_GET(pack) == (int32_t) -656647454);
    assert(p49_longitude_GET(pack) == (int32_t)276822135);
    assert(p49_time_usec_TRY(ph) == (uint64_t)8612738780002949568L);
    assert(p49_altitude_GET(pack) == (int32_t) -83230994);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p50_scale_GET(pack) == (float) -1.7163168E38F);
    assert(p50_param_value_min_GET(pack) == (float)1.8041413E37F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -30789);
    assert(p50_param_value_max_GET(pack) == (float)2.4438411E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p50_param_value0_GET(pack) == (float) -7.857354E37F);
    assert(p50_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"ngbxmotx";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)97);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)52931);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p54_p2y_GET(pack) == (float) -2.544604E37F);
    assert(p54_p1x_GET(pack) == (float) -2.0874972E38F);
    assert(p54_p2x_GET(pack) == (float)2.7398759E38F);
    assert(p54_p1y_GET(pack) == (float)1.116793E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p54_p2z_GET(pack) == (float) -6.201303E37F);
    assert(p54_p1z_GET(pack) == (float)9.660682E37F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p55_p2x_GET(pack) == (float)7.769319E37F);
    assert(p55_p1x_GET(pack) == (float) -3.3822492E38F);
    assert(p55_p2y_GET(pack) == (float)1.3145149E38F);
    assert(p55_p2z_GET(pack) == (float) -1.6298922E38F);
    assert(p55_p1y_GET(pack) == (float) -1.3009032E37F);
    assert(p55_p1z_GET(pack) == (float)2.9484949E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {6.9841164E37F, 2.4518529E38F, -2.467665E38F, -3.2632052E38F, -4.292368E37F, 1.9098515E38F, 6.706663E36F, -9.980864E37F, -1.2132169E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {6.3786997E37F, -2.1535585E38F, 6.6326553E36F, -2.99704E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)2.9085404E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)4854365606939103500L);
    assert(p61_yawspeed_GET(pack) == (float) -1.776749E38F);
    assert(p61_pitchspeed_GET(pack) == (float)1.2130578E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_alt_error_GET(pack) == (float) -1.9297778E38F);
    assert(p62_nav_pitch_GET(pack) == (float) -2.7463E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)13040);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)11579);
    assert(p62_aspd_error_GET(pack) == (float) -1.1218654E38F);
    assert(p62_nav_roll_GET(pack) == (float)3.327999E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)31592);
    assert(p62_xtrack_error_GET(pack) == (float) -2.5338762E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_relative_alt_GET(pack) == (int32_t) -1164613176);
    assert(p63_vy_GET(pack) == (float) -1.0984095E38F);
    assert(p63_vz_GET(pack) == (float) -2.7836451E38F);
    assert(p63_alt_GET(pack) == (int32_t)422168745);
    {
        float exemplary[] =  {-1.3155995E38F, 1.3902299E38F, 3.2324167E38F, -2.7046384E36F, 2.297097E38F, 1.9694636E38F, 5.9869353E37F, 1.4521406E38F, 1.0127852E38F, -1.6785161E38F, 1.965986E38F, -8.952148E37F, -2.4322536E38F, -1.0640851E38F, 2.7698776E38F, 8.092147E36F, 1.7805613E38F, -2.9465625E38F, -2.3093485E38F, -3.288549E38F, 1.7172546E38F, -3.2910262E38F, -9.418213E37F, 3.2116666E38F, 3.04114E38F, 2.0476985E38F, 2.6738522E38F, 6.0526483E37F, 3.1584738E38F, -2.36446E38F, 2.5528704E38F, -2.598191E38F, 2.6559343E38F, -7.186295E37F, -1.8627738E38F, -2.2727576E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p63_lon_GET(pack) == (int32_t)1228408403);
    assert(p63_lat_GET(pack) == (int32_t)1467097246);
    assert(p63_time_usec_GET(pack) == (uint64_t)1411558573325857449L);
    assert(p63_vx_GET(pack) == (float) -2.0551364E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_x_GET(pack) == (float)9.45743E36F);
    assert(p64_ay_GET(pack) == (float) -3.3160807E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_y_GET(pack) == (float) -1.1145502E37F);
    assert(p64_ax_GET(pack) == (float) -2.4883322E38F);
    assert(p64_vx_GET(pack) == (float)1.9541595E38F);
    {
        float exemplary[] =  {-1.92428E38F, -1.148704E38F, -8.982149E37F, 4.893119E37F, 8.1797487E37F, -1.1895023E38F, 2.3970843E38F, -1.989887E38F, -2.9796423E38F, -1.8063772E38F, -5.5955497E37F, 3.681566E37F, -1.7847239E38F, -2.8715503E38F, 1.0663437E38F, -2.7141809E38F, 3.044285E38F, 7.365548E37F, 1.3451297E38F, -3.113809E38F, 1.77355E38F, -5.077064E37F, 3.2221147E38F, 6.326462E37F, 1.007861E38F, 1.1946256E38F, 2.9365628E37F, -1.971252E38F, -1.0230155E38F, 5.1961373E37F, -2.6450353E38F, -2.912981E38F, -6.6560566E37F, -1.0897444E38F, 2.8125843E38F, -2.0682405E38F, 6.0947353E37F, -6.8285224E37F, 2.853398E38F, -5.0792387E37F, 3.398376E38F, 1.3506833E38F, 1.0354983E38F, 2.2036954E38F, 3.2391075E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_z_GET(pack) == (float)2.2283026E38F);
    assert(p64_az_GET(pack) == (float) -4.6700253E37F);
    assert(p64_vz_GET(pack) == (float)1.2864134E38F);
    assert(p64_vy_GET(pack) == (float)5.568113E37F);
    assert(p64_time_usec_GET(pack) == (uint64_t)3155314427644830505L);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)605879625L);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)7660);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)4712);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)21111);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)15277);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)52940);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)57965);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)36018);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)4164);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)12354);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)34636);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)38628);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)51564);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)18221);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)7888);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)62638);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)17775);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)6231);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)41980);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)39348);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)18039);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)30989);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)14366);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -9659);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -6366);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)2000);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)58426);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)14454);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)39400);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)37525);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)499);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)17564);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)3203);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)41442);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_z_GET(pack) == (float) -5.9672243E37F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)49521);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM);
    assert(p73_param3_GET(pack) == (float)9.852989E37F);
    assert(p73_param2_GET(pack) == (float) -2.044242E38F);
    assert(p73_param4_GET(pack) == (float)1.5125737E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p73_param1_GET(pack) == (float) -2.0721443E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p73_x_GET(pack) == (int32_t)1526948572);
    assert(p73_y_GET(pack) == (int32_t) -1572376029);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_climb_GET(pack) == (float) -2.4165984E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)3131);
    assert(p74_airspeed_GET(pack) == (float)1.0485253E38F);
    assert(p74_alt_GET(pack) == (float)2.1479698E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -11611);
    assert(p74_groundspeed_GET(pack) == (float) -3.402787E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p75_param3_GET(pack) == (float)2.9233246E38F);
    assert(p75_param2_GET(pack) == (float) -2.123491E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p75_x_GET(pack) == (int32_t) -390325247);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p75_z_GET(pack) == (float)2.391306E38F);
    assert(p75_param1_GET(pack) == (float) -5.26136E37F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE);
    assert(p75_param4_GET(pack) == (float)2.6141884E38F);
    assert(p75_y_GET(pack) == (int32_t)547336148);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param4_GET(pack) == (float)4.921867E37F);
    assert(p76_param7_GET(pack) == (float)1.5851074E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH);
    assert(p76_param3_GET(pack) == (float)1.7587494E38F);
    assert(p76_param2_GET(pack) == (float) -6.059154E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p76_param5_GET(pack) == (float) -3.6362E36F);
    assert(p76_param6_GET(pack) == (float)1.0144138E38F);
    assert(p76_param1_GET(pack) == (float) -3.2894736E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t)892402047);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)71);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)14);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)170);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_pitch_GET(pack) == (float)1.5901317E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p81_thrust_GET(pack) == (float) -2.4916887E38F);
    assert(p81_roll_GET(pack) == (float) -1.4661469E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p81_yaw_GET(pack) == (float)5.2026896E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1036933286L);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_yaw_rate_GET(pack) == (float)1.5548614E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2535104121L);
    {
        float exemplary[] =  {-1.0224555E38F, -2.8821393E38F, -1.4277284E38F, 1.848114E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_roll_rate_GET(pack) == (float)1.6246406E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.7492426E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p82_thrust_GET(pack) == (float)2.2869176E36F);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p83_thrust_GET(pack) == (float)8.87309E37F);
    {
        float exemplary[] =  {2.0106519E38F, 1.8717451E38F, -3.3856696E38F, -2.8201395E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_yaw_rate_GET(pack) == (float)2.4577964E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3054172248L);
    assert(p83_body_pitch_rate_GET(pack) == (float) -2.011818E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)9.000205E37F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_z_GET(pack) == (float) -3.1875865E38F);
    assert(p84_y_GET(pack) == (float)2.0732533E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)46002);
    assert(p84_yaw_GET(pack) == (float) -7.5928265E37F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p84_afx_GET(pack) == (float) -5.080117E37F);
    assert(p84_vz_GET(pack) == (float)5.0224327E37F);
    assert(p84_afz_GET(pack) == (float)4.5614607E36F);
    assert(p84_x_GET(pack) == (float)2.6381344E38F);
    assert(p84_vx_GET(pack) == (float)1.4782384E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p84_yaw_rate_GET(pack) == (float)1.9750814E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2386034450L);
    assert(p84_vy_GET(pack) == (float) -1.18521E38F);
    assert(p84_afy_GET(pack) == (float) -2.3990616E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vz_GET(pack) == (float)1.6301812E38F);
    assert(p86_afx_GET(pack) == (float)2.8976876E38F);
    assert(p86_vy_GET(pack) == (float) -1.827532E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p86_afz_GET(pack) == (float)2.5419473E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)2087876949);
    assert(p86_yaw_rate_GET(pack) == (float) -6.435129E37F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p86_yaw_GET(pack) == (float) -7.210482E37F);
    assert(p86_lat_int_GET(pack) == (int32_t) -72030663);
    assert(p86_alt_GET(pack) == (float)1.5212599E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)46439);
    assert(p86_afy_GET(pack) == (float) -8.2673373E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3624882837L);
    assert(p86_vx_GET(pack) == (float)2.6038474E38F);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afx_GET(pack) == (float) -3.243549E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)9982);
    assert(p87_lat_int_GET(pack) == (int32_t) -194065072);
    assert(p87_afy_GET(pack) == (float)2.3306525E38F);
    assert(p87_yaw_GET(pack) == (float) -8.506911E37F);
    assert(p87_alt_GET(pack) == (float) -1.516016E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)758246354);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p87_vy_GET(pack) == (float)2.6227458E38F);
    assert(p87_vx_GET(pack) == (float)6.1966595E37F);
    assert(p87_afz_GET(pack) == (float)2.3029388E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.1030589E38F);
    assert(p87_vz_GET(pack) == (float)2.3839946E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)348238432L);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_roll_GET(pack) == (float) -2.1337517E38F);
    assert(p89_pitch_GET(pack) == (float) -2.4047975E38F);
    assert(p89_x_GET(pack) == (float)2.3351085E38F);
    assert(p89_y_GET(pack) == (float) -2.3733013E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)1711734387L);
    assert(p89_z_GET(pack) == (float)1.6302703E37F);
    assert(p89_yaw_GET(pack) == (float)2.7718466E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -29731);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)12299);
    assert(p90_rollspeed_GET(pack) == (float) -2.4519652E38F);
    assert(p90_lon_GET(pack) == (int32_t)890075484);
    assert(p90_lat_GET(pack) == (int32_t)1632889668);
    assert(p90_roll_GET(pack) == (float) -1.0227802E38F);
    assert(p90_time_usec_GET(pack) == (uint64_t)2875566780613275902L);
    assert(p90_pitchspeed_GET(pack) == (float) -3.0455519E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)32);
    assert(p90_pitch_GET(pack) == (float)3.3332276E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -3725);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)4936);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -16395);
    assert(p90_yawspeed_GET(pack) == (float) -7.1002145E37F);
    assert(p90_yaw_GET(pack) == (float)3.1437534E38F);
    assert(p90_alt_GET(pack) == (int32_t)473957599);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_time_usec_GET(pack) == (uint64_t)3574366073108082075L);
    assert(p91_throttle_GET(pack) == (float) -1.4897131E38F);
    assert(p91_aux3_GET(pack) == (float) -2.05094E38F);
    assert(p91_aux1_GET(pack) == (float) -2.3923473E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)9.58729E37F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
    assert(p91_roll_ailerons_GET(pack) == (float)2.904319E38F);
    assert(p91_aux2_GET(pack) == (float)2.0971336E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p91_aux4_GET(pack) == (float) -1.3287774E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -2.6737272E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)16495);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)41364);
    assert(p92_time_usec_GET(pack) == (uint64_t)6128562353897259889L);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)35812);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)62034);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)42865);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)43035);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)35594);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)44064);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)65185);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)25501);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)55284);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)12442);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)246);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.659579E38F, 1.440456E37F, -1.8178613E38F, -2.8117932E38F, -1.4668692E38F, -2.5041373E38F, 2.4063972E38F, -9.850118E37F, -1.3453557E38F, 1.5900587E38F, -3.1374464E38F, -2.7913494E38F, -1.2085362E38F, 2.6269686E38F, -2.8572877E38F, -2.951357E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_ARMED);
    assert(p93_flags_GET(pack) == (uint64_t)1906839943334649562L);
    assert(p93_time_usec_GET(pack) == (uint64_t)1074516196518442904L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -27528);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.2041745E38F);
    assert(p100_ground_distance_GET(pack) == (float) -2.2954704E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p100_flow_rate_y_TRY(ph) == (float) -2.7811078E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)1340);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -9.171393E36F);
    assert(p100_time_usec_GET(pack) == (uint64_t)24015338748740162L);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -3.2457753E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_yaw_GET(pack) == (float) -1.648196E38F);
    assert(p101_y_GET(pack) == (float) -7.8664017E37F);
    assert(p101_x_GET(pack) == (float) -1.0470714E38F);
    assert(p101_usec_GET(pack) == (uint64_t)3063363755707340993L);
    assert(p101_pitch_GET(pack) == (float) -2.0738466E38F);
    assert(p101_z_GET(pack) == (float) -6.5377655E37F);
    assert(p101_roll_GET(pack) == (float) -2.8482561E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float) -4.3327775E37F);
    assert(p102_usec_GET(pack) == (uint64_t)466094875094332831L);
    assert(p102_yaw_GET(pack) == (float)9.230135E37F);
    assert(p102_pitch_GET(pack) == (float) -6.8136E37F);
    assert(p102_x_GET(pack) == (float)3.1441481E38F);
    assert(p102_y_GET(pack) == (float) -5.6172397E37F);
    assert(p102_roll_GET(pack) == (float) -2.7900062E37F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)8569109981523207379L);
    assert(p103_y_GET(pack) == (float)3.4671152E37F);
    assert(p103_x_GET(pack) == (float) -3.2380015E38F);
    assert(p103_z_GET(pack) == (float) -1.0279806E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_pitch_GET(pack) == (float) -2.3334705E38F);
    assert(p104_z_GET(pack) == (float)5.8559054E37F);
    assert(p104_usec_GET(pack) == (uint64_t)3874027962599398964L);
    assert(p104_yaw_GET(pack) == (float)2.4368648E38F);
    assert(p104_x_GET(pack) == (float)2.9980823E38F);
    assert(p104_y_GET(pack) == (float)1.4289651E37F);
    assert(p104_roll_GET(pack) == (float)2.6494526E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_pressure_alt_GET(pack) == (float)1.991228E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.8094936E38F);
    assert(p105_ygyro_GET(pack) == (float) -1.8804262E38F);
    assert(p105_xmag_GET(pack) == (float) -7.833256E37F);
    assert(p105_abs_pressure_GET(pack) == (float) -3.3345486E38F);
    assert(p105_ymag_GET(pack) == (float) -2.602572E38F);
    assert(p105_zacc_GET(pack) == (float) -2.3313167E38F);
    assert(p105_zgyro_GET(pack) == (float) -2.054596E38F);
    assert(p105_xgyro_GET(pack) == (float)1.3540088E38F);
    assert(p105_zmag_GET(pack) == (float)1.5282807E38F);
    assert(p105_temperature_GET(pack) == (float) -3.3391022E38F);
    assert(p105_yacc_GET(pack) == (float)2.850916E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4314834508368526773L);
    assert(p105_xacc_GET(pack) == (float)9.185729E36F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)37605);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_distance_GET(pack) == (float) -2.8103666E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)232416971L);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p106_integrated_zgyro_GET(pack) == (float)3.2702584E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -1.4794364E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)7511081L);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p106_integrated_y_GET(pack) == (float)1.7358226E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)7462121062121325875L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -2187);
    assert(p106_integrated_x_GET(pack) == (float) -2.822283E37F);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.244041E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_time_usec_GET(pack) == (uint64_t)3560774013116263292L);
    assert(p107_pressure_alt_GET(pack) == (float) -2.0690127E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2237157470L);
    assert(p107_ygyro_GET(pack) == (float) -9.373474E37F);
    assert(p107_ymag_GET(pack) == (float) -1.5712412E38F);
    assert(p107_zgyro_GET(pack) == (float)9.97767E37F);
    assert(p107_xmag_GET(pack) == (float)5.0210845E36F);
    assert(p107_temperature_GET(pack) == (float)3.279033E38F);
    assert(p107_xacc_GET(pack) == (float)6.516222E37F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.5303187E38F);
    assert(p107_zmag_GET(pack) == (float)1.7267922E38F);
    assert(p107_yacc_GET(pack) == (float) -2.0426405E38F);
    assert(p107_abs_pressure_GET(pack) == (float)2.630373E38F);
    assert(p107_zacc_GET(pack) == (float) -5.5915784E37F);
    assert(p107_xgyro_GET(pack) == (float) -2.7260814E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_q1_GET(pack) == (float) -2.385164E38F);
    assert(p108_lon_GET(pack) == (float)2.5278707E38F);
    assert(p108_xgyro_GET(pack) == (float) -1.9789385E38F);
    assert(p108_lat_GET(pack) == (float) -1.7222512E38F);
    assert(p108_vn_GET(pack) == (float)2.5344351E38F);
    assert(p108_zacc_GET(pack) == (float) -2.576013E37F);
    assert(p108_std_dev_vert_GET(pack) == (float) -2.9556318E38F);
    assert(p108_alt_GET(pack) == (float) -2.8389366E38F);
    assert(p108_yaw_GET(pack) == (float)2.7416155E38F);
    assert(p108_roll_GET(pack) == (float)1.9140766E38F);
    assert(p108_ve_GET(pack) == (float) -1.823243E38F);
    assert(p108_xacc_GET(pack) == (float)2.4222028E38F);
    assert(p108_q3_GET(pack) == (float)5.632754E36F);
    assert(p108_q4_GET(pack) == (float) -3.1924772E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.9718432E38F);
    assert(p108_pitch_GET(pack) == (float)2.1629762E38F);
    assert(p108_zgyro_GET(pack) == (float) -7.106244E37F);
    assert(p108_vd_GET(pack) == (float) -1.756188E38F);
    assert(p108_ygyro_GET(pack) == (float) -4.1979018E37F);
    assert(p108_yacc_GET(pack) == (float) -3.1776936E38F);
    assert(p108_q2_GET(pack) == (float) -2.6264165E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)13648);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)5810);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)72);
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)108, (uint8_t)199, (uint8_t)236, (uint8_t)191, (uint8_t)224, (uint8_t)114, (uint8_t)128, (uint8_t)216, (uint8_t)240, (uint8_t)210, (uint8_t)1, (uint8_t)171, (uint8_t)215, (uint8_t)46, (uint8_t)65, (uint8_t)132, (uint8_t)203, (uint8_t)95, (uint8_t)234, (uint8_t)94, (uint8_t)112, (uint8_t)176, (uint8_t)197, (uint8_t)237, (uint8_t)135, (uint8_t)180, (uint8_t)208, (uint8_t)108, (uint8_t)246, (uint8_t)69, (uint8_t)53, (uint8_t)67, (uint8_t)239, (uint8_t)157, (uint8_t)234, (uint8_t)42, (uint8_t)136, (uint8_t)247, (uint8_t)128, (uint8_t)42, (uint8_t)31, (uint8_t)49, (uint8_t)31, (uint8_t)58, (uint8_t)214, (uint8_t)169, (uint8_t)122, (uint8_t)3, (uint8_t)226, (uint8_t)215, (uint8_t)129, (uint8_t)33, (uint8_t)210, (uint8_t)229, (uint8_t)208, (uint8_t)174, (uint8_t)111, (uint8_t)10, (uint8_t)110, (uint8_t)84, (uint8_t)60, (uint8_t)48, (uint8_t)237, (uint8_t)99, (uint8_t)244, (uint8_t)155, (uint8_t)176, (uint8_t)238, (uint8_t)73, (uint8_t)186, (uint8_t)2, (uint8_t)143, (uint8_t)113, (uint8_t)133, (uint8_t)68, (uint8_t)245, (uint8_t)35, (uint8_t)249, (uint8_t)223, (uint8_t)43, (uint8_t)113, (uint8_t)129, (uint8_t)44, (uint8_t)80, (uint8_t)138, (uint8_t)191, (uint8_t)253, (uint8_t)193, (uint8_t)134, (uint8_t)92, (uint8_t)84, (uint8_t)146, (uint8_t)142, (uint8_t)219, (uint8_t)151, (uint8_t)158, (uint8_t)14, (uint8_t)209, (uint8_t)128, (uint8_t)121, (uint8_t)133, (uint8_t)230, (uint8_t)86, (uint8_t)48, (uint8_t)107, (uint8_t)114, (uint8_t)200, (uint8_t)187, (uint8_t)211, (uint8_t)45, (uint8_t)70, (uint8_t)210, (uint8_t)129, (uint8_t)171, (uint8_t)152, (uint8_t)30, (uint8_t)122, (uint8_t)30, (uint8_t)10, (uint8_t)60, (uint8_t)196, (uint8_t)222, (uint8_t)132, (uint8_t)90, (uint8_t)159, (uint8_t)156, (uint8_t)229, (uint8_t)161, (uint8_t)5, (uint8_t)27, (uint8_t)66, (uint8_t)232, (uint8_t)34, (uint8_t)163, (uint8_t)101, (uint8_t)186, (uint8_t)15, (uint8_t)126, (uint8_t)140, (uint8_t)128, (uint8_t)121, (uint8_t)79, (uint8_t)116, (uint8_t)198, (uint8_t)104, (uint8_t)243, (uint8_t)135, (uint8_t)60, (uint8_t)176, (uint8_t)150, (uint8_t)193, (uint8_t)106, (uint8_t)153, (uint8_t)38, (uint8_t)116, (uint8_t)90, (uint8_t)229, (uint8_t)101, (uint8_t)156, (uint8_t)8, (uint8_t)208, (uint8_t)109, (uint8_t)60, (uint8_t)171, (uint8_t)17, (uint8_t)3, (uint8_t)198, (uint8_t)109, (uint8_t)133, (uint8_t)19, (uint8_t)145, (uint8_t)173, (uint8_t)206, (uint8_t)147, (uint8_t)159, (uint8_t)55, (uint8_t)188, (uint8_t)114, (uint8_t)212, (uint8_t)24, (uint8_t)141, (uint8_t)5, (uint8_t)225, (uint8_t)81, (uint8_t)74, (uint8_t)171, (uint8_t)142, (uint8_t)9, (uint8_t)42, (uint8_t)121, (uint8_t)53, (uint8_t)37, (uint8_t)251, (uint8_t)34, (uint8_t)57, (uint8_t)57, (uint8_t)182, (uint8_t)25, (uint8_t)235, (uint8_t)44, (uint8_t)236, (uint8_t)248, (uint8_t)231, (uint8_t)150, (uint8_t)41, (uint8_t)146, (uint8_t)112, (uint8_t)138, (uint8_t)85, (uint8_t)120, (uint8_t)49, (uint8_t)12, (uint8_t)254, (uint8_t)226, (uint8_t)124, (uint8_t)64, (uint8_t)154, (uint8_t)168, (uint8_t)11, (uint8_t)235, (uint8_t)158, (uint8_t)158, (uint8_t)215, (uint8_t)221, (uint8_t)126, (uint8_t)117, (uint8_t)81, (uint8_t)146, (uint8_t)43, (uint8_t)49, (uint8_t)110, (uint8_t)129, (uint8_t)7, (uint8_t)219, (uint8_t)107, (uint8_t)108, (uint8_t)19, (uint8_t)81, (uint8_t)165, (uint8_t)136, (uint8_t)222, (uint8_t)135, (uint8_t)72, (uint8_t)116, (uint8_t)141, (uint8_t)199, (uint8_t)163, (uint8_t)248, (uint8_t)77, (uint8_t)155} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -4808969450363479193L);
    assert(p111_ts1_GET(pack) == (int64_t)4466127170473427585L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)107188697263573144L);
    assert(p112_seq_GET(pack) == (uint32_t)239404337L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)3275);
    assert(p113_lon_GET(pack) == (int32_t)61169770);
    assert(p113_lat_GET(pack) == (int32_t) -1427239501);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)62585);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)4072);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)7660);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)53968);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)59641);
    assert(p113_alt_GET(pack) == (int32_t)736202634);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)40287);
    assert(p113_time_usec_GET(pack) == (uint64_t)1997056478941307628L);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_ygyro_GET(pack) == (float) -2.511642E38F);
    assert(p114_integrated_y_GET(pack) == (float) -2.3821094E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.8138535E38F);
    assert(p114_integrated_x_GET(pack) == (float) -2.1045647E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -19864);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2226133378L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p114_distance_GET(pack) == (float)1.0830284E36F);
    assert(p114_time_usec_GET(pack) == (uint64_t)711551704056499869L);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.3535418E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)2226145207L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)32737);
    assert(p115_alt_GET(pack) == (int32_t) -346919627);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -19463);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -25290);
    assert(p115_lon_GET(pack) == (int32_t) -195769534);
    assert(p115_time_usec_GET(pack) == (uint64_t)6924289734490413147L);
    assert(p115_lat_GET(pack) == (int32_t) -1745386364);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -2725);
    {
        float exemplary[] =  {-1.0687694E38F, 2.4313932E38F, -2.1720388E38F, 9.35792E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -3138);
    assert(p115_pitchspeed_GET(pack) == (float) -3.1841493E38F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)13988);
    assert(p115_yawspeed_GET(pack) == (float)3.0019175E38F);
    assert(p115_rollspeed_GET(pack) == (float)1.5603484E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)18461);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)17332);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3094726655L);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)15387);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)8999);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)17237);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -5003);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -17727);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)10117);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)15666);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)16812);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)31183);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)29082);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)26519);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)54954);
    assert(p118_time_utc_GET(pack) == (uint32_t)1959894889L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)47522);
    assert(p118_size_GET(pack) == (uint32_t)561117032L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)7313);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)36629);
    assert(p119_count_GET(pack) == (uint32_t)4031110537L);
    assert(p119_ofs_GET(pack) == (uint32_t)4100230672L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)252);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)3531183921L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)62476);
    {
        uint8_t exemplary[] =  {(uint8_t)148, (uint8_t)225, (uint8_t)58, (uint8_t)158, (uint8_t)11, (uint8_t)6, (uint8_t)140, (uint8_t)58, (uint8_t)33, (uint8_t)17, (uint8_t)19, (uint8_t)201, (uint8_t)44, (uint8_t)10, (uint8_t)167, (uint8_t)78, (uint8_t)145, (uint8_t)187, (uint8_t)219, (uint8_t)155, (uint8_t)165, (uint8_t)23, (uint8_t)31, (uint8_t)43, (uint8_t)101, (uint8_t)32, (uint8_t)235, (uint8_t)34, (uint8_t)194, (uint8_t)78, (uint8_t)52, (uint8_t)37, (uint8_t)185, (uint8_t)215, (uint8_t)237, (uint8_t)136, (uint8_t)232, (uint8_t)148, (uint8_t)127, (uint8_t)77, (uint8_t)172, (uint8_t)28, (uint8_t)40, (uint8_t)87, (uint8_t)212, (uint8_t)43, (uint8_t)10, (uint8_t)110, (uint8_t)74, (uint8_t)10, (uint8_t)253, (uint8_t)21, (uint8_t)51, (uint8_t)86, (uint8_t)120, (uint8_t)94, (uint8_t)37, (uint8_t)45, (uint8_t)120, (uint8_t)212, (uint8_t)188, (uint8_t)194, (uint8_t)95, (uint8_t)42, (uint8_t)128, (uint8_t)181, (uint8_t)12, (uint8_t)135, (uint8_t)136, (uint8_t)164, (uint8_t)178, (uint8_t)144, (uint8_t)245, (uint8_t)220, (uint8_t)94, (uint8_t)97, (uint8_t)210, (uint8_t)63, (uint8_t)14, (uint8_t)54, (uint8_t)9, (uint8_t)78, (uint8_t)205, (uint8_t)37, (uint8_t)141, (uint8_t)74, (uint8_t)159, (uint8_t)5, (uint8_t)163, (uint8_t)154} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)98);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)151, (uint8_t)156, (uint8_t)180, (uint8_t)208, (uint8_t)175, (uint8_t)135, (uint8_t)91, (uint8_t)40, (uint8_t)106, (uint8_t)115, (uint8_t)122, (uint8_t)253, (uint8_t)37, (uint8_t)218, (uint8_t)46, (uint8_t)219, (uint8_t)182, (uint8_t)32, (uint8_t)147, (uint8_t)231, (uint8_t)195, (uint8_t)251, (uint8_t)219, (uint8_t)61, (uint8_t)209, (uint8_t)66, (uint8_t)254, (uint8_t)221, (uint8_t)107, (uint8_t)219, (uint8_t)71, (uint8_t)95, (uint8_t)130, (uint8_t)49, (uint8_t)45, (uint8_t)83, (uint8_t)231, (uint8_t)142, (uint8_t)19, (uint8_t)247, (uint8_t)181, (uint8_t)204, (uint8_t)6, (uint8_t)18, (uint8_t)94, (uint8_t)84, (uint8_t)133, (uint8_t)111, (uint8_t)140, (uint8_t)33, (uint8_t)213, (uint8_t)78, (uint8_t)255, (uint8_t)87, (uint8_t)122, (uint8_t)240, (uint8_t)85, (uint8_t)241, (uint8_t)155, (uint8_t)8, (uint8_t)32, (uint8_t)199, (uint8_t)41, (uint8_t)160, (uint8_t)204, (uint8_t)34, (uint8_t)250, (uint8_t)136, (uint8_t)81, (uint8_t)47, (uint8_t)3, (uint8_t)39, (uint8_t)8, (uint8_t)50, (uint8_t)36, (uint8_t)155, (uint8_t)97, (uint8_t)148, (uint8_t)8, (uint8_t)183, (uint8_t)59, (uint8_t)145, (uint8_t)44, (uint8_t)207, (uint8_t)151, (uint8_t)101, (uint8_t)248, (uint8_t)36, (uint8_t)27, (uint8_t)233, (uint8_t)240, (uint8_t)114, (uint8_t)208, (uint8_t)177, (uint8_t)110, (uint8_t)226, (uint8_t)19, (uint8_t)10, (uint8_t)43, (uint8_t)217, (uint8_t)80, (uint8_t)176, (uint8_t)231, (uint8_t)187, (uint8_t)79, (uint8_t)85, (uint8_t)52, (uint8_t)230, (uint8_t)245, (uint8_t)171} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)28);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)28024);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)16510);
    assert(p124_lat_GET(pack) == (int32_t)1555729842);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)37470);
    assert(p124_dgps_age_GET(pack) == (uint32_t)404762167L);
    assert(p124_lon_GET(pack) == (int32_t)1965869035);
    assert(p124_time_usec_GET(pack) == (uint64_t)6686221928487639060L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)63181);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p124_alt_GET(pack) == (int32_t) -2086863732);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)33757);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)59073);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE));
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)1566);
    {
        uint8_t exemplary[] =  {(uint8_t)119, (uint8_t)43, (uint8_t)125, (uint8_t)89, (uint8_t)76, (uint8_t)58, (uint8_t)39, (uint8_t)45, (uint8_t)92, (uint8_t)75, (uint8_t)176, (uint8_t)103, (uint8_t)117, (uint8_t)65, (uint8_t)164, (uint8_t)99, (uint8_t)158, (uint8_t)144, (uint8_t)247, (uint8_t)170, (uint8_t)88, (uint8_t)187, (uint8_t)16, (uint8_t)98, (uint8_t)17, (uint8_t)229, (uint8_t)251, (uint8_t)225, (uint8_t)236, (uint8_t)10, (uint8_t)69, (uint8_t)21, (uint8_t)252, (uint8_t)133, (uint8_t)123, (uint8_t)229, (uint8_t)64, (uint8_t)116, (uint8_t)35, (uint8_t)88, (uint8_t)183, (uint8_t)198, (uint8_t)80, (uint8_t)21, (uint8_t)57, (uint8_t)17, (uint8_t)214, (uint8_t)164, (uint8_t)41, (uint8_t)229, (uint8_t)93, (uint8_t)40, (uint8_t)204, (uint8_t)154, (uint8_t)78, (uint8_t)197, (uint8_t)186, (uint8_t)116, (uint8_t)234, (uint8_t)174, (uint8_t)150, (uint8_t)37, (uint8_t)125, (uint8_t)110, (uint8_t)142, (uint8_t)36, (uint8_t)18, (uint8_t)226, (uint8_t)134, (uint8_t)86} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_baudrate_GET(pack) == (uint32_t)2525311889L);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1958421470);
    assert(p127_tow_GET(pack) == (uint32_t)3792058414L);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p127_accuracy_GET(pack) == (uint32_t)2593122813L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1795630538);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)2080525735);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)3381908578L);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)843386258);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)8767);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p128_accuracy_GET(pack) == (uint32_t)4112938188L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)2214651760L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)9742);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1450758137);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -2114908491);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1771901990);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)934091774);
    assert(p128_tow_GET(pack) == (uint32_t)923922609L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)3749147331L);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -29696);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)7680);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -4391);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)13691);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)3980);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)29129);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -30049);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -16267);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)17878);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)8672);
    assert(p130_size_GET(pack) == (uint32_t)36721486L);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)30630);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)29929);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)43);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)250, (uint8_t)0, (uint8_t)91, (uint8_t)20, (uint8_t)63, (uint8_t)53, (uint8_t)82, (uint8_t)17, (uint8_t)152, (uint8_t)50, (uint8_t)212, (uint8_t)185, (uint8_t)115, (uint8_t)75, (uint8_t)2, (uint8_t)192, (uint8_t)237, (uint8_t)250, (uint8_t)205, (uint8_t)30, (uint8_t)35, (uint8_t)126, (uint8_t)128, (uint8_t)226, (uint8_t)3, (uint8_t)220, (uint8_t)162, (uint8_t)221, (uint8_t)228, (uint8_t)55, (uint8_t)252, (uint8_t)225, (uint8_t)181, (uint8_t)133, (uint8_t)230, (uint8_t)114, (uint8_t)54, (uint8_t)209, (uint8_t)30, (uint8_t)67, (uint8_t)90, (uint8_t)79, (uint8_t)205, (uint8_t)32, (uint8_t)39, (uint8_t)139, (uint8_t)118, (uint8_t)74, (uint8_t)176, (uint8_t)113, (uint8_t)222, (uint8_t)105, (uint8_t)194, (uint8_t)143, (uint8_t)171, (uint8_t)25, (uint8_t)162, (uint8_t)10, (uint8_t)70, (uint8_t)145, (uint8_t)147, (uint8_t)153, (uint8_t)39, (uint8_t)185, (uint8_t)207, (uint8_t)74, (uint8_t)189, (uint8_t)58, (uint8_t)198, (uint8_t)122, (uint8_t)66, (uint8_t)92, (uint8_t)120, (uint8_t)82, (uint8_t)161, (uint8_t)33, (uint8_t)107, (uint8_t)138, (uint8_t)184, (uint8_t)30, (uint8_t)91, (uint8_t)255, (uint8_t)192, (uint8_t)252, (uint8_t)98, (uint8_t)240, (uint8_t)154, (uint8_t)126, (uint8_t)153, (uint8_t)49, (uint8_t)9, (uint8_t)47, (uint8_t)228, (uint8_t)139, (uint8_t)253, (uint8_t)90, (uint8_t)185, (uint8_t)4, (uint8_t)237, (uint8_t)172, (uint8_t)8, (uint8_t)226, (uint8_t)218, (uint8_t)62, (uint8_t)194, (uint8_t)155, (uint8_t)234, (uint8_t)95, (uint8_t)21, (uint8_t)172, (uint8_t)12, (uint8_t)59, (uint8_t)81, (uint8_t)91, (uint8_t)78, (uint8_t)217, (uint8_t)116, (uint8_t)47, (uint8_t)221, (uint8_t)186, (uint8_t)77, (uint8_t)43, (uint8_t)117, (uint8_t)74, (uint8_t)226, (uint8_t)242, (uint8_t)11, (uint8_t)69, (uint8_t)172, (uint8_t)70, (uint8_t)81, (uint8_t)209, (uint8_t)82, (uint8_t)83, (uint8_t)27, (uint8_t)29, (uint8_t)142, (uint8_t)179, (uint8_t)185, (uint8_t)151, (uint8_t)88, (uint8_t)69, (uint8_t)119, (uint8_t)157, (uint8_t)158, (uint8_t)188, (uint8_t)104, (uint8_t)23, (uint8_t)9, (uint8_t)17, (uint8_t)83, (uint8_t)41, (uint8_t)241, (uint8_t)80, (uint8_t)104, (uint8_t)198, (uint8_t)148, (uint8_t)77, (uint8_t)67, (uint8_t)157, (uint8_t)54, (uint8_t)186, (uint8_t)213, (uint8_t)95, (uint8_t)199, (uint8_t)252, (uint8_t)40, (uint8_t)204, (uint8_t)23, (uint8_t)41, (uint8_t)122, (uint8_t)94, (uint8_t)80, (uint8_t)163, (uint8_t)1, (uint8_t)142, (uint8_t)228, (uint8_t)33, (uint8_t)42, (uint8_t)248, (uint8_t)182, (uint8_t)250, (uint8_t)112, (uint8_t)228, (uint8_t)132, (uint8_t)173, (uint8_t)234, (uint8_t)155, (uint8_t)74, (uint8_t)11, (uint8_t)40, (uint8_t)194, (uint8_t)218, (uint8_t)91, (uint8_t)172, (uint8_t)71, (uint8_t)119, (uint8_t)36, (uint8_t)57, (uint8_t)220, (uint8_t)151, (uint8_t)73, (uint8_t)210, (uint8_t)175, (uint8_t)22, (uint8_t)74, (uint8_t)162, (uint8_t)31, (uint8_t)114, (uint8_t)66, (uint8_t)23, (uint8_t)81, (uint8_t)133, (uint8_t)127, (uint8_t)143, (uint8_t)170, (uint8_t)57, (uint8_t)128, (uint8_t)109, (uint8_t)1, (uint8_t)45, (uint8_t)179, (uint8_t)107, (uint8_t)181, (uint8_t)184, (uint8_t)128, (uint8_t)166, (uint8_t)33, (uint8_t)73, (uint8_t)184, (uint8_t)226, (uint8_t)36, (uint8_t)103, (uint8_t)53, (uint8_t)107, (uint8_t)245, (uint8_t)56, (uint8_t)94, (uint8_t)185, (uint8_t)150, (uint8_t)54, (uint8_t)146, (uint8_t)238, (uint8_t)239, (uint8_t)175, (uint8_t)6, (uint8_t)208, (uint8_t)123, (uint8_t)137, (uint8_t)231, (uint8_t)169, (uint8_t)28, (uint8_t)30} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)46622);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)40839);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)60197);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)32285);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3525231444L);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)7424);
    assert(p133_mask_GET(pack) == (uint64_t)5376098903523479303L);
    assert(p133_lon_GET(pack) == (int32_t) -201934445);
    assert(p133_lat_GET(pack) == (int32_t)130451606);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t)7909, (int16_t) -2545, (int16_t) -19694, (int16_t) -3363, (int16_t)16041, (int16_t) -8472, (int16_t)11983, (int16_t)23078, (int16_t) -18059, (int16_t) -6518, (int16_t)30168, (int16_t) -12034, (int16_t) -27865, (int16_t)20442, (int16_t)27778, (int16_t) -3790} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p134_lat_GET(pack) == (int32_t) -1126288938);
    assert(p134_lon_GET(pack) == (int32_t)1921228453);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)43259);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)1678525694);
    assert(p135_lon_GET(pack) == (int32_t) -999017217);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_current_height_GET(pack) == (float)1.7928413E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)38143);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)26712);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)63177);
    assert(p136_lon_GET(pack) == (int32_t) -47189490);
    assert(p136_lat_GET(pack) == (int32_t) -557327606);
    assert(p136_terrain_height_GET(pack) == (float)1.5162148E38F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)30128);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)952436588L);
    assert(p137_press_diff_GET(pack) == (float) -4.184477E37F);
    assert(p137_press_abs_GET(pack) == (float)2.2688853E38F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_z_GET(pack) == (float) -2.0129359E38F);
    assert(p138_y_GET(pack) == (float) -2.3908716E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)6788311365808006843L);
    {
        float exemplary[] =  {2.8489632E38F, -1.9783929E38F, -2.1572673E38F, -1.8559934E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float) -3.0153497E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)195);
    {
        float exemplary[] =  {-9.18818E36F, -3.322564E38F, 2.788617E37F, -3.4010765E38F, 5.8143026E37F, -3.4403685E37F, -2.5493655E37F, -1.3751208E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p139_time_usec_GET(pack) == (uint64_t)8144043233969516582L);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p140_time_usec_GET(pack) == (uint64_t)7687474178998607012L);
    {
        float exemplary[] =  {4.266482E37F, -7.8321463E37F, 2.6700137E38F, -9.4690375E36F, -2.0636175E38F, 5.275763E37F, -2.6534917E38F, -1.759357E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_amsl_GET(pack) == (float)2.0019645E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -2.1811683E37F);
    assert(p141_altitude_local_GET(pack) == (float) -1.6230814E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)3.3762044E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)7675684929751497188L);
    assert(p141_altitude_monotonic_GET(pack) == (float)1.391186E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -4.2665864E37F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)107);
    {
        uint8_t exemplary[] =  {(uint8_t)235, (uint8_t)233, (uint8_t)2, (uint8_t)82, (uint8_t)208, (uint8_t)215, (uint8_t)66, (uint8_t)62, (uint8_t)9, (uint8_t)191, (uint8_t)140, (uint8_t)143, (uint8_t)126, (uint8_t)186, (uint8_t)70, (uint8_t)145, (uint8_t)203, (uint8_t)7, (uint8_t)12, (uint8_t)83, (uint8_t)48, (uint8_t)233, (uint8_t)25, (uint8_t)124, (uint8_t)114, (uint8_t)223, (uint8_t)95, (uint8_t)158, (uint8_t)174, (uint8_t)150, (uint8_t)247, (uint8_t)207, (uint8_t)57, (uint8_t)1, (uint8_t)152, (uint8_t)120, (uint8_t)193, (uint8_t)179, (uint8_t)43, (uint8_t)204, (uint8_t)169, (uint8_t)96, (uint8_t)116, (uint8_t)247, (uint8_t)255, (uint8_t)110, (uint8_t)87, (uint8_t)226, (uint8_t)17, (uint8_t)163, (uint8_t)27, (uint8_t)185, (uint8_t)176, (uint8_t)104, (uint8_t)117, (uint8_t)56, (uint8_t)228, (uint8_t)123, (uint8_t)124, (uint8_t)49, (uint8_t)236, (uint8_t)24, (uint8_t)230, (uint8_t)207, (uint8_t)142, (uint8_t)16, (uint8_t)210, (uint8_t)200, (uint8_t)191, (uint8_t)106, (uint8_t)86, (uint8_t)144, (uint8_t)0, (uint8_t)123, (uint8_t)54, (uint8_t)228, (uint8_t)54, (uint8_t)3, (uint8_t)138, (uint8_t)169, (uint8_t)83, (uint8_t)31, (uint8_t)29, (uint8_t)152, (uint8_t)203, (uint8_t)94, (uint8_t)106, (uint8_t)33, (uint8_t)72, (uint8_t)116, (uint8_t)12, (uint8_t)37, (uint8_t)215, (uint8_t)155, (uint8_t)185, (uint8_t)244, (uint8_t)139, (uint8_t)229, (uint8_t)187, (uint8_t)180, (uint8_t)212, (uint8_t)102, (uint8_t)188, (uint8_t)232, (uint8_t)37, (uint8_t)36, (uint8_t)151, (uint8_t)137, (uint8_t)211, (uint8_t)26, (uint8_t)17, (uint8_t)89, (uint8_t)87, (uint8_t)54, (uint8_t)168, (uint8_t)132, (uint8_t)216, (uint8_t)130, (uint8_t)184, (uint8_t)107} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)234, (uint8_t)215, (uint8_t)178, (uint8_t)209, (uint8_t)84, (uint8_t)94, (uint8_t)36, (uint8_t)172, (uint8_t)143, (uint8_t)182, (uint8_t)221, (uint8_t)71, (uint8_t)93, (uint8_t)7, (uint8_t)73, (uint8_t)160, (uint8_t)166, (uint8_t)90, (uint8_t)254, (uint8_t)208, (uint8_t)5, (uint8_t)57, (uint8_t)124, (uint8_t)196, (uint8_t)41, (uint8_t)212, (uint8_t)166, (uint8_t)241, (uint8_t)110, (uint8_t)12, (uint8_t)58, (uint8_t)43, (uint8_t)91, (uint8_t)56, (uint8_t)159, (uint8_t)103, (uint8_t)82, (uint8_t)147, (uint8_t)177, (uint8_t)139, (uint8_t)29, (uint8_t)176, (uint8_t)241, (uint8_t)212, (uint8_t)20, (uint8_t)129, (uint8_t)235, (uint8_t)1, (uint8_t)133, (uint8_t)22, (uint8_t)208, (uint8_t)176, (uint8_t)206, (uint8_t)72, (uint8_t)62, (uint8_t)168, (uint8_t)38, (uint8_t)175, (uint8_t)205, (uint8_t)110, (uint8_t)69, (uint8_t)25, (uint8_t)6, (uint8_t)64, (uint8_t)251, (uint8_t)246, (uint8_t)30, (uint8_t)21, (uint8_t)63, (uint8_t)179, (uint8_t)192, (uint8_t)50, (uint8_t)101, (uint8_t)160, (uint8_t)199, (uint8_t)159, (uint8_t)35, (uint8_t)93, (uint8_t)97, (uint8_t)248, (uint8_t)199, (uint8_t)6, (uint8_t)5, (uint8_t)163, (uint8_t)165, (uint8_t)148, (uint8_t)137, (uint8_t)102, (uint8_t)166, (uint8_t)60, (uint8_t)132, (uint8_t)108, (uint8_t)4, (uint8_t)93, (uint8_t)247, (uint8_t)202, (uint8_t)117, (uint8_t)29, (uint8_t)141, (uint8_t)213, (uint8_t)41, (uint8_t)94, (uint8_t)202, (uint8_t)198, (uint8_t)178, (uint8_t)129, (uint8_t)213, (uint8_t)247, (uint8_t)41, (uint8_t)90, (uint8_t)151, (uint8_t)223, (uint8_t)219, (uint8_t)114, (uint8_t)6, (uint8_t)86, (uint8_t)19, (uint8_t)69, (uint8_t)177, (uint8_t)226} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_diff_GET(pack) == (float) -1.0501637E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)915320789L);
    assert(p143_press_abs_GET(pack) == (float)9.833381E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -25759);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_lon_GET(pack) == (int32_t)163106773);
    {
        float exemplary[] =  {9.248325E37F, -2.5210374E38F, -1.3234706E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)2537045181640322586L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)15);
    {
        float exemplary[] =  {-1.0326604E38F, 7.220848E37F, -1.768321E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)7902677224933319380L);
    {
        float exemplary[] =  {-2.4574626E38F, 8.3079985E37F, -2.4464918E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.0431628E38F, -6.295642E37F, -7.7203086E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)1.9052983E38F);
    {
        float exemplary[] =  {2.5292714E38F, 2.7784932E38F, -1.8883706E38F, -1.1201132E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -1051000547);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.3679586E38F, 4.4897897E37F, -1.1875946E38F, -6.3169856E36F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float) -8.216843E36F);
    assert(p146_yaw_rate_GET(pack) == (float) -1.8713667E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.579914E38F);
    assert(p146_x_acc_GET(pack) == (float)2.9781798E38F);
    assert(p146_z_acc_GET(pack) == (float)3.3589382E38F);
    assert(p146_x_vel_GET(pack) == (float) -2.83762E38F);
    assert(p146_z_vel_GET(pack) == (float)1.956881E38F);
    assert(p146_y_vel_GET(pack) == (float) -1.4505716E38F);
    assert(p146_y_pos_GET(pack) == (float)1.5027061E38F);
    {
        float exemplary[] =  {1.3434326E38F, -1.5623309E38F, 1.4219273E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float) -1.0986221E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)3935293713771575850L);
    assert(p146_z_pos_GET(pack) == (float) -1.7384015E38F);
    assert(p146_x_pos_GET(pack) == (float)2.8793598E38F);
    assert(p146_roll_rate_GET(pack) == (float)9.770724E37F);
    {
        float exemplary[] =  {-5.6313385E37F, 1.2825253E38F, -1.8364501E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -27184);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)35);
    assert(p147_energy_consumed_GET(pack) == (int32_t)357270241);
    assert(p147_current_consumed_GET(pack) == (int32_t)60716383);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -27774);
    {
        uint16_t exemplary[] =  {(uint16_t)35959, (uint16_t)55733, (uint16_t)33440, (uint16_t)47760, (uint16_t)52021, (uint16_t)61142, (uint16_t)50679, (uint16_t)22990, (uint16_t)57440, (uint16_t)20951} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)4228423922L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3310840116L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)18426);
    assert(p148_board_version_GET(pack) == (uint32_t)1955236772L);
    {
        uint8_t exemplary[] =  {(uint8_t)67, (uint8_t)83, (uint8_t)103, (uint8_t)240, (uint8_t)110, (uint8_t)107, (uint8_t)136, (uint8_t)79} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED));
    {
        uint8_t exemplary[] =  {(uint8_t)194, (uint8_t)118, (uint8_t)60, (uint8_t)202, (uint8_t)2, (uint8_t)196, (uint8_t)190, (uint8_t)186, (uint8_t)249, (uint8_t)80, (uint8_t)157, (uint8_t)37, (uint8_t)58, (uint8_t)124, (uint8_t)146, (uint8_t)25, (uint8_t)210, (uint8_t)212} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)179850250L);
    {
        uint8_t exemplary[] =  {(uint8_t)22, (uint8_t)125, (uint8_t)115, (uint8_t)184, (uint8_t)85, (uint8_t)145, (uint8_t)208, (uint8_t)82} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)1474796567999771983L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)17722);
    {
        uint8_t exemplary[] =  {(uint8_t)133, (uint8_t)62, (uint8_t)51, (uint8_t)45, (uint8_t)152, (uint8_t)100, (uint8_t)161, (uint8_t)255} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)40);
    assert(p149_size_y_GET(pack) == (float)1.288513E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    {
        float exemplary[] =  {1.7524026E37F, 1.5973942E38F, 1.1150264E38F, 8.650149E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_y_TRY(ph) == (float)1.1484832E38F);
    assert(p149_angle_y_GET(pack) == (float) -1.7124651E38F);
    assert(p149_angle_x_GET(pack) == (float)1.7847488E38F);
    assert(p149_x_TRY(ph) == (float) -4.139928E37F);
    assert(p149_distance_GET(pack) == (float) -3.068368E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)770957333631576448L);
    assert(p149_size_x_GET(pack) == (float)3.1380374E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p149_z_TRY(ph) == (float) -7.5764637E37F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
};


void c_CommunicationChannel_on_ARRAY_TEST_0_150(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -11, (int8_t) -56, (int8_t) -24, (int8_t) -78} ;
        int8_t*  sample = p150_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)104, (uint8_t)246, (uint8_t)227, (uint8_t)41} ;
        uint8_t*  sample = p150_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p150_v1_GET(pack) == (uint8_t)(uint8_t)88);
    {
        uint16_t exemplary[] =  {(uint16_t)5782, (uint16_t)16853, (uint16_t)34731, (uint16_t)29947} ;
        uint16_t*  sample = p150_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {200991789L, 3218202286L, 3050167179L, 655013630L} ;
        uint32_t*  sample = p150_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_1_151(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {2790157338L, 1393970172L, 3325349052L, 337743361L} ;
        uint32_t*  sample = p151_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_3_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_v_GET(pack) == (uint8_t)(uint8_t)95);
    {
        uint32_t exemplary[] =  {1323725041L, 1443414153L, 226112504L, 3040154566L} ;
        uint32_t*  sample = p153_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_4_154(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {457385148L, 1843650754L, 2632600807L, 4278329681L} ;
        uint32_t*  sample = p154_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p154_v_GET(pack) == (uint8_t)(uint8_t)86);
};


void c_CommunicationChannel_on_ARRAY_TEST_5_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_c2_LEN(ph) == 3);
    {
        char16_t * exemplary = u"pev";
        char16_t * sample = p155_c2_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p155_c1_LEN(ph) == 4);
    {
        char16_t * exemplary = u"rsky";
        char16_t * sample = p155_c1_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_6_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_v3_GET(pack) == (uint32_t)2243230783L);
    {
        int16_t exemplary[] =  {(int16_t) -27004, (int16_t)5390} ;
        int16_t*  sample = p156_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {615169844, 286821525} ;
        int32_t*  sample = p156_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {3436617778L, 54386205L} ;
        uint32_t*  sample = p156_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        double exemplary[] =  {-5.676126481802689E307, 9.505825766387631E307} ;
        double*  sample = p156_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_v2_GET(pack) == (uint16_t)(uint16_t)38507);
    {
        uint16_t exemplary[] =  {(uint16_t)11442, (uint16_t)40935} ;
        uint16_t*  sample = p156_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_v1_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p156_ar_c_LEN(ph) == 26);
    {
        char16_t * exemplary = u"hyfcvjwuxvbnmauygbCjjtgjjd";
        char16_t * sample = p156_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 52);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.1057348E38F, 5.850045E37F} ;
        float*  sample = p156_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)96} ;
        uint8_t*  sample = p156_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int8_t exemplary[] =  {(int8_t) -41, (int8_t) -48} ;
        int8_t*  sample = p156_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_7_157(Bounds_Inside * ph, Pack * pack)
{
    {
        double exemplary[] =  {3.8847536325401475E307, -9.520412933810854E307} ;
        double*  sample = p157_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)16193, (uint16_t)16897} ;
        uint16_t*  sample = p157_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int8_t exemplary[] =  {(int8_t)0, (int8_t) -112} ;
        int8_t*  sample = p157_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t) -28965, (int16_t)12312} ;
        int16_t*  sample = p157_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-6.496474E37F, -3.1105116E38F} ;
        float*  sample = p157_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {-2032517127, 453145279} ;
        int32_t*  sample = p157_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p157_ar_c_LEN(ph) == 18);
    {
        char16_t * exemplary = u"saungfeggmxIeYbrft";
        char16_t * sample = p157_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {1832362238L, 2874008704L} ;
        uint32_t*  sample = p157_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)66, (uint8_t)235} ;
        uint8_t*  sample = p157_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_8_158(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)23146, (uint16_t)29176} ;
        uint16_t*  sample = p158_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p158_v3_GET(pack) == (uint32_t)985454040L);
    {
        double exemplary[] =  {-1.7369277779164172E308, -5.580903771321988E307} ;
        double*  sample = p158_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_tas_ratio_GET(pack) == (float)2.0044503E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -2.458885E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.4340909E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.8910327E38F);
    assert(p230_vel_ratio_GET(pack) == (float)6.823571E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)90148554649163140L);
    assert(p230_hagl_ratio_GET(pack) == (float)1.131735E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE));
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -3.257225E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -2.9843246E37F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_horiz_accuracy_GET(pack) == (float)2.89031E38F);
    assert(p231_var_vert_GET(pack) == (float)2.8747407E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)3927655173479423878L);
    assert(p231_vert_accuracy_GET(pack) == (float)2.3439851E38F);
    assert(p231_wind_alt_GET(pack) == (float)2.0382889E38F);
    assert(p231_var_horiz_GET(pack) == (float) -1.7957897E38F);
    assert(p231_wind_z_GET(pack) == (float) -1.5532533E38F);
    assert(p231_wind_x_GET(pack) == (float)8.71852E37F);
    assert(p231_wind_y_GET(pack) == (float)1.310306E37F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP));
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)45976);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p232_speed_accuracy_GET(pack) == (float)1.5597756E38F);
    assert(p232_alt_GET(pack) == (float) -3.0330068E38F);
    assert(p232_hdop_GET(pack) == (float) -1.3545165E38F);
    assert(p232_vn_GET(pack) == (float) -3.182495E38F);
    assert(p232_lon_GET(pack) == (int32_t)1744975733);
    assert(p232_time_usec_GET(pack) == (uint64_t)2987914323238244942L);
    assert(p232_vert_accuracy_GET(pack) == (float)1.1402843E38F);
    assert(p232_vdop_GET(pack) == (float) -3.1455873E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -2.6750863E36F);
    assert(p232_ve_GET(pack) == (float)3.0688188E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)988407345L);
    assert(p232_vd_GET(pack) == (float) -2.9245184E38F);
    assert(p232_lat_GET(pack) == (int32_t)1652988382);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)224);
    {
        uint8_t exemplary[] =  {(uint8_t)225, (uint8_t)183, (uint8_t)63, (uint8_t)157, (uint8_t)60, (uint8_t)253, (uint8_t)189, (uint8_t)42, (uint8_t)85, (uint8_t)112, (uint8_t)133, (uint8_t)217, (uint8_t)222, (uint8_t)94, (uint8_t)80, (uint8_t)139, (uint8_t)129, (uint8_t)207, (uint8_t)133, (uint8_t)212, (uint8_t)64, (uint8_t)217, (uint8_t)171, (uint8_t)138, (uint8_t)232, (uint8_t)163, (uint8_t)6, (uint8_t)249, (uint8_t)183, (uint8_t)110, (uint8_t)241, (uint8_t)13, (uint8_t)220, (uint8_t)25, (uint8_t)171, (uint8_t)202, (uint8_t)230, (uint8_t)206, (uint8_t)243, (uint8_t)75, (uint8_t)221, (uint8_t)75, (uint8_t)11, (uint8_t)243, (uint8_t)85, (uint8_t)212, (uint8_t)246, (uint8_t)151, (uint8_t)40, (uint8_t)241, (uint8_t)89, (uint8_t)10, (uint8_t)241, (uint8_t)143, (uint8_t)119, (uint8_t)116, (uint8_t)146, (uint8_t)197, (uint8_t)202, (uint8_t)246, (uint8_t)69, (uint8_t)39, (uint8_t)78, (uint8_t)226, (uint8_t)131, (uint8_t)191, (uint8_t)92, (uint8_t)238, (uint8_t)158, (uint8_t)245, (uint8_t)233, (uint8_t)55, (uint8_t)181, (uint8_t)123, (uint8_t)72, (uint8_t)37, (uint8_t)44, (uint8_t)169, (uint8_t)132, (uint8_t)56, (uint8_t)225, (uint8_t)31, (uint8_t)233, (uint8_t)141, (uint8_t)237, (uint8_t)220, (uint8_t)199, (uint8_t)89, (uint8_t)221, (uint8_t)189, (uint8_t)215, (uint8_t)169, (uint8_t)142, (uint8_t)232, (uint8_t)98, (uint8_t)92, (uint8_t)169, (uint8_t)36, (uint8_t)89, (uint8_t)45, (uint8_t)88, (uint8_t)215, (uint8_t)177, (uint8_t)196, (uint8_t)110, (uint8_t)104, (uint8_t)135, (uint8_t)156, (uint8_t)126, (uint8_t)214, (uint8_t)202, (uint8_t)180, (uint8_t)225, (uint8_t)79, (uint8_t)74, (uint8_t)226, (uint8_t)26, (uint8_t)57, (uint8_t)49, (uint8_t)218, (uint8_t)187, (uint8_t)13, (uint8_t)228, (uint8_t)225, (uint8_t)235, (uint8_t)67, (uint8_t)8, (uint8_t)46, (uint8_t)207, (uint8_t)160, (uint8_t)160, (uint8_t)165, (uint8_t)73, (uint8_t)102, (uint8_t)252, (uint8_t)63, (uint8_t)109, (uint8_t)113, (uint8_t)37, (uint8_t)240, (uint8_t)115, (uint8_t)96, (uint8_t)213, (uint8_t)25, (uint8_t)61, (uint8_t)40, (uint8_t)175, (uint8_t)121, (uint8_t)174, (uint8_t)252, (uint8_t)161, (uint8_t)5, (uint8_t)212, (uint8_t)229, (uint8_t)144, (uint8_t)221, (uint8_t)22, (uint8_t)190, (uint8_t)71, (uint8_t)248, (uint8_t)187, (uint8_t)105, (uint8_t)241, (uint8_t)148, (uint8_t)2, (uint8_t)203, (uint8_t)4, (uint8_t)138, (uint8_t)179, (uint8_t)207, (uint8_t)254, (uint8_t)216, (uint8_t)45, (uint8_t)200, (uint8_t)25, (uint8_t)178, (uint8_t)189, (uint8_t)104, (uint8_t)247, (uint8_t)95} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)9321);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -63);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)23249);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)1362);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)8399);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -4593);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)14);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)26183);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p234_latitude_GET(pack) == (int32_t)1754639526);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED));
    assert(p234_custom_mode_GET(pack) == (uint32_t)3697017404L);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)28946);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -22);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)33);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p234_longitude_GET(pack) == (int32_t) -873108756);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_time_usec_GET(pack) == (uint64_t)3769409311068546626L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)702941688L);
    assert(p241_vibration_y_GET(pack) == (float) -1.9706087E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)459427748L);
    assert(p241_vibration_x_GET(pack) == (float)3.1024722E38F);
    assert(p241_vibration_z_GET(pack) == (float)1.099249E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)119887554L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.3899234E38F, -3.259716E38F, 2.4817163E38F, -2.796832E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_altitude_GET(pack) == (int32_t)705034989);
    assert(p242_approach_x_GET(pack) == (float) -8.64221E37F);
    assert(p242_latitude_GET(pack) == (int32_t)778474367);
    assert(p242_x_GET(pack) == (float) -2.9063773E37F);
    assert(p242_longitude_GET(pack) == (int32_t)299157718);
    assert(p242_z_GET(pack) == (float)1.5300556E38F);
    assert(p242_approach_z_GET(pack) == (float) -1.9838752E38F);
    assert(p242_y_GET(pack) == (float) -1.2526044E38F);
    assert(p242_approach_y_GET(pack) == (float) -3.3164247E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)7372663302914146179L);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-7.8360466E37F, -2.5262719E38F, -5.6275797E37F, 1.4040673E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_y_GET(pack) == (float)2.3838679E38F);
    assert(p243_latitude_GET(pack) == (int32_t)912516051);
    assert(p243_approach_x_GET(pack) == (float)3.178479E38F);
    assert(p243_x_GET(pack) == (float)1.5702829E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.1789309E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p243_longitude_GET(pack) == (int32_t)388498236);
    assert(p243_z_GET(pack) == (float)1.6978365E38F);
    assert(p243_altitude_GET(pack) == (int32_t)2051162958);
    assert(p243_time_usec_TRY(ph) == (uint64_t)8310788440890370371L);
    assert(p243_y_GET(pack) == (float) -3.2648658E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)138174084);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)46426);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED));
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p246_lat_GET(pack) == (int32_t)1588805790);
    assert(p246_callsign_LEN(ph) == 1);
    {
        char16_t * exemplary = u"t";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)45342);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)48472);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)20804);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2115454360L);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)27569);
    assert(p246_lon_GET(pack) == (int32_t) -1891183009);
    assert(p246_altitude_GET(pack) == (int32_t)522981379);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)1.8177063E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)2.8895371E38F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.1923115E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_id_GET(pack) == (uint32_t)474681608L);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)104);
    {
        uint8_t exemplary[] =  {(uint8_t)244, (uint8_t)26, (uint8_t)34, (uint8_t)201, (uint8_t)247, (uint8_t)172, (uint8_t)182, (uint8_t)40, (uint8_t)152, (uint8_t)110, (uint8_t)202, (uint8_t)181, (uint8_t)68, (uint8_t)174, (uint8_t)179, (uint8_t)74, (uint8_t)13, (uint8_t)10, (uint8_t)224, (uint8_t)2, (uint8_t)133, (uint8_t)157, (uint8_t)41, (uint8_t)103, (uint8_t)188, (uint8_t)80, (uint8_t)62, (uint8_t)161, (uint8_t)184, (uint8_t)190, (uint8_t)98, (uint8_t)1, (uint8_t)29, (uint8_t)67, (uint8_t)20, (uint8_t)234, (uint8_t)66, (uint8_t)37, (uint8_t)24, (uint8_t)221, (uint8_t)52, (uint8_t)116, (uint8_t)84, (uint8_t)255, (uint8_t)175, (uint8_t)48, (uint8_t)249, (uint8_t)191, (uint8_t)161, (uint8_t)231, (uint8_t)43, (uint8_t)134, (uint8_t)130, (uint8_t)86, (uint8_t)24, (uint8_t)136, (uint8_t)149, (uint8_t)94, (uint8_t)242, (uint8_t)254, (uint8_t)204, (uint8_t)197, (uint8_t)160, (uint8_t)157, (uint8_t)14, (uint8_t)153, (uint8_t)20, (uint8_t)20, (uint8_t)206, (uint8_t)31, (uint8_t)15, (uint8_t)76, (uint8_t)65, (uint8_t)179, (uint8_t)43, (uint8_t)152, (uint8_t)139, (uint8_t)124, (uint8_t)255, (uint8_t)40, (uint8_t)232, (uint8_t)227, (uint8_t)210, (uint8_t)70, (uint8_t)151, (uint8_t)33, (uint8_t)84, (uint8_t)127, (uint8_t)225, (uint8_t)18, (uint8_t)157, (uint8_t)164, (uint8_t)161, (uint8_t)50, (uint8_t)75, (uint8_t)183, (uint8_t)154, (uint8_t)3, (uint8_t)55, (uint8_t)110, (uint8_t)255, (uint8_t)144, (uint8_t)13, (uint8_t)207, (uint8_t)143, (uint8_t)236, (uint8_t)10, (uint8_t)43, (uint8_t)130, (uint8_t)30, (uint8_t)134, (uint8_t)199, (uint8_t)100, (uint8_t)218, (uint8_t)169, (uint8_t)158, (uint8_t)255, (uint8_t)128, (uint8_t)240, (uint8_t)103, (uint8_t)122, (uint8_t)52, (uint8_t)220, (uint8_t)163, (uint8_t)59, (uint8_t)238, (uint8_t)39, (uint8_t)255, (uint8_t)132, (uint8_t)171, (uint8_t)36, (uint8_t)148, (uint8_t)112, (uint8_t)3, (uint8_t)42, (uint8_t)234, (uint8_t)126, (uint8_t)244, (uint8_t)218, (uint8_t)118, (uint8_t)35, (uint8_t)220, (uint8_t)164, (uint8_t)123, (uint8_t)111, (uint8_t)129, (uint8_t)107, (uint8_t)19, (uint8_t)207, (uint8_t)21, (uint8_t)204, (uint8_t)90, (uint8_t)170, (uint8_t)148, (uint8_t)104, (uint8_t)21, (uint8_t)63, (uint8_t)81, (uint8_t)217, (uint8_t)11, (uint8_t)43, (uint8_t)64, (uint8_t)7, (uint8_t)53, (uint8_t)37, (uint8_t)48, (uint8_t)138, (uint8_t)86, (uint8_t)186, (uint8_t)35, (uint8_t)71, (uint8_t)140, (uint8_t)136, (uint8_t)184, (uint8_t)243, (uint8_t)251, (uint8_t)223, (uint8_t)239, (uint8_t)143, (uint8_t)127, (uint8_t)81, (uint8_t)19, (uint8_t)8, (uint8_t)166, (uint8_t)241, (uint8_t)54, (uint8_t)210, (uint8_t)91, (uint8_t)234, (uint8_t)254, (uint8_t)182, (uint8_t)214, (uint8_t)29, (uint8_t)76, (uint8_t)33, (uint8_t)228, (uint8_t)203, (uint8_t)73, (uint8_t)10, (uint8_t)11, (uint8_t)76, (uint8_t)130, (uint8_t)247, (uint8_t)189, (uint8_t)149, (uint8_t)27, (uint8_t)88, (uint8_t)167, (uint8_t)91, (uint8_t)99, (uint8_t)111, (uint8_t)225, (uint8_t)135, (uint8_t)159, (uint8_t)202, (uint8_t)81, (uint8_t)155, (uint8_t)29, (uint8_t)84, (uint8_t)116, (uint8_t)74, (uint8_t)33, (uint8_t)161, (uint8_t)210, (uint8_t)138, (uint8_t)41, (uint8_t)145, (uint8_t)184, (uint8_t)193, (uint8_t)205, (uint8_t)21, (uint8_t)167, (uint8_t)85, (uint8_t)171, (uint8_t)80, (uint8_t)220, (uint8_t)20, (uint8_t)16, (uint8_t)45, (uint8_t)128, (uint8_t)136, (uint8_t)16, (uint8_t)69, (uint8_t)246, (uint8_t)158, (uint8_t)64, (uint8_t)10, (uint8_t)105, (uint8_t)40} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)63344);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)128);
    {
        int8_t exemplary[] =  {(int8_t)77, (int8_t) -35, (int8_t) -7, (int8_t) -25, (int8_t)68, (int8_t)46, (int8_t) -70, (int8_t)73, (int8_t)0, (int8_t) -14, (int8_t) -55, (int8_t)22, (int8_t)101, (int8_t)20, (int8_t) -99, (int8_t)120, (int8_t)127, (int8_t)16, (int8_t) -90, (int8_t)105, (int8_t) -27, (int8_t)72, (int8_t) -107, (int8_t) -30, (int8_t) -80, (int8_t)27, (int8_t)22, (int8_t)110, (int8_t) -59, (int8_t)15, (int8_t)123, (int8_t) -92} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float) -1.7337507E38F);
    assert(p250_x_GET(pack) == (float)3.350157E38F);
    assert(p250_z_GET(pack) == (float)1.8167766E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5725698315186161870L);
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"grsaj";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -2.9234035E38F);
    assert(p251_name_LEN(ph) == 7);
    {
        char16_t * exemplary = u"bwjbutJ";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)70550626L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t)1039354930);
    assert(p252_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"eyhf";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1947673926L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_DEBUG);
    assert(p253_text_LEN(ph) == 8);
    {
        char16_t * exemplary = u"Ukynubuq";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float) -2.661974E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)344260765L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)743517577514623283L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)40, (uint8_t)156, (uint8_t)250, (uint8_t)145, (uint8_t)15, (uint8_t)64, (uint8_t)55, (uint8_t)213, (uint8_t)233, (uint8_t)84, (uint8_t)125, (uint8_t)211, (uint8_t)65, (uint8_t)41, (uint8_t)15, (uint8_t)239, (uint8_t)25, (uint8_t)176, (uint8_t)199, (uint8_t)135, (uint8_t)180, (uint8_t)249, (uint8_t)85, (uint8_t)17, (uint8_t)16, (uint8_t)139, (uint8_t)28, (uint8_t)152, (uint8_t)6, (uint8_t)10, (uint8_t)191} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)3890706026L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2559170119L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 24);
    {
        char16_t * exemplary = u"qgxjutiqtefbwjwlipkkvhwp";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)215);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_focal_length_GET(pack) == (float)2.6299772E37F);
    assert(p259_sensor_size_h_GET(pack) == (float) -2.3851868E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)168, (uint8_t)65, (uint8_t)88, (uint8_t)88, (uint8_t)152, (uint8_t)53, (uint8_t)12, (uint8_t)229, (uint8_t)159, (uint8_t)103, (uint8_t)45, (uint8_t)95, (uint8_t)30, (uint8_t)24, (uint8_t)129, (uint8_t)237, (uint8_t)237, (uint8_t)62, (uint8_t)186, (uint8_t)66, (uint8_t)18, (uint8_t)173, (uint8_t)10, (uint8_t)18, (uint8_t)128, (uint8_t)231, (uint8_t)31, (uint8_t)23, (uint8_t)223, (uint8_t)62, (uint8_t)245, (uint8_t)140} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float) -1.2184333E38F);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)50809);
    {
        uint8_t exemplary[] =  {(uint8_t)236, (uint8_t)159, (uint8_t)71, (uint8_t)227, (uint8_t)148, (uint8_t)123, (uint8_t)171, (uint8_t)86, (uint8_t)131, (uint8_t)207, (uint8_t)154, (uint8_t)232, (uint8_t)25, (uint8_t)45, (uint8_t)78, (uint8_t)64, (uint8_t)226, (uint8_t)168, (uint8_t)121, (uint8_t)76, (uint8_t)185, (uint8_t)235, (uint8_t)12, (uint8_t)81, (uint8_t)95, (uint8_t)115, (uint8_t)26, (uint8_t)134, (uint8_t)206, (uint8_t)238, (uint8_t)129, (uint8_t)150} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3297609011L);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)13777);
    assert(p259_cam_definition_uri_LEN(ph) == 65);
    {
        char16_t * exemplary = u"vvIowdIpawqfhbclmmxgasrzpTlasyvpczsfrulfVuuyAgubQvwcyoarSdipknqjL";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 130);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_firmware_version_GET(pack) == (uint32_t)2701732863L);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)64948);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)24);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)248810581L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1066333929L);
    assert(p261_used_capacity_GET(pack) == (float) -2.5679662E38F);
    assert(p261_read_speed_GET(pack) == (float) -3.1717738E38F);
    assert(p261_total_capacity_GET(pack) == (float) -5.0129304E37F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p261_write_speed_GET(pack) == (float) -7.47629E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p261_available_capacity_GET(pack) == (float) -1.4660218E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)39);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float)2.2104105E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1738790857L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2757859543L);
    assert(p262_available_capacity_GET(pack) == (float)1.9551464E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_file_url_LEN(ph) == 49);
    {
        char16_t * exemplary = u"XqdmckUgxwqnjofXeajHsfVbddeuhjndcgqwflladhbsvdrde";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 98);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.6814327E38F, -2.6137006E38F, -3.3365856E38F, 3.400568E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t) -742125986);
    assert(p263_alt_GET(pack) == (int32_t) -2146254574);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p263_image_index_GET(pack) == (int32_t) -1599595911);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1238207462L);
    assert(p263_lon_GET(pack) == (int32_t) -1504634033);
    assert(p263_time_utc_GET(pack) == (uint64_t)298530228091122358L);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -113);
    assert(p263_lat_GET(pack) == (int32_t)33617688);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3240022778L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)4096324315966967708L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)614549206019726109L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)6534873161212456747L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float) -1.3190496E38F);
    assert(p265_roll_GET(pack) == (float) -7.0604254E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)3007528026L);
    assert(p265_yaw_GET(pack) == (float)8.696323E37F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)43, (uint8_t)251, (uint8_t)65, (uint8_t)157, (uint8_t)181, (uint8_t)21, (uint8_t)110, (uint8_t)48, (uint8_t)17, (uint8_t)80, (uint8_t)179, (uint8_t)175, (uint8_t)15, (uint8_t)118, (uint8_t)5, (uint8_t)162, (uint8_t)185, (uint8_t)33, (uint8_t)218, (uint8_t)77, (uint8_t)239, (uint8_t)191, (uint8_t)164, (uint8_t)128, (uint8_t)58, (uint8_t)188, (uint8_t)49, (uint8_t)167, (uint8_t)136, (uint8_t)41, (uint8_t)13, (uint8_t)104, (uint8_t)151, (uint8_t)200, (uint8_t)30, (uint8_t)164, (uint8_t)136, (uint8_t)94, (uint8_t)109, (uint8_t)44, (uint8_t)187, (uint8_t)202, (uint8_t)131, (uint8_t)237, (uint8_t)243, (uint8_t)239, (uint8_t)106, (uint8_t)26, (uint8_t)105, (uint8_t)2, (uint8_t)223, (uint8_t)194, (uint8_t)191, (uint8_t)17, (uint8_t)179, (uint8_t)105, (uint8_t)246, (uint8_t)31, (uint8_t)143, (uint8_t)185, (uint8_t)51, (uint8_t)65, (uint8_t)172, (uint8_t)161, (uint8_t)112, (uint8_t)169, (uint8_t)43, (uint8_t)235, (uint8_t)235, (uint8_t)232, (uint8_t)170, (uint8_t)138, (uint8_t)90, (uint8_t)85, (uint8_t)149, (uint8_t)24, (uint8_t)239, (uint8_t)196, (uint8_t)239, (uint8_t)59, (uint8_t)202, (uint8_t)6, (uint8_t)60, (uint8_t)171, (uint8_t)37, (uint8_t)58, (uint8_t)212, (uint8_t)129, (uint8_t)80, (uint8_t)42, (uint8_t)72, (uint8_t)91, (uint8_t)192, (uint8_t)142, (uint8_t)120, (uint8_t)169, (uint8_t)100, (uint8_t)60, (uint8_t)30, (uint8_t)184, (uint8_t)242, (uint8_t)170, (uint8_t)246, (uint8_t)133, (uint8_t)39, (uint8_t)121, (uint8_t)211, (uint8_t)89, (uint8_t)84, (uint8_t)129, (uint8_t)33, (uint8_t)173, (uint8_t)81, (uint8_t)245, (uint8_t)101, (uint8_t)102, (uint8_t)141, (uint8_t)196, (uint8_t)47, (uint8_t)218, (uint8_t)127, (uint8_t)96, (uint8_t)209, (uint8_t)70, (uint8_t)205, (uint8_t)111, (uint8_t)144, (uint8_t)21, (uint8_t)10, (uint8_t)85, (uint8_t)240, (uint8_t)19, (uint8_t)208, (uint8_t)96, (uint8_t)62, (uint8_t)125, (uint8_t)158, (uint8_t)141, (uint8_t)205, (uint8_t)115, (uint8_t)128, (uint8_t)52, (uint8_t)65, (uint8_t)20, (uint8_t)229, (uint8_t)148, (uint8_t)102, (uint8_t)78, (uint8_t)172, (uint8_t)58, (uint8_t)64, (uint8_t)249, (uint8_t)211, (uint8_t)14, (uint8_t)163, (uint8_t)120, (uint8_t)105, (uint8_t)80, (uint8_t)22, (uint8_t)208, (uint8_t)170, (uint8_t)126, (uint8_t)10, (uint8_t)43, (uint8_t)12, (uint8_t)18, (uint8_t)30, (uint8_t)164, (uint8_t)231, (uint8_t)253, (uint8_t)202, (uint8_t)95, (uint8_t)178, (uint8_t)154, (uint8_t)48, (uint8_t)242, (uint8_t)89, (uint8_t)23, (uint8_t)111, (uint8_t)15, (uint8_t)143, (uint8_t)26, (uint8_t)132, (uint8_t)134, (uint8_t)21, (uint8_t)52, (uint8_t)139, (uint8_t)60, (uint8_t)91, (uint8_t)221, (uint8_t)105, (uint8_t)196, (uint8_t)135, (uint8_t)253, (uint8_t)139, (uint8_t)209, (uint8_t)38, (uint8_t)213, (uint8_t)247, (uint8_t)47, (uint8_t)82, (uint8_t)172, (uint8_t)191, (uint8_t)123, (uint8_t)220, (uint8_t)36, (uint8_t)65, (uint8_t)136, (uint8_t)58, (uint8_t)187, (uint8_t)167, (uint8_t)69, (uint8_t)178, (uint8_t)37, (uint8_t)151, (uint8_t)140, (uint8_t)226, (uint8_t)44, (uint8_t)50, (uint8_t)72, (uint8_t)16, (uint8_t)154, (uint8_t)77, (uint8_t)188, (uint8_t)67, (uint8_t)2, (uint8_t)56, (uint8_t)126, (uint8_t)152, (uint8_t)241, (uint8_t)128, (uint8_t)55, (uint8_t)159, (uint8_t)139, (uint8_t)87, (uint8_t)14, (uint8_t)128, (uint8_t)151, (uint8_t)27, (uint8_t)7, (uint8_t)32, (uint8_t)36, (uint8_t)148, (uint8_t)29, (uint8_t)238, (uint8_t)214, (uint8_t)16, (uint8_t)40, (uint8_t)8} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)16087);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)233);
    {
        uint8_t exemplary[] =  {(uint8_t)20, (uint8_t)147, (uint8_t)48, (uint8_t)199, (uint8_t)45, (uint8_t)150, (uint8_t)13, (uint8_t)181, (uint8_t)204, (uint8_t)68, (uint8_t)95, (uint8_t)69, (uint8_t)84, (uint8_t)122, (uint8_t)237, (uint8_t)89, (uint8_t)192, (uint8_t)32, (uint8_t)180, (uint8_t)72, (uint8_t)196, (uint8_t)227, (uint8_t)142, (uint8_t)178, (uint8_t)41, (uint8_t)233, (uint8_t)1, (uint8_t)134, (uint8_t)157, (uint8_t)157, (uint8_t)43, (uint8_t)175, (uint8_t)238, (uint8_t)24, (uint8_t)32, (uint8_t)127, (uint8_t)0, (uint8_t)121, (uint8_t)107, (uint8_t)70, (uint8_t)86, (uint8_t)147, (uint8_t)126, (uint8_t)141, (uint8_t)249, (uint8_t)9, (uint8_t)169, (uint8_t)14, (uint8_t)89, (uint8_t)241, (uint8_t)0, (uint8_t)187, (uint8_t)32, (uint8_t)232, (uint8_t)235, (uint8_t)244, (uint8_t)117, (uint8_t)106, (uint8_t)233, (uint8_t)227, (uint8_t)204, (uint8_t)120, (uint8_t)158, (uint8_t)16, (uint8_t)0, (uint8_t)203, (uint8_t)149, (uint8_t)52, (uint8_t)15, (uint8_t)115, (uint8_t)42, (uint8_t)68, (uint8_t)86, (uint8_t)193, (uint8_t)169, (uint8_t)209, (uint8_t)138, (uint8_t)8, (uint8_t)82, (uint8_t)190, (uint8_t)193, (uint8_t)202, (uint8_t)38, (uint8_t)181, (uint8_t)21, (uint8_t)235, (uint8_t)239, (uint8_t)101, (uint8_t)72, (uint8_t)167, (uint8_t)152, (uint8_t)92, (uint8_t)182, (uint8_t)71, (uint8_t)251, (uint8_t)59, (uint8_t)142, (uint8_t)215, (uint8_t)97, (uint8_t)101, (uint8_t)237, (uint8_t)113, (uint8_t)132, (uint8_t)138, (uint8_t)42, (uint8_t)118, (uint8_t)92, (uint8_t)106, (uint8_t)125, (uint8_t)245, (uint8_t)228, (uint8_t)254, (uint8_t)118, (uint8_t)238, (uint8_t)97, (uint8_t)50, (uint8_t)51, (uint8_t)243, (uint8_t)124, (uint8_t)72, (uint8_t)83, (uint8_t)26, (uint8_t)127, (uint8_t)162, (uint8_t)6, (uint8_t)227, (uint8_t)187, (uint8_t)134, (uint8_t)73, (uint8_t)84, (uint8_t)97, (uint8_t)173, (uint8_t)145, (uint8_t)101, (uint8_t)110, (uint8_t)204, (uint8_t)209, (uint8_t)123, (uint8_t)207, (uint8_t)110, (uint8_t)131, (uint8_t)11, (uint8_t)49, (uint8_t)192, (uint8_t)158, (uint8_t)51, (uint8_t)130, (uint8_t)65, (uint8_t)148, (uint8_t)172, (uint8_t)86, (uint8_t)84, (uint8_t)211, (uint8_t)7, (uint8_t)36, (uint8_t)123, (uint8_t)13, (uint8_t)66, (uint8_t)231, (uint8_t)87, (uint8_t)202, (uint8_t)129, (uint8_t)79, (uint8_t)225, (uint8_t)235, (uint8_t)55, (uint8_t)23, (uint8_t)87, (uint8_t)0, (uint8_t)255, (uint8_t)51, (uint8_t)238, (uint8_t)234, (uint8_t)209, (uint8_t)81, (uint8_t)130, (uint8_t)7, (uint8_t)165, (uint8_t)161, (uint8_t)154, (uint8_t)9, (uint8_t)155, (uint8_t)255, (uint8_t)90, (uint8_t)3, (uint8_t)9, (uint8_t)113, (uint8_t)240, (uint8_t)104, (uint8_t)56, (uint8_t)56, (uint8_t)245, (uint8_t)43, (uint8_t)224, (uint8_t)228, (uint8_t)154, (uint8_t)143, (uint8_t)61, (uint8_t)65, (uint8_t)114, (uint8_t)50, (uint8_t)131, (uint8_t)60, (uint8_t)31, (uint8_t)99, (uint8_t)47, (uint8_t)127, (uint8_t)164, (uint8_t)161, (uint8_t)190, (uint8_t)60, (uint8_t)11, (uint8_t)54, (uint8_t)8, (uint8_t)209, (uint8_t)217, (uint8_t)103, (uint8_t)190, (uint8_t)126, (uint8_t)64, (uint8_t)72, (uint8_t)174, (uint8_t)214, (uint8_t)98, (uint8_t)205, (uint8_t)32, (uint8_t)131, (uint8_t)112, (uint8_t)6, (uint8_t)226, (uint8_t)126, (uint8_t)243, (uint8_t)113, (uint8_t)255, (uint8_t)208, (uint8_t)25, (uint8_t)47, (uint8_t)162, (uint8_t)92, (uint8_t)136, (uint8_t)107, (uint8_t)174, (uint8_t)199, (uint8_t)81, (uint8_t)21, (uint8_t)121, (uint8_t)229, (uint8_t)172, (uint8_t)248} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)42905);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)64211);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)34);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)60673);
    assert(p269_bitrate_GET(pack) == (uint32_t)155490002L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)46563);
    assert(p269_framerate_GET(pack) == (float) -1.7293393E38F);
    assert(p269_uri_LEN(ph) == 1);
    {
        char16_t * exemplary = u"f";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)61539);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float)9.149451E37F);
    assert(p270_uri_LEN(ph) == 167);
    {
        char16_t * exemplary = u"pfkvrzjiyrcrvbvMvydvsfqewstqbysxoljtmotiputrgjhFwCaRvsHjwcjxtxKnuzpxtyfpfcwxLmiunulgxtvvetsuerppltcatmvdgDkjjlxlyxigUudisijonmvzjmpzmQfvvtczvtovcgbvsbxdxbgsddeyrufkmri";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 334);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)2379547223L);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)23021);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)61282);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)46988);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)74);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 2);
    {
        char16_t * exemplary = u"sv";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 61);
    {
        char16_t * exemplary = u"gmdweyfkpeGpiptcvxynwozQGmbcZFobjpszhgxuqVuirfzynfbGhEbgiwjwi";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 122);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)60928);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)16209);
    {
        uint8_t exemplary[] =  {(uint8_t)154, (uint8_t)246, (uint8_t)123, (uint8_t)195, (uint8_t)249, (uint8_t)29, (uint8_t)175, (uint8_t)9} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)6413);
    {
        uint8_t exemplary[] =  {(uint8_t)237, (uint8_t)131, (uint8_t)44, (uint8_t)149, (uint8_t)211, (uint8_t)229, (uint8_t)103, (uint8_t)98} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)16183552L);
    assert(p310_time_usec_GET(pack) == (uint64_t)2136164674491232892L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)34785);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1916334047L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3557761435L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p311_name_LEN(ph) == 52);
    {
        char16_t * exemplary = u"mlvzjwtkpmqfohdzpuzcrzfngczgwdqizqsyagqrDPzqptzpDjsn";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 104);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)5653438397623360828L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)245);
    {
        uint8_t exemplary[] =  {(uint8_t)171, (uint8_t)65, (uint8_t)114, (uint8_t)22, (uint8_t)139, (uint8_t)153, (uint8_t)177, (uint8_t)12, (uint8_t)165, (uint8_t)76, (uint8_t)163, (uint8_t)36, (uint8_t)138, (uint8_t)118, (uint8_t)19, (uint8_t)222} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p320_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"xJnkyysxes";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -31542);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)94);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)184);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p322_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"cDuwep";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 108);
    {
        char16_t * exemplary = u"dhChpmjkhosrsgiyecvpqtyprzjxHgkacaicpjaivcfNuusjukrDupnIqcuxzPhFHlrrvljhsczxslizmpdlojAgglnkudXxexcMecdckaim";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 216);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)2938);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)20897);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"jqIteP";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p323_param_value_LEN(ph) == 80);
    {
        char16_t * exemplary = u"ujhbsiywzralibizslcfYapkprMvpxerKbiOopgyvaoWlsNpfcyfotylhbjgLzzmtuckejoqCjwThxun";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 160);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_value_LEN(ph) == 15);
    {
        char16_t * exemplary = u"ryafWuvzcigzaci";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"ryv";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    assert(p330_time_usec_GET(pack) == (uint64_t)8928771683732725207L);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)44090);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)40218);
    {
        uint16_t exemplary[] =  {(uint16_t)30840, (uint16_t)61420, (uint16_t)20182, (uint16_t)47933, (uint16_t)41209, (uint16_t)55033, (uint16_t)28129, (uint16_t)40140, (uint16_t)6174, (uint16_t)35359, (uint16_t)47773, (uint16_t)41575, (uint16_t)23591, (uint16_t)36350, (uint16_t)63568, (uint16_t)60360, (uint16_t)33749, (uint16_t)33290, (uint16_t)32582, (uint16_t)60403, (uint16_t)39782, (uint16_t)16921, (uint16_t)9244, (uint16_t)7052, (uint16_t)25500, (uint16_t)50654, (uint16_t)13219, (uint16_t)11382, (uint16_t)35880, (uint16_t)63073, (uint16_t)14502, (uint16_t)19688, (uint16_t)40483, (uint16_t)42892, (uint16_t)41228, (uint16_t)47975, (uint16_t)23123, (uint16_t)5441, (uint16_t)64967, (uint16_t)51360, (uint16_t)45223, (uint16_t)154, (uint16_t)2248, (uint16_t)28093, (uint16_t)31446, (uint16_t)12279, (uint16_t)26998, (uint16_t)54199, (uint16_t)1979, (uint16_t)35699, (uint16_t)44497, (uint16_t)62363, (uint16_t)63198, (uint16_t)52726, (uint16_t)21599, (uint16_t)63941, (uint16_t)38000, (uint16_t)14633, (uint16_t)51267, (uint16_t)45970, (uint16_t)15274, (uint16_t)30165, (uint16_t)10493, (uint16_t)15494, (uint16_t)59671, (uint16_t)43969, (uint16_t)59577, (uint16_t)2354, (uint16_t)34572, (uint16_t)35964, (uint16_t)3206, (uint16_t)41918} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PPZ, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)871436993L, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_ONBOARD_CONTROLLER, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count3_SET((uint16_t)(uint16_t)52855, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH), PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)36025, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)55297, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)53654, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)38721, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -38, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)34040, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)61017, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)25366, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t)6785, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)6611651006765643611L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)492467128L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_time_boot_ms_SET((uint32_t)1731220167L, PH.base.pack) ;
        p3_afy_SET((float) -3.8991173E37F, PH.base.pack) ;
        p3_z_SET((float) -3.1055298E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)9.202929E37F, PH.base.pack) ;
        p3_y_SET((float) -2.67447E38F, PH.base.pack) ;
        p3_vy_SET((float) -1.4382453E37F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p3_x_SET((float)1.5823821E38F, PH.base.pack) ;
        p3_vz_SET((float) -2.1527105E38F, PH.base.pack) ;
        p3_afz_SET((float) -1.6305881E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)53900, PH.base.pack) ;
        p3_vx_SET((float)1.8723781E38F, PH.base.pack) ;
        p3_afx_SET((float) -2.982289E37F, PH.base.pack) ;
        p3_yaw_SET((float) -2.8443485E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)6302494446634245873L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p4_seq_SET((uint32_t)365994858L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        {
            char16_t* passkey = u"dRrymaoiejzceznzoymts";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"teWzaygolyhlhnaqiuepivW";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)2633885312L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        {
            char16_t* param_id = u"haeLtuysbjushN";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t) -13192, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"w";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float)3.8603812E37F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)63096, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)17462, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        {
            char16_t* param_id = u"yikbkmrpn";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p23_param_value_SET((float) -1.5280107E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)3636253548L, &PH) ;
        p24_alt_SET((int32_t) -1654815279, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)5655149024254006198L, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)13436, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1066124706L, &PH) ;
        p24_lon_SET((int32_t)670684767, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)304625170L, &PH) ;
        p24_lat_SET((int32_t) -75896297, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)3321982060L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -749806192, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)12619, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)28283, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)16161, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)31, (uint8_t)232, (uint8_t)132, (uint8_t)46, (uint8_t)30, (uint8_t)102, (uint8_t)186, (uint8_t)190, (uint8_t)198, (uint8_t)121, (uint8_t)234, (uint8_t)38, (uint8_t)178, (uint8_t)99, (uint8_t)47, (uint8_t)32, (uint8_t)224, (uint8_t)127, (uint8_t)0, (uint8_t)247};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)74, (uint8_t)89, (uint8_t)81, (uint8_t)18, (uint8_t)255, (uint8_t)47, (uint8_t)57, (uint8_t)0, (uint8_t)1, (uint8_t)211, (uint8_t)47, (uint8_t)197, (uint8_t)77, (uint8_t)157, (uint8_t)167, (uint8_t)196, (uint8_t)12, (uint8_t)158, (uint8_t)142, (uint8_t)238};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)116, (uint8_t)166, (uint8_t)95, (uint8_t)228, (uint8_t)31, (uint8_t)66, (uint8_t)154, (uint8_t)235, (uint8_t)63, (uint8_t)13, (uint8_t)121, (uint8_t)11, (uint8_t)103, (uint8_t)5, (uint8_t)59, (uint8_t)238, (uint8_t)122, (uint8_t)0, (uint8_t)199, (uint8_t)202};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        {
            uint8_t satellite_used[] =  {(uint8_t)171, (uint8_t)141, (uint8_t)57, (uint8_t)164, (uint8_t)98, (uint8_t)151, (uint8_t)48, (uint8_t)142, (uint8_t)237, (uint8_t)251, (uint8_t)219, (uint8_t)138, (uint8_t)2, (uint8_t)60, (uint8_t)242, (uint8_t)224, (uint8_t)230, (uint8_t)106, (uint8_t)110, (uint8_t)187};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)180, (uint8_t)13, (uint8_t)121, (uint8_t)31, (uint8_t)28, (uint8_t)15, (uint8_t)41, (uint8_t)225, (uint8_t)38, (uint8_t)13, (uint8_t)174, (uint8_t)194, (uint8_t)167, (uint8_t)254, (uint8_t)53, (uint8_t)236, (uint8_t)241, (uint8_t)51, (uint8_t)89, (uint8_t)187};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_zacc_SET((int16_t)(int16_t) -13291, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -14856, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -8576, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -11369, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)24922, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)21559, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)1653395348L, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)3155, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)18976, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -10669, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_xacc_SET((int16_t)(int16_t) -29934, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -3647, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -13496, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)27002, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)29694, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)25097, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -2595, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -31036, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -18541, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)9218885039736564971L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_temperature_SET((int16_t)(int16_t) -15840, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)6968413614037378389L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -1939, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -31157, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -24136, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float)1.4225954E38F, PH.base.pack) ;
        p29_press_abs_SET((float)6.5931547E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1398711674L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -32235, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float) -2.6078903E38F, PH.base.pack) ;
        p30_pitch_SET((float) -2.528399E38F, PH.base.pack) ;
        p30_roll_SET((float) -1.0697225E38F, PH.base.pack) ;
        p30_yawspeed_SET((float) -3.5494932E37F, PH.base.pack) ;
        p30_yaw_SET((float)1.7154736E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)3.1727424E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3629138493L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_time_boot_ms_SET((uint32_t)3664188803L, PH.base.pack) ;
        p31_q1_SET((float)8.880396E37F, PH.base.pack) ;
        p31_q2_SET((float) -1.0652034E38F, PH.base.pack) ;
        p31_rollspeed_SET((float)1.9504926E38F, PH.base.pack) ;
        p31_yawspeed_SET((float)9.612121E37F, PH.base.pack) ;
        p31_q4_SET((float)3.079741E37F, PH.base.pack) ;
        p31_pitchspeed_SET((float)6.2292046E37F, PH.base.pack) ;
        p31_q3_SET((float)6.212208E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_y_SET((float)1.7047377E38F, PH.base.pack) ;
        p32_vz_SET((float)1.9234693E38F, PH.base.pack) ;
        p32_vy_SET((float) -2.4418857E38F, PH.base.pack) ;
        p32_x_SET((float) -3.0304686E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)3328979060L, PH.base.pack) ;
        p32_vx_SET((float) -2.3212966E38F, PH.base.pack) ;
        p32_z_SET((float)2.7826148E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_vx_SET((int16_t)(int16_t)17566, PH.base.pack) ;
        p33_lat_SET((int32_t)1296900327, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -1122643787, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)4083379058L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)38056, PH.base.pack) ;
        p33_lon_SET((int32_t) -2005968202, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -19444, PH.base.pack) ;
        p33_alt_SET((int32_t)496298056, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)31865, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan3_scaled_SET((int16_t)(int16_t)14630, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)18886, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -24190, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -32339, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)3463, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)13333, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3095055223L, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -26966, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -30619, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan1_raw_SET((uint16_t)(uint16_t)28900, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)31114, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3736555819L, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)23514, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)65405, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)1300, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)54270, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)3204, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)39625, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo14_raw_SET((uint16_t)(uint16_t)64980, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)56765, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)1685, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)15080, &PH) ;
        p36_time_usec_SET((uint32_t)2927463588L, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)49082, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)34576, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)51969, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)47199, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)17365, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)17954, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)27165, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)58984, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)3061, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)64890, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)63174, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)56609, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)28773, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -17098, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -27014, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)5375, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p39_param1_SET((float)1.9016E38F, PH.base.pack) ;
        p39_x_SET((float)3.1507624E38F, PH.base.pack) ;
        p39_y_SET((float) -2.8709791E38F, PH.base.pack) ;
        p39_param2_SET((float) -2.6794315E38F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_STORAGE_INFORMATION, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)37854, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p39_param4_SET((float)2.731169E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p39_param3_SET((float)3.304267E38F, PH.base.pack) ;
        p39_z_SET((float)1.5647951E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)42534, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)17453, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)26825, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)49284, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)20197, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ACCEPTED, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -368294850, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p48_longitude_SET((int32_t) -868355018, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1471809522, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)3346985133315847267L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_longitude_SET((int32_t)276822135, PH.base.pack) ;
        p49_altitude_SET((int32_t) -83230994, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)8612738780002949568L, &PH) ;
        p49_latitude_SET((int32_t) -656647454, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value0_SET((float) -7.857354E37F, PH.base.pack) ;
        p50_param_value_max_SET((float)2.4438411E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -30789, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p50_param_value_min_SET((float)1.8041413E37F, PH.base.pack) ;
        {
            char16_t* param_id = u"ngbxmotx";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p50_scale_SET((float) -1.7163168E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)52931, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float)9.660682E37F, PH.base.pack) ;
        p54_p1x_SET((float) -2.0874972E38F, PH.base.pack) ;
        p54_p2z_SET((float) -6.201303E37F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p54_p1y_SET((float)1.116793E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p54_p2y_SET((float) -2.544604E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p54_p2x_SET((float)2.7398759E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float) -3.3822492E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p55_p2x_SET((float)7.769319E37F, PH.base.pack) ;
        p55_p2y_SET((float)1.3145149E38F, PH.base.pack) ;
        p55_p1y_SET((float) -1.3009032E37F, PH.base.pack) ;
        p55_p1z_SET((float)2.9484949E38F, PH.base.pack) ;
        p55_p2z_SET((float) -1.6298922E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        {
            float covariance[] =  {6.9841164E37F, 2.4518529E38F, -2.467665E38F, -3.2632052E38F, -4.292368E37F, 1.9098515E38F, 6.706663E36F, -9.980864E37F, -1.2132169E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)4854365606939103500L, PH.base.pack) ;
        {
            float q[] =  {6.3786997E37F, -2.1535585E38F, 6.6326553E36F, -2.99704E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)1.2130578E38F, PH.base.pack) ;
        p61_yawspeed_SET((float) -1.776749E38F, PH.base.pack) ;
        p61_rollspeed_SET((float)2.9085404E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_alt_error_SET((float) -1.9297778E38F, PH.base.pack) ;
        p62_aspd_error_SET((float) -1.1218654E38F, PH.base.pack) ;
        p62_nav_roll_SET((float)3.327999E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)13040, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.5338762E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)31592, PH.base.pack) ;
        p62_nav_pitch_SET((float) -2.7463E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)11579, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vy_SET((float) -1.0984095E38F, PH.base.pack) ;
        p63_lat_SET((int32_t)1467097246, PH.base.pack) ;
        p63_alt_SET((int32_t)422168745, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)1411558573325857449L, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p63_vx_SET((float) -2.0551364E38F, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -1164613176, PH.base.pack) ;
        p63_lon_SET((int32_t)1228408403, PH.base.pack) ;
        {
            float covariance[] =  {-1.3155995E38F, 1.3902299E38F, 3.2324167E38F, -2.7046384E36F, 2.297097E38F, 1.9694636E38F, 5.9869353E37F, 1.4521406E38F, 1.0127852E38F, -1.6785161E38F, 1.965986E38F, -8.952148E37F, -2.4322536E38F, -1.0640851E38F, 2.7698776E38F, 8.092147E36F, 1.7805613E38F, -2.9465625E38F, -2.3093485E38F, -3.288549E38F, 1.7172546E38F, -3.2910262E38F, -9.418213E37F, 3.2116666E38F, 3.04114E38F, 2.0476985E38F, 2.6738522E38F, 6.0526483E37F, 3.1584738E38F, -2.36446E38F, 2.5528704E38F, -2.598191E38F, 2.6559343E38F, -7.186295E37F, -1.8627738E38F, -2.2727576E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float) -2.7836451E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_z_SET((float)2.2283026E38F, PH.base.pack) ;
        p64_vz_SET((float)1.2864134E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.92428E38F, -1.148704E38F, -8.982149E37F, 4.893119E37F, 8.1797487E37F, -1.1895023E38F, 2.3970843E38F, -1.989887E38F, -2.9796423E38F, -1.8063772E38F, -5.5955497E37F, 3.681566E37F, -1.7847239E38F, -2.8715503E38F, 1.0663437E38F, -2.7141809E38F, 3.044285E38F, 7.365548E37F, 1.3451297E38F, -3.113809E38F, 1.77355E38F, -5.077064E37F, 3.2221147E38F, 6.326462E37F, 1.007861E38F, 1.1946256E38F, 2.9365628E37F, -1.971252E38F, -1.0230155E38F, 5.1961373E37F, -2.6450353E38F, -2.912981E38F, -6.6560566E37F, -1.0897444E38F, 2.8125843E38F, -2.0682405E38F, 6.0947353E37F, -6.8285224E37F, 2.853398E38F, -5.0792387E37F, 3.398376E38F, 1.3506833E38F, 1.0354983E38F, 2.2036954E38F, 3.2391075E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_ax_SET((float) -2.4883322E38F, PH.base.pack) ;
        p64_ay_SET((float) -3.3160807E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p64_y_SET((float) -1.1145502E37F, PH.base.pack) ;
        p64_az_SET((float) -4.6700253E37F, PH.base.pack) ;
        p64_vx_SET((float)1.9541595E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)3155314427644830505L, PH.base.pack) ;
        p64_x_SET((float)9.45743E36F, PH.base.pack) ;
        p64_vy_SET((float)5.568113E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan18_raw_SET((uint16_t)(uint16_t)36018, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)38628, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)62638, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)17775, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)41980, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)7888, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)12354, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)57965, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)21111, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)4164, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)34636, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)6231, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)7660, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)15277, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)52940, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)605879625L, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)51564, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)18221, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)4712, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)39348, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)18039, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_target_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)2000, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)30989, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -9659, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -6366, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)14366, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan5_raw_SET((uint16_t)(uint16_t)3203, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)14454, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)17564, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)499, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)58426, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)37525, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)39400, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)41442, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param2_SET((float) -2.044242E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, PH.base.pack) ;
        p73_param3_SET((float)9.852989E37F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p73_param4_SET((float)1.5125737E38F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p73_param1_SET((float) -2.0721443E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p73_y_SET((int32_t) -1572376029, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)49521, PH.base.pack) ;
        p73_x_SET((int32_t)1526948572, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p73_z_SET((float) -5.9672243E37F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_throttle_SET((uint16_t)(uint16_t)3131, PH.base.pack) ;
        p74_climb_SET((float) -2.4165984E38F, PH.base.pack) ;
        p74_alt_SET((float)2.1479698E38F, PH.base.pack) ;
        p74_groundspeed_SET((float) -3.402787E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -11611, PH.base.pack) ;
        p74_airspeed_SET((float)1.0485253E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p75_param3_SET((float)2.9233246E38F, PH.base.pack) ;
        p75_x_SET((int32_t) -390325247, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE, PH.base.pack) ;
        p75_param2_SET((float) -2.123491E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p75_z_SET((float)2.391306E38F, PH.base.pack) ;
        p75_param1_SET((float) -5.26136E37F, PH.base.pack) ;
        p75_y_SET((int32_t)547336148, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p75_param4_SET((float)2.6141884E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param3_SET((float)1.7587494E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p76_param1_SET((float) -3.2894736E38F, PH.base.pack) ;
        p76_param6_SET((float)1.0144138E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p76_param7_SET((float)1.5851074E38F, PH.base.pack) ;
        p76_param2_SET((float) -6.059154E37F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p76_param5_SET((float) -3.6362E36F, PH.base.pack) ;
        p76_param4_SET((float)4.921867E37F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)71, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)14, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_STREAMING, PH.base.pack) ;
        p77_result_param2_SET((int32_t)892402047, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)170, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float)1.5901317E38F, PH.base.pack) ;
        p81_roll_SET((float) -1.4661469E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p81_yaw_SET((float)5.2026896E37F, PH.base.pack) ;
        p81_thrust_SET((float) -2.4916887E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1036933286L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_time_boot_ms_SET((uint32_t)2535104121L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.6246406E38F, PH.base.pack) ;
        p82_thrust_SET((float)2.2869176E36F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)1.5548614E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        {
            float q[] =  {-1.0224555E38F, -2.8821393E38F, -1.4277284E38F, 1.848114E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_pitch_rate_SET((float) -2.7492426E37F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_pitch_rate_SET((float) -2.011818E38F, PH.base.pack) ;
        p83_thrust_SET((float)8.87309E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)3054172248L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        {
            float q[] =  {2.0106519E38F, 1.8717451E38F, -3.3856696E38F, -2.8201395E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)2.4577964E38F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)9.000205E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vx_SET((float)1.4782384E38F, PH.base.pack) ;
        p84_yaw_SET((float) -7.5928265E37F, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.9750814E38F, PH.base.pack) ;
        p84_afz_SET((float)4.5614607E36F, PH.base.pack) ;
        p84_vz_SET((float)5.0224327E37F, PH.base.pack) ;
        p84_z_SET((float) -3.1875865E38F, PH.base.pack) ;
        p84_x_SET((float)2.6381344E38F, PH.base.pack) ;
        p84_y_SET((float)2.0732533E38F, PH.base.pack) ;
        p84_afy_SET((float) -2.3990616E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)46002, PH.base.pack) ;
        p84_afx_SET((float) -5.080117E37F, PH.base.pack) ;
        p84_vy_SET((float) -1.18521E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2386034450L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_yaw_SET((float) -7.210482E37F, PH.base.pack) ;
        p86_vx_SET((float)2.6038474E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)2087876949, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p86_yaw_rate_SET((float) -6.435129E37F, PH.base.pack) ;
        p86_afz_SET((float)2.5419473E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)46439, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -72030663, PH.base.pack) ;
        p86_alt_SET((float)1.5212599E38F, PH.base.pack) ;
        p86_vz_SET((float)1.6301812E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3624882837L, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p86_vy_SET((float) -1.827532E38F, PH.base.pack) ;
        p86_afx_SET((float)2.8976876E38F, PH.base.pack) ;
        p86_afy_SET((float) -8.2673373E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vz_SET((float)2.3839946E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)758246354, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)348238432L, PH.base.pack) ;
        p87_vx_SET((float)6.1966595E37F, PH.base.pack) ;
        p87_afz_SET((float)2.3029388E38F, PH.base.pack) ;
        p87_yaw_SET((float) -8.506911E37F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p87_vy_SET((float)2.6227458E38F, PH.base.pack) ;
        p87_afx_SET((float) -3.243549E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.1030589E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -194065072, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)9982, PH.base.pack) ;
        p87_alt_SET((float) -1.516016E38F, PH.base.pack) ;
        p87_afy_SET((float)2.3306525E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_yaw_SET((float)2.7718466E38F, PH.base.pack) ;
        p89_x_SET((float)2.3351085E38F, PH.base.pack) ;
        p89_y_SET((float) -2.3733013E38F, PH.base.pack) ;
        p89_roll_SET((float) -2.1337517E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)1711734387L, PH.base.pack) ;
        p89_z_SET((float)1.6302703E37F, PH.base.pack) ;
        p89_pitch_SET((float) -2.4047975E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_yacc_SET((int16_t)(int16_t) -16395, PH.base.pack) ;
        p90_pitchspeed_SET((float) -3.0455519E38F, PH.base.pack) ;
        p90_roll_SET((float) -1.0227802E38F, PH.base.pack) ;
        p90_pitch_SET((float)3.3332276E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)12299, PH.base.pack) ;
        p90_yaw_SET((float)3.1437534E38F, PH.base.pack) ;
        p90_lat_SET((int32_t)1632889668, PH.base.pack) ;
        p90_rollspeed_SET((float) -2.4519652E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)32, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2875566780613275902L, PH.base.pack) ;
        p90_yawspeed_SET((float) -7.1002145E37F, PH.base.pack) ;
        p90_alt_SET((int32_t)473957599, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -3725, PH.base.pack) ;
        p90_lon_SET((int32_t)890075484, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)4936, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -29731, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_yaw_rudder_SET((float)9.58729E37F, PH.base.pack) ;
        p91_aux2_SET((float)2.0971336E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)2.904319E38F, PH.base.pack) ;
        p91_aux3_SET((float) -2.05094E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)3574366073108082075L, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p91_aux1_SET((float) -2.3923473E38F, PH.base.pack) ;
        p91_aux4_SET((float) -1.3287774E38F, PH.base.pack) ;
        p91_throttle_SET((float) -1.4897131E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -2.6737272E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan9_raw_SET((uint16_t)(uint16_t)62034, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)41364, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)6128562353897259889L, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)12442, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)55284, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)35812, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)44064, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)25501, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)16495, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)42865, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)43035, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)65185, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)35594, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)1074516196518442904L, PH.base.pack) ;
        p93_flags_SET((uint64_t)1906839943334649562L, PH.base.pack) ;
        {
            float controls[] =  {-2.659579E38F, 1.440456E37F, -1.8178613E38F, -2.8117932E38F, -1.4668692E38F, -2.5041373E38F, 2.4063972E38F, -9.850118E37F, -1.3453557E38F, 1.5900587E38F, -3.1374464E38F, -2.7913494E38F, -1.2085362E38F, 2.6269686E38F, -2.8572877E38F, -2.951357E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_y_SET((int16_t)(int16_t)1340, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)24015338748740162L, PH.base.pack) ;
        p100_ground_distance_SET((float) -2.2954704E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -2.7811078E38F, &PH) ;
        p100_flow_rate_x_SET((float)1.2041745E38F, &PH) ;
        p100_flow_comp_m_y_SET((float) -3.2457753E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -9.171393E36F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -27528, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)3063363755707340993L, PH.base.pack) ;
        p101_y_SET((float) -7.8664017E37F, PH.base.pack) ;
        p101_roll_SET((float) -2.8482561E38F, PH.base.pack) ;
        p101_yaw_SET((float) -1.648196E38F, PH.base.pack) ;
        p101_z_SET((float) -6.5377655E37F, PH.base.pack) ;
        p101_x_SET((float) -1.0470714E38F, PH.base.pack) ;
        p101_pitch_SET((float) -2.0738466E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float) -5.6172397E37F, PH.base.pack) ;
        p102_yaw_SET((float)9.230135E37F, PH.base.pack) ;
        p102_roll_SET((float) -2.7900062E37F, PH.base.pack) ;
        p102_x_SET((float)3.1441481E38F, PH.base.pack) ;
        p102_z_SET((float) -4.3327775E37F, PH.base.pack) ;
        p102_usec_SET((uint64_t)466094875094332831L, PH.base.pack) ;
        p102_pitch_SET((float) -6.8136E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float) -3.2380015E38F, PH.base.pack) ;
        p103_z_SET((float) -1.0279806E38F, PH.base.pack) ;
        p103_y_SET((float)3.4671152E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)8569109981523207379L, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_roll_SET((float)2.6494526E38F, PH.base.pack) ;
        p104_z_SET((float)5.8559054E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)3874027962599398964L, PH.base.pack) ;
        p104_x_SET((float)2.9980823E38F, PH.base.pack) ;
        p104_y_SET((float)1.4289651E37F, PH.base.pack) ;
        p104_pitch_SET((float) -2.3334705E38F, PH.base.pack) ;
        p104_yaw_SET((float)2.4368648E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_ymag_SET((float) -2.602572E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4314834508368526773L, PH.base.pack) ;
        p105_ygyro_SET((float) -1.8804262E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.991228E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)37605, PH.base.pack) ;
        p105_yacc_SET((float)2.850916E38F, PH.base.pack) ;
        p105_zacc_SET((float) -2.3313167E38F, PH.base.pack) ;
        p105_xmag_SET((float) -7.833256E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.8094936E38F, PH.base.pack) ;
        p105_xacc_SET((float)9.185729E36F, PH.base.pack) ;
        p105_temperature_SET((float) -3.3391022E38F, PH.base.pack) ;
        p105_xgyro_SET((float)1.3540088E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.054596E38F, PH.base.pack) ;
        p105_zmag_SET((float)1.5282807E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -3.3345486E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_usec_SET((uint64_t)7462121062121325875L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.822283E37F, PH.base.pack) ;
        p106_integrated_y_SET((float)1.7358226E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.244041E38F, PH.base.pack) ;
        p106_distance_SET((float) -2.8103666E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -2187, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)7511081L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)3.2702584E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -1.4794364E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)232416971L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_zgyro_SET((float)9.97767E37F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.7260814E38F, PH.base.pack) ;
        p107_zacc_SET((float) -5.5915784E37F, PH.base.pack) ;
        p107_zmag_SET((float)1.7267922E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)3560774013116263292L, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.5303187E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2237157470L, PH.base.pack) ;
        p107_xmag_SET((float)5.0210845E36F, PH.base.pack) ;
        p107_temperature_SET((float)3.279033E38F, PH.base.pack) ;
        p107_yacc_SET((float) -2.0426405E38F, PH.base.pack) ;
        p107_xacc_SET((float)6.516222E37F, PH.base.pack) ;
        p107_ymag_SET((float) -1.5712412E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -2.0690127E38F, PH.base.pack) ;
        p107_ygyro_SET((float) -9.373474E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.630373E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_lon_SET((float)2.5278707E38F, PH.base.pack) ;
        p108_yaw_SET((float)2.7416155E38F, PH.base.pack) ;
        p108_yacc_SET((float) -3.1776936E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -4.1979018E37F, PH.base.pack) ;
        p108_vd_SET((float) -1.756188E38F, PH.base.pack) ;
        p108_xacc_SET((float)2.4222028E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.1629762E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -2.9556318E38F, PH.base.pack) ;
        p108_ve_SET((float) -1.823243E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.9718432E38F, PH.base.pack) ;
        p108_q2_SET((float) -2.6264165E38F, PH.base.pack) ;
        p108_zacc_SET((float) -2.576013E37F, PH.base.pack) ;
        p108_lat_SET((float) -1.7222512E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -7.106244E37F, PH.base.pack) ;
        p108_vn_SET((float)2.5344351E38F, PH.base.pack) ;
        p108_q3_SET((float)5.632754E36F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.9789385E38F, PH.base.pack) ;
        p108_alt_SET((float) -2.8389366E38F, PH.base.pack) ;
        p108_q4_SET((float) -3.1924772E38F, PH.base.pack) ;
        p108_q1_SET((float) -2.385164E38F, PH.base.pack) ;
        p108_roll_SET((float)1.9140766E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_txbuf_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)13648, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)5810, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)74, (uint8_t)108, (uint8_t)199, (uint8_t)236, (uint8_t)191, (uint8_t)224, (uint8_t)114, (uint8_t)128, (uint8_t)216, (uint8_t)240, (uint8_t)210, (uint8_t)1, (uint8_t)171, (uint8_t)215, (uint8_t)46, (uint8_t)65, (uint8_t)132, (uint8_t)203, (uint8_t)95, (uint8_t)234, (uint8_t)94, (uint8_t)112, (uint8_t)176, (uint8_t)197, (uint8_t)237, (uint8_t)135, (uint8_t)180, (uint8_t)208, (uint8_t)108, (uint8_t)246, (uint8_t)69, (uint8_t)53, (uint8_t)67, (uint8_t)239, (uint8_t)157, (uint8_t)234, (uint8_t)42, (uint8_t)136, (uint8_t)247, (uint8_t)128, (uint8_t)42, (uint8_t)31, (uint8_t)49, (uint8_t)31, (uint8_t)58, (uint8_t)214, (uint8_t)169, (uint8_t)122, (uint8_t)3, (uint8_t)226, (uint8_t)215, (uint8_t)129, (uint8_t)33, (uint8_t)210, (uint8_t)229, (uint8_t)208, (uint8_t)174, (uint8_t)111, (uint8_t)10, (uint8_t)110, (uint8_t)84, (uint8_t)60, (uint8_t)48, (uint8_t)237, (uint8_t)99, (uint8_t)244, (uint8_t)155, (uint8_t)176, (uint8_t)238, (uint8_t)73, (uint8_t)186, (uint8_t)2, (uint8_t)143, (uint8_t)113, (uint8_t)133, (uint8_t)68, (uint8_t)245, (uint8_t)35, (uint8_t)249, (uint8_t)223, (uint8_t)43, (uint8_t)113, (uint8_t)129, (uint8_t)44, (uint8_t)80, (uint8_t)138, (uint8_t)191, (uint8_t)253, (uint8_t)193, (uint8_t)134, (uint8_t)92, (uint8_t)84, (uint8_t)146, (uint8_t)142, (uint8_t)219, (uint8_t)151, (uint8_t)158, (uint8_t)14, (uint8_t)209, (uint8_t)128, (uint8_t)121, (uint8_t)133, (uint8_t)230, (uint8_t)86, (uint8_t)48, (uint8_t)107, (uint8_t)114, (uint8_t)200, (uint8_t)187, (uint8_t)211, (uint8_t)45, (uint8_t)70, (uint8_t)210, (uint8_t)129, (uint8_t)171, (uint8_t)152, (uint8_t)30, (uint8_t)122, (uint8_t)30, (uint8_t)10, (uint8_t)60, (uint8_t)196, (uint8_t)222, (uint8_t)132, (uint8_t)90, (uint8_t)159, (uint8_t)156, (uint8_t)229, (uint8_t)161, (uint8_t)5, (uint8_t)27, (uint8_t)66, (uint8_t)232, (uint8_t)34, (uint8_t)163, (uint8_t)101, (uint8_t)186, (uint8_t)15, (uint8_t)126, (uint8_t)140, (uint8_t)128, (uint8_t)121, (uint8_t)79, (uint8_t)116, (uint8_t)198, (uint8_t)104, (uint8_t)243, (uint8_t)135, (uint8_t)60, (uint8_t)176, (uint8_t)150, (uint8_t)193, (uint8_t)106, (uint8_t)153, (uint8_t)38, (uint8_t)116, (uint8_t)90, (uint8_t)229, (uint8_t)101, (uint8_t)156, (uint8_t)8, (uint8_t)208, (uint8_t)109, (uint8_t)60, (uint8_t)171, (uint8_t)17, (uint8_t)3, (uint8_t)198, (uint8_t)109, (uint8_t)133, (uint8_t)19, (uint8_t)145, (uint8_t)173, (uint8_t)206, (uint8_t)147, (uint8_t)159, (uint8_t)55, (uint8_t)188, (uint8_t)114, (uint8_t)212, (uint8_t)24, (uint8_t)141, (uint8_t)5, (uint8_t)225, (uint8_t)81, (uint8_t)74, (uint8_t)171, (uint8_t)142, (uint8_t)9, (uint8_t)42, (uint8_t)121, (uint8_t)53, (uint8_t)37, (uint8_t)251, (uint8_t)34, (uint8_t)57, (uint8_t)57, (uint8_t)182, (uint8_t)25, (uint8_t)235, (uint8_t)44, (uint8_t)236, (uint8_t)248, (uint8_t)231, (uint8_t)150, (uint8_t)41, (uint8_t)146, (uint8_t)112, (uint8_t)138, (uint8_t)85, (uint8_t)120, (uint8_t)49, (uint8_t)12, (uint8_t)254, (uint8_t)226, (uint8_t)124, (uint8_t)64, (uint8_t)154, (uint8_t)168, (uint8_t)11, (uint8_t)235, (uint8_t)158, (uint8_t)158, (uint8_t)215, (uint8_t)221, (uint8_t)126, (uint8_t)117, (uint8_t)81, (uint8_t)146, (uint8_t)43, (uint8_t)49, (uint8_t)110, (uint8_t)129, (uint8_t)7, (uint8_t)219, (uint8_t)107, (uint8_t)108, (uint8_t)19, (uint8_t)81, (uint8_t)165, (uint8_t)136, (uint8_t)222, (uint8_t)135, (uint8_t)72, (uint8_t)116, (uint8_t)141, (uint8_t)199, (uint8_t)163, (uint8_t)248, (uint8_t)77, (uint8_t)155};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t) -4808969450363479193L, PH.base.pack) ;
        p111_ts1_SET((int64_t)4466127170473427585L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)107188697263573144L, PH.base.pack) ;
        p112_seq_SET((uint32_t)239404337L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_fix_type_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p113_lon_SET((int32_t)61169770, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)40287, PH.base.pack) ;
        p113_lat_SET((int32_t) -1427239501, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)59641, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)53968, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)7660, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)4072, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)3275, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p113_alt_SET((int32_t)736202634, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)62585, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)1997056478941307628L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_ygyro_SET((float) -2.511642E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)2226145207L, PH.base.pack) ;
        p114_integrated_y_SET((float) -2.3821094E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p114_distance_SET((float)1.0830284E36F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)711551704056499869L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -2.3535418E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -19864, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.1045647E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2226133378L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.8138535E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_rollspeed_SET((float)1.5603484E38F, PH.base.pack) ;
        p115_lat_SET((int32_t) -1745386364, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -3138, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)6924289734490413147L, PH.base.pack) ;
        p115_pitchspeed_SET((float) -3.1841493E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -25290, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)18461, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)13988, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)32737, PH.base.pack) ;
        p115_lon_SET((int32_t) -195769534, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)17332, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -2725, PH.base.pack) ;
        p115_yawspeed_SET((float)3.0019175E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-1.0687694E38F, 2.4313932E38F, -2.1720388E38F, 9.35792E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_alt_SET((int32_t) -346919627, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -19463, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_time_boot_ms_SET((uint32_t)3094726655L, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)15666, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -17727, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)15387, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -5003, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)17237, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)8999, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)10117, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)16812, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)31183, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)29082, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)26519, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)561117032L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)54954, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)47522, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)7313, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1959894889L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)4031110537L, PH.base.pack) ;
        p119_ofs_SET((uint32_t)4100230672L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)36629, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)148, (uint8_t)225, (uint8_t)58, (uint8_t)158, (uint8_t)11, (uint8_t)6, (uint8_t)140, (uint8_t)58, (uint8_t)33, (uint8_t)17, (uint8_t)19, (uint8_t)201, (uint8_t)44, (uint8_t)10, (uint8_t)167, (uint8_t)78, (uint8_t)145, (uint8_t)187, (uint8_t)219, (uint8_t)155, (uint8_t)165, (uint8_t)23, (uint8_t)31, (uint8_t)43, (uint8_t)101, (uint8_t)32, (uint8_t)235, (uint8_t)34, (uint8_t)194, (uint8_t)78, (uint8_t)52, (uint8_t)37, (uint8_t)185, (uint8_t)215, (uint8_t)237, (uint8_t)136, (uint8_t)232, (uint8_t)148, (uint8_t)127, (uint8_t)77, (uint8_t)172, (uint8_t)28, (uint8_t)40, (uint8_t)87, (uint8_t)212, (uint8_t)43, (uint8_t)10, (uint8_t)110, (uint8_t)74, (uint8_t)10, (uint8_t)253, (uint8_t)21, (uint8_t)51, (uint8_t)86, (uint8_t)120, (uint8_t)94, (uint8_t)37, (uint8_t)45, (uint8_t)120, (uint8_t)212, (uint8_t)188, (uint8_t)194, (uint8_t)95, (uint8_t)42, (uint8_t)128, (uint8_t)181, (uint8_t)12, (uint8_t)135, (uint8_t)136, (uint8_t)164, (uint8_t)178, (uint8_t)144, (uint8_t)245, (uint8_t)220, (uint8_t)94, (uint8_t)97, (uint8_t)210, (uint8_t)63, (uint8_t)14, (uint8_t)54, (uint8_t)9, (uint8_t)78, (uint8_t)205, (uint8_t)37, (uint8_t)141, (uint8_t)74, (uint8_t)159, (uint8_t)5, (uint8_t)163, (uint8_t)154};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_id_SET((uint16_t)(uint16_t)62476, PH.base.pack) ;
        p120_ofs_SET((uint32_t)3531183921L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)151, (uint8_t)156, (uint8_t)180, (uint8_t)208, (uint8_t)175, (uint8_t)135, (uint8_t)91, (uint8_t)40, (uint8_t)106, (uint8_t)115, (uint8_t)122, (uint8_t)253, (uint8_t)37, (uint8_t)218, (uint8_t)46, (uint8_t)219, (uint8_t)182, (uint8_t)32, (uint8_t)147, (uint8_t)231, (uint8_t)195, (uint8_t)251, (uint8_t)219, (uint8_t)61, (uint8_t)209, (uint8_t)66, (uint8_t)254, (uint8_t)221, (uint8_t)107, (uint8_t)219, (uint8_t)71, (uint8_t)95, (uint8_t)130, (uint8_t)49, (uint8_t)45, (uint8_t)83, (uint8_t)231, (uint8_t)142, (uint8_t)19, (uint8_t)247, (uint8_t)181, (uint8_t)204, (uint8_t)6, (uint8_t)18, (uint8_t)94, (uint8_t)84, (uint8_t)133, (uint8_t)111, (uint8_t)140, (uint8_t)33, (uint8_t)213, (uint8_t)78, (uint8_t)255, (uint8_t)87, (uint8_t)122, (uint8_t)240, (uint8_t)85, (uint8_t)241, (uint8_t)155, (uint8_t)8, (uint8_t)32, (uint8_t)199, (uint8_t)41, (uint8_t)160, (uint8_t)204, (uint8_t)34, (uint8_t)250, (uint8_t)136, (uint8_t)81, (uint8_t)47, (uint8_t)3, (uint8_t)39, (uint8_t)8, (uint8_t)50, (uint8_t)36, (uint8_t)155, (uint8_t)97, (uint8_t)148, (uint8_t)8, (uint8_t)183, (uint8_t)59, (uint8_t)145, (uint8_t)44, (uint8_t)207, (uint8_t)151, (uint8_t)101, (uint8_t)248, (uint8_t)36, (uint8_t)27, (uint8_t)233, (uint8_t)240, (uint8_t)114, (uint8_t)208, (uint8_t)177, (uint8_t)110, (uint8_t)226, (uint8_t)19, (uint8_t)10, (uint8_t)43, (uint8_t)217, (uint8_t)80, (uint8_t)176, (uint8_t)231, (uint8_t)187, (uint8_t)79, (uint8_t)85, (uint8_t)52, (uint8_t)230, (uint8_t)245, (uint8_t)171};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_lat_SET((int32_t)1555729842, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)16510, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)37470, PH.base.pack) ;
        p124_alt_SET((int32_t) -2086863732, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)63181, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)28024, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6686221928487639060L, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)404762167L, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_lon_SET((int32_t)1965869035, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)59073, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)33757, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_count_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)119, (uint8_t)43, (uint8_t)125, (uint8_t)89, (uint8_t)76, (uint8_t)58, (uint8_t)39, (uint8_t)45, (uint8_t)92, (uint8_t)75, (uint8_t)176, (uint8_t)103, (uint8_t)117, (uint8_t)65, (uint8_t)164, (uint8_t)99, (uint8_t)158, (uint8_t)144, (uint8_t)247, (uint8_t)170, (uint8_t)88, (uint8_t)187, (uint8_t)16, (uint8_t)98, (uint8_t)17, (uint8_t)229, (uint8_t)251, (uint8_t)225, (uint8_t)236, (uint8_t)10, (uint8_t)69, (uint8_t)21, (uint8_t)252, (uint8_t)133, (uint8_t)123, (uint8_t)229, (uint8_t)64, (uint8_t)116, (uint8_t)35, (uint8_t)88, (uint8_t)183, (uint8_t)198, (uint8_t)80, (uint8_t)21, (uint8_t)57, (uint8_t)17, (uint8_t)214, (uint8_t)164, (uint8_t)41, (uint8_t)229, (uint8_t)93, (uint8_t)40, (uint8_t)204, (uint8_t)154, (uint8_t)78, (uint8_t)197, (uint8_t)186, (uint8_t)116, (uint8_t)234, (uint8_t)174, (uint8_t)150, (uint8_t)37, (uint8_t)125, (uint8_t)110, (uint8_t)142, (uint8_t)36, (uint8_t)18, (uint8_t)226, (uint8_t)134, (uint8_t)86};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)1566, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)2525311889L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_wn_SET((uint16_t)(uint16_t)8767, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)3381908578L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)2080525735, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)843386258, PH.base.pack) ;
        p127_tow_SET((uint32_t)3792058414L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1958421470, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1795630538, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2593122813L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_nsats_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)2214651760L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)9742, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1771901990, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)4112938188L, PH.base.pack) ;
        p128_tow_SET((uint32_t)923922609L, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)934091774, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1450758137, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -2114908491, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ymag_SET((int16_t)(int16_t)13691, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)7680, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -4391, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)3749147331L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)3980, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -29696, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -30049, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -16267, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t)17878, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)29129, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_packets_SET((uint16_t)(uint16_t)30630, PH.base.pack) ;
        p130_size_SET((uint32_t)36721486L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)29929, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)8672, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)250, (uint8_t)0, (uint8_t)91, (uint8_t)20, (uint8_t)63, (uint8_t)53, (uint8_t)82, (uint8_t)17, (uint8_t)152, (uint8_t)50, (uint8_t)212, (uint8_t)185, (uint8_t)115, (uint8_t)75, (uint8_t)2, (uint8_t)192, (uint8_t)237, (uint8_t)250, (uint8_t)205, (uint8_t)30, (uint8_t)35, (uint8_t)126, (uint8_t)128, (uint8_t)226, (uint8_t)3, (uint8_t)220, (uint8_t)162, (uint8_t)221, (uint8_t)228, (uint8_t)55, (uint8_t)252, (uint8_t)225, (uint8_t)181, (uint8_t)133, (uint8_t)230, (uint8_t)114, (uint8_t)54, (uint8_t)209, (uint8_t)30, (uint8_t)67, (uint8_t)90, (uint8_t)79, (uint8_t)205, (uint8_t)32, (uint8_t)39, (uint8_t)139, (uint8_t)118, (uint8_t)74, (uint8_t)176, (uint8_t)113, (uint8_t)222, (uint8_t)105, (uint8_t)194, (uint8_t)143, (uint8_t)171, (uint8_t)25, (uint8_t)162, (uint8_t)10, (uint8_t)70, (uint8_t)145, (uint8_t)147, (uint8_t)153, (uint8_t)39, (uint8_t)185, (uint8_t)207, (uint8_t)74, (uint8_t)189, (uint8_t)58, (uint8_t)198, (uint8_t)122, (uint8_t)66, (uint8_t)92, (uint8_t)120, (uint8_t)82, (uint8_t)161, (uint8_t)33, (uint8_t)107, (uint8_t)138, (uint8_t)184, (uint8_t)30, (uint8_t)91, (uint8_t)255, (uint8_t)192, (uint8_t)252, (uint8_t)98, (uint8_t)240, (uint8_t)154, (uint8_t)126, (uint8_t)153, (uint8_t)49, (uint8_t)9, (uint8_t)47, (uint8_t)228, (uint8_t)139, (uint8_t)253, (uint8_t)90, (uint8_t)185, (uint8_t)4, (uint8_t)237, (uint8_t)172, (uint8_t)8, (uint8_t)226, (uint8_t)218, (uint8_t)62, (uint8_t)194, (uint8_t)155, (uint8_t)234, (uint8_t)95, (uint8_t)21, (uint8_t)172, (uint8_t)12, (uint8_t)59, (uint8_t)81, (uint8_t)91, (uint8_t)78, (uint8_t)217, (uint8_t)116, (uint8_t)47, (uint8_t)221, (uint8_t)186, (uint8_t)77, (uint8_t)43, (uint8_t)117, (uint8_t)74, (uint8_t)226, (uint8_t)242, (uint8_t)11, (uint8_t)69, (uint8_t)172, (uint8_t)70, (uint8_t)81, (uint8_t)209, (uint8_t)82, (uint8_t)83, (uint8_t)27, (uint8_t)29, (uint8_t)142, (uint8_t)179, (uint8_t)185, (uint8_t)151, (uint8_t)88, (uint8_t)69, (uint8_t)119, (uint8_t)157, (uint8_t)158, (uint8_t)188, (uint8_t)104, (uint8_t)23, (uint8_t)9, (uint8_t)17, (uint8_t)83, (uint8_t)41, (uint8_t)241, (uint8_t)80, (uint8_t)104, (uint8_t)198, (uint8_t)148, (uint8_t)77, (uint8_t)67, (uint8_t)157, (uint8_t)54, (uint8_t)186, (uint8_t)213, (uint8_t)95, (uint8_t)199, (uint8_t)252, (uint8_t)40, (uint8_t)204, (uint8_t)23, (uint8_t)41, (uint8_t)122, (uint8_t)94, (uint8_t)80, (uint8_t)163, (uint8_t)1, (uint8_t)142, (uint8_t)228, (uint8_t)33, (uint8_t)42, (uint8_t)248, (uint8_t)182, (uint8_t)250, (uint8_t)112, (uint8_t)228, (uint8_t)132, (uint8_t)173, (uint8_t)234, (uint8_t)155, (uint8_t)74, (uint8_t)11, (uint8_t)40, (uint8_t)194, (uint8_t)218, (uint8_t)91, (uint8_t)172, (uint8_t)71, (uint8_t)119, (uint8_t)36, (uint8_t)57, (uint8_t)220, (uint8_t)151, (uint8_t)73, (uint8_t)210, (uint8_t)175, (uint8_t)22, (uint8_t)74, (uint8_t)162, (uint8_t)31, (uint8_t)114, (uint8_t)66, (uint8_t)23, (uint8_t)81, (uint8_t)133, (uint8_t)127, (uint8_t)143, (uint8_t)170, (uint8_t)57, (uint8_t)128, (uint8_t)109, (uint8_t)1, (uint8_t)45, (uint8_t)179, (uint8_t)107, (uint8_t)181, (uint8_t)184, (uint8_t)128, (uint8_t)166, (uint8_t)33, (uint8_t)73, (uint8_t)184, (uint8_t)226, (uint8_t)36, (uint8_t)103, (uint8_t)53, (uint8_t)107, (uint8_t)245, (uint8_t)56, (uint8_t)94, (uint8_t)185, (uint8_t)150, (uint8_t)54, (uint8_t)146, (uint8_t)238, (uint8_t)239, (uint8_t)175, (uint8_t)6, (uint8_t)208, (uint8_t)123, (uint8_t)137, (uint8_t)231, (uint8_t)169, (uint8_t)28, (uint8_t)30};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)46622, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)32285, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)60197, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)40839, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3525231444L, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)5376098903523479303L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)7424, PH.base.pack) ;
        p133_lon_SET((int32_t) -201934445, PH.base.pack) ;
        p133_lat_SET((int32_t)130451606, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_gridbit_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p134_lon_SET((int32_t)1921228453, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)43259, PH.base.pack) ;
        p134_lat_SET((int32_t) -1126288938, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)7909, (int16_t) -2545, (int16_t) -19694, (int16_t) -3363, (int16_t)16041, (int16_t) -8472, (int16_t)11983, (int16_t)23078, (int16_t) -18059, (int16_t) -6518, (int16_t)30168, (int16_t) -12034, (int16_t) -27865, (int16_t)20442, (int16_t)27778, (int16_t) -3790};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)1678525694, PH.base.pack) ;
        p135_lon_SET((int32_t) -999017217, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)38143, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)26712, PH.base.pack) ;
        p136_terrain_height_SET((float)1.5162148E38F, PH.base.pack) ;
        p136_lon_SET((int32_t) -47189490, PH.base.pack) ;
        p136_lat_SET((int32_t) -557327606, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)63177, PH.base.pack) ;
        p136_current_height_SET((float)1.7928413E38F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)952436588L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)30128, PH.base.pack) ;
        p137_press_diff_SET((float) -4.184477E37F, PH.base.pack) ;
        p137_press_abs_SET((float)2.2688853E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)6788311365808006843L, PH.base.pack) ;
        p138_y_SET((float) -2.3908716E38F, PH.base.pack) ;
        p138_z_SET((float) -2.0129359E38F, PH.base.pack) ;
        p138_x_SET((float) -3.0153497E38F, PH.base.pack) ;
        {
            float q[] =  {2.8489632E38F, -1.9783929E38F, -2.1572673E38F, -1.8559934E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        {
            float controls[] =  {-9.18818E36F, -3.322564E38F, 2.788617E37F, -3.4010765E38F, 5.8143026E37F, -3.4403685E37F, -2.5493655E37F, -1.3751208E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)8144043233969516582L, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)7687474178998607012L, PH.base.pack) ;
        {
            float controls[] =  {4.266482E37F, -7.8321463E37F, 2.6700137E38F, -9.4690375E36F, -2.0636175E38F, 5.275763E37F, -2.6534917E38F, -1.759357E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_terrain_SET((float)3.3762044E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.6230814E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -4.2665864E37F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)1.391186E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)2.0019645E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)7675684929751497188L, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -2.1811683E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)234, (uint8_t)215, (uint8_t)178, (uint8_t)209, (uint8_t)84, (uint8_t)94, (uint8_t)36, (uint8_t)172, (uint8_t)143, (uint8_t)182, (uint8_t)221, (uint8_t)71, (uint8_t)93, (uint8_t)7, (uint8_t)73, (uint8_t)160, (uint8_t)166, (uint8_t)90, (uint8_t)254, (uint8_t)208, (uint8_t)5, (uint8_t)57, (uint8_t)124, (uint8_t)196, (uint8_t)41, (uint8_t)212, (uint8_t)166, (uint8_t)241, (uint8_t)110, (uint8_t)12, (uint8_t)58, (uint8_t)43, (uint8_t)91, (uint8_t)56, (uint8_t)159, (uint8_t)103, (uint8_t)82, (uint8_t)147, (uint8_t)177, (uint8_t)139, (uint8_t)29, (uint8_t)176, (uint8_t)241, (uint8_t)212, (uint8_t)20, (uint8_t)129, (uint8_t)235, (uint8_t)1, (uint8_t)133, (uint8_t)22, (uint8_t)208, (uint8_t)176, (uint8_t)206, (uint8_t)72, (uint8_t)62, (uint8_t)168, (uint8_t)38, (uint8_t)175, (uint8_t)205, (uint8_t)110, (uint8_t)69, (uint8_t)25, (uint8_t)6, (uint8_t)64, (uint8_t)251, (uint8_t)246, (uint8_t)30, (uint8_t)21, (uint8_t)63, (uint8_t)179, (uint8_t)192, (uint8_t)50, (uint8_t)101, (uint8_t)160, (uint8_t)199, (uint8_t)159, (uint8_t)35, (uint8_t)93, (uint8_t)97, (uint8_t)248, (uint8_t)199, (uint8_t)6, (uint8_t)5, (uint8_t)163, (uint8_t)165, (uint8_t)148, (uint8_t)137, (uint8_t)102, (uint8_t)166, (uint8_t)60, (uint8_t)132, (uint8_t)108, (uint8_t)4, (uint8_t)93, (uint8_t)247, (uint8_t)202, (uint8_t)117, (uint8_t)29, (uint8_t)141, (uint8_t)213, (uint8_t)41, (uint8_t)94, (uint8_t)202, (uint8_t)198, (uint8_t)178, (uint8_t)129, (uint8_t)213, (uint8_t)247, (uint8_t)41, (uint8_t)90, (uint8_t)151, (uint8_t)223, (uint8_t)219, (uint8_t)114, (uint8_t)6, (uint8_t)86, (uint8_t)19, (uint8_t)69, (uint8_t)177, (uint8_t)226};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)235, (uint8_t)233, (uint8_t)2, (uint8_t)82, (uint8_t)208, (uint8_t)215, (uint8_t)66, (uint8_t)62, (uint8_t)9, (uint8_t)191, (uint8_t)140, (uint8_t)143, (uint8_t)126, (uint8_t)186, (uint8_t)70, (uint8_t)145, (uint8_t)203, (uint8_t)7, (uint8_t)12, (uint8_t)83, (uint8_t)48, (uint8_t)233, (uint8_t)25, (uint8_t)124, (uint8_t)114, (uint8_t)223, (uint8_t)95, (uint8_t)158, (uint8_t)174, (uint8_t)150, (uint8_t)247, (uint8_t)207, (uint8_t)57, (uint8_t)1, (uint8_t)152, (uint8_t)120, (uint8_t)193, (uint8_t)179, (uint8_t)43, (uint8_t)204, (uint8_t)169, (uint8_t)96, (uint8_t)116, (uint8_t)247, (uint8_t)255, (uint8_t)110, (uint8_t)87, (uint8_t)226, (uint8_t)17, (uint8_t)163, (uint8_t)27, (uint8_t)185, (uint8_t)176, (uint8_t)104, (uint8_t)117, (uint8_t)56, (uint8_t)228, (uint8_t)123, (uint8_t)124, (uint8_t)49, (uint8_t)236, (uint8_t)24, (uint8_t)230, (uint8_t)207, (uint8_t)142, (uint8_t)16, (uint8_t)210, (uint8_t)200, (uint8_t)191, (uint8_t)106, (uint8_t)86, (uint8_t)144, (uint8_t)0, (uint8_t)123, (uint8_t)54, (uint8_t)228, (uint8_t)54, (uint8_t)3, (uint8_t)138, (uint8_t)169, (uint8_t)83, (uint8_t)31, (uint8_t)29, (uint8_t)152, (uint8_t)203, (uint8_t)94, (uint8_t)106, (uint8_t)33, (uint8_t)72, (uint8_t)116, (uint8_t)12, (uint8_t)37, (uint8_t)215, (uint8_t)155, (uint8_t)185, (uint8_t)244, (uint8_t)139, (uint8_t)229, (uint8_t)187, (uint8_t)180, (uint8_t)212, (uint8_t)102, (uint8_t)188, (uint8_t)232, (uint8_t)37, (uint8_t)36, (uint8_t)151, (uint8_t)137, (uint8_t)211, (uint8_t)26, (uint8_t)17, (uint8_t)89, (uint8_t)87, (uint8_t)54, (uint8_t)168, (uint8_t)132, (uint8_t)216, (uint8_t)130, (uint8_t)184, (uint8_t)107};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t) -25759, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)915320789L, PH.base.pack) ;
        p143_press_diff_SET((float) -1.0501637E38F, PH.base.pack) ;
        p143_press_abs_SET((float)9.833381E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_lon_SET((int32_t)163106773, PH.base.pack) ;
        p144_lat_SET((int32_t) -1051000547, PH.base.pack) ;
        {
            float position_cov[] =  {9.248325E37F, -2.5210374E38F, -1.3234706E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {2.5292714E38F, 2.7784932E38F, -1.8883706E38F, -1.1201132E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)7902677224933319380L, PH.base.pack) ;
        {
            float vel[] =  {-1.0326604E38F, 7.220848E37F, -1.768321E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)1.9052983E38F, PH.base.pack) ;
        {
            float rates[] =  {-2.4574626E38F, 8.3079985E37F, -2.4464918E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)2537045181640322586L, PH.base.pack) ;
        {
            float acc[] =  {1.0431628E38F, -6.295642E37F, -7.7203086E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_pos_SET((float) -1.7384015E38F, PH.base.pack) ;
        p146_x_pos_SET((float)2.8793598E38F, PH.base.pack) ;
        p146_z_acc_SET((float)3.3589382E38F, PH.base.pack) ;
        p146_roll_rate_SET((float)9.770724E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {-5.6313385E37F, 1.2825253E38F, -1.8364501E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)3935293713771575850L, PH.base.pack) ;
        {
            float q[] =  {3.3679586E38F, 4.4897897E37F, -1.1875946E38F, -6.3169856E36F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_x_vel_SET((float) -2.83762E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -1.0986221E38F, PH.base.pack) ;
        p146_y_pos_SET((float)1.5027061E38F, PH.base.pack) ;
        p146_x_acc_SET((float)2.9781798E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.579914E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {1.3434326E38F, -1.5623309E38F, 1.4219273E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float)1.956881E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -1.4505716E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -1.8713667E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -8.216843E36F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_current_battery_SET((int16_t)(int16_t) -27184, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)35959, (uint16_t)55733, (uint16_t)33440, (uint16_t)47760, (uint16_t)52021, (uint16_t)61142, (uint16_t)50679, (uint16_t)22990, (uint16_t)57440, (uint16_t)20951};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)35, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)357270241, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -27774, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)60716383, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_os_sw_version_SET((uint32_t)4228423922L, PH.base.pack) ;
        p148_uid_SET((uint64_t)1474796567999771983L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)133, (uint8_t)62, (uint8_t)51, (uint8_t)45, (uint8_t)152, (uint8_t)100, (uint8_t)161, (uint8_t)255};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)67, (uint8_t)83, (uint8_t)103, (uint8_t)240, (uint8_t)110, (uint8_t)107, (uint8_t)136, (uint8_t)79};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)1955236772L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)194, (uint8_t)118, (uint8_t)60, (uint8_t)202, (uint8_t)2, (uint8_t)196, (uint8_t)190, (uint8_t)186, (uint8_t)249, (uint8_t)80, (uint8_t)157, (uint8_t)37, (uint8_t)58, (uint8_t)124, (uint8_t)146, (uint8_t)25, (uint8_t)210, (uint8_t)212};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)22, (uint8_t)125, (uint8_t)115, (uint8_t)184, (uint8_t)85, (uint8_t)145, (uint8_t)208, (uint8_t)82};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)18426, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)179850250L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3310840116L, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED), PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)17722, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_x_SET((float) -4.139928E37F, &PH) ;
        p149_angle_y_SET((float) -1.7124651E38F, PH.base.pack) ;
        p149_distance_SET((float) -3.068368E38F, PH.base.pack) ;
        p149_angle_x_SET((float)1.7847488E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)40, &PH) ;
        p149_z_SET((float) -7.5764637E37F, &PH) ;
        p149_size_x_SET((float)3.1380374E38F, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p149_y_SET((float)1.1484832E38F, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)770957333631576448L, PH.base.pack) ;
        {
            float q[] =  {1.7524026E37F, 1.5973942E38F, 1.1150264E38F, 8.650149E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_size_y_SET((float)1.288513E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_0_150(), &PH);
        {
            uint32_t ar_u32[] =  {200991789L, 3218202286L, 3050167179L, 655013630L};
            p150_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        p150_v1_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        {
            int8_t ar_i8[] =  {(int8_t) -11, (int8_t) -56, (int8_t) -24, (int8_t) -78};
            p150_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)5782, (uint16_t)16853, (uint16_t)34731, (uint16_t)29947};
            p150_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            uint8_t ar_u8[] =  {(uint8_t)104, (uint8_t)246, (uint8_t)227, (uint8_t)41};
            p150_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_0_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_1_151(), &PH);
        {
            uint32_t ar_u32[] =  {2790157338L, 1393970172L, 3325349052L, 337743361L};
            p151_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_1_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_3_153(), &PH);
        p153_v_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        {
            uint32_t ar_u32[] =  {1323725041L, 1443414153L, 226112504L, 3040154566L};
            p153_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_3_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_4_154(), &PH);
        {
            uint32_t ar_u32[] =  {457385148L, 1843650754L, 2632600807L, 4278329681L};
            p154_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        p154_v_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_CommunicationChannel_on_ARRAY_TEST_4_154(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_5_155(), &PH);
        {
            char16_t* c2 = u"pev";
            p155_c2_SET_(c2, &PH) ;
        }
        {
            char16_t* c1 = u"rsky";
            p155_c1_SET_(c1, &PH) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_5_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_6_156(), &PH);
        {
            uint8_t ar_u8[] =  {(uint8_t)27, (uint8_t)96};
            p156_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t) -41, (int8_t) -48};
            p156_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            int16_t ar_i16[] =  {(int16_t) -27004, (int16_t)5390};
            p156_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        {
            float ar_f[] =  {2.1057348E38F, 5.850045E37F};
            p156_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)11442, (uint16_t)40935};
            p156_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            int32_t ar_i32[] =  {615169844, 286821525};
            p156_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        p156_v1_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p156_v3_SET((uint32_t)2243230783L, PH.base.pack) ;
        {
            char16_t* ar_c = u"hyfcvjwuxvbnmauygbCjjtgjjd";
            p156_ar_c_SET_(ar_c, &PH) ;
        }
        p156_v2_SET((uint16_t)(uint16_t)38507, PH.base.pack) ;
        {
            double ar_d[] =  {-5.676126481802689E307, 9.505825766387631E307};
            p156_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        {
            uint32_t ar_u32[] =  {3436617778L, 54386205L};
            p156_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_6_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_7_157(), &PH);
        {
            float ar_f[] =  {-6.496474E37F, -3.1105116E38F};
            p157_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t)0, (int8_t) -112};
            p157_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            uint8_t ar_u8[] =  {(uint8_t)66, (uint8_t)235};
            p157_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            uint32_t ar_u32[] =  {1832362238L, 2874008704L};
            p157_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)16193, (uint16_t)16897};
            p157_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            int32_t ar_i32[] =  {-2032517127, 453145279};
            p157_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        {
            double ar_d[] =  {3.8847536325401475E307, -9.520412933810854E307};
            p157_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        {
            int16_t ar_i16[] =  {(int16_t) -28965, (int16_t)12312};
            p157_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        {
            char16_t* ar_c = u"saungfeggmxIeYbrft";
            p157_ar_c_SET_(ar_c, &PH) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_7_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_8_158(), &PH);
        {
            uint16_t ar_u16[] =  {(uint16_t)23146, (uint16_t)29176};
            p158_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        p158_v3_SET((uint32_t)985454040L, PH.base.pack) ;
        {
            double ar_d[] =  {-1.7369277779164172E308, -5.580903771321988E307};
            p158_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_8_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE), PH.base.pack) ;
        p230_vel_ratio_SET((float)6.823571E37F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -3.257225E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -2.458885E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -1.4340909E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.8910327E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float)2.0044503E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)90148554649163140L, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.131735E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -2.9843246E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float) -1.5532533E38F, PH.base.pack) ;
        p231_wind_alt_SET((float)2.0382889E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -1.7957897E38F, PH.base.pack) ;
        p231_var_vert_SET((float)2.8747407E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.3439851E38F, PH.base.pack) ;
        p231_wind_x_SET((float)8.71852E37F, PH.base.pack) ;
        p231_wind_y_SET((float)1.310306E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.89031E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)3927655173479423878L, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_vn_SET((float) -3.182495E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)1.5597756E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.1402843E38F, PH.base.pack) ;
        p232_alt_SET((float) -3.0330068E38F, PH.base.pack) ;
        p232_vdop_SET((float) -3.1455873E38F, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP), PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)988407345L, PH.base.pack) ;
        p232_lon_SET((int32_t)1744975733, PH.base.pack) ;
        p232_hdop_SET((float) -1.3545165E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -2.6750863E36F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p232_lat_SET((int32_t)1652988382, PH.base.pack) ;
        p232_ve_SET((float)3.0688188E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)45976, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)2987914323238244942L, PH.base.pack) ;
        p232_vd_SET((float) -2.9245184E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)225, (uint8_t)183, (uint8_t)63, (uint8_t)157, (uint8_t)60, (uint8_t)253, (uint8_t)189, (uint8_t)42, (uint8_t)85, (uint8_t)112, (uint8_t)133, (uint8_t)217, (uint8_t)222, (uint8_t)94, (uint8_t)80, (uint8_t)139, (uint8_t)129, (uint8_t)207, (uint8_t)133, (uint8_t)212, (uint8_t)64, (uint8_t)217, (uint8_t)171, (uint8_t)138, (uint8_t)232, (uint8_t)163, (uint8_t)6, (uint8_t)249, (uint8_t)183, (uint8_t)110, (uint8_t)241, (uint8_t)13, (uint8_t)220, (uint8_t)25, (uint8_t)171, (uint8_t)202, (uint8_t)230, (uint8_t)206, (uint8_t)243, (uint8_t)75, (uint8_t)221, (uint8_t)75, (uint8_t)11, (uint8_t)243, (uint8_t)85, (uint8_t)212, (uint8_t)246, (uint8_t)151, (uint8_t)40, (uint8_t)241, (uint8_t)89, (uint8_t)10, (uint8_t)241, (uint8_t)143, (uint8_t)119, (uint8_t)116, (uint8_t)146, (uint8_t)197, (uint8_t)202, (uint8_t)246, (uint8_t)69, (uint8_t)39, (uint8_t)78, (uint8_t)226, (uint8_t)131, (uint8_t)191, (uint8_t)92, (uint8_t)238, (uint8_t)158, (uint8_t)245, (uint8_t)233, (uint8_t)55, (uint8_t)181, (uint8_t)123, (uint8_t)72, (uint8_t)37, (uint8_t)44, (uint8_t)169, (uint8_t)132, (uint8_t)56, (uint8_t)225, (uint8_t)31, (uint8_t)233, (uint8_t)141, (uint8_t)237, (uint8_t)220, (uint8_t)199, (uint8_t)89, (uint8_t)221, (uint8_t)189, (uint8_t)215, (uint8_t)169, (uint8_t)142, (uint8_t)232, (uint8_t)98, (uint8_t)92, (uint8_t)169, (uint8_t)36, (uint8_t)89, (uint8_t)45, (uint8_t)88, (uint8_t)215, (uint8_t)177, (uint8_t)196, (uint8_t)110, (uint8_t)104, (uint8_t)135, (uint8_t)156, (uint8_t)126, (uint8_t)214, (uint8_t)202, (uint8_t)180, (uint8_t)225, (uint8_t)79, (uint8_t)74, (uint8_t)226, (uint8_t)26, (uint8_t)57, (uint8_t)49, (uint8_t)218, (uint8_t)187, (uint8_t)13, (uint8_t)228, (uint8_t)225, (uint8_t)235, (uint8_t)67, (uint8_t)8, (uint8_t)46, (uint8_t)207, (uint8_t)160, (uint8_t)160, (uint8_t)165, (uint8_t)73, (uint8_t)102, (uint8_t)252, (uint8_t)63, (uint8_t)109, (uint8_t)113, (uint8_t)37, (uint8_t)240, (uint8_t)115, (uint8_t)96, (uint8_t)213, (uint8_t)25, (uint8_t)61, (uint8_t)40, (uint8_t)175, (uint8_t)121, (uint8_t)174, (uint8_t)252, (uint8_t)161, (uint8_t)5, (uint8_t)212, (uint8_t)229, (uint8_t)144, (uint8_t)221, (uint8_t)22, (uint8_t)190, (uint8_t)71, (uint8_t)248, (uint8_t)187, (uint8_t)105, (uint8_t)241, (uint8_t)148, (uint8_t)2, (uint8_t)203, (uint8_t)4, (uint8_t)138, (uint8_t)179, (uint8_t)207, (uint8_t)254, (uint8_t)216, (uint8_t)45, (uint8_t)200, (uint8_t)25, (uint8_t)178, (uint8_t)189, (uint8_t)104, (uint8_t)247, (uint8_t)95};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_heading_sp_SET((int16_t)(int16_t)9321, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)33, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)23249, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3697017404L, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED), PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -22, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p234_longitude_SET((int32_t) -873108756, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)14, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -4593, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)8399, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)1362, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -63, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)28946, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)26183, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p234_latitude_SET((int32_t)1754639526, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_x_SET((float)3.1024722E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)3769409311068546626L, PH.base.pack) ;
        p241_vibration_z_SET((float)1.099249E38F, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.9706087E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)702941688L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)119887554L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)459427748L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_approach_x_SET((float) -8.64221E37F, PH.base.pack) ;
        {
            float q[] =  {3.3899234E38F, -3.259716E38F, 2.4817163E38F, -2.796832E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_altitude_SET((int32_t)705034989, PH.base.pack) ;
        p242_approach_y_SET((float) -3.3164247E38F, PH.base.pack) ;
        p242_x_SET((float) -2.9063773E37F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.9838752E38F, PH.base.pack) ;
        p242_z_SET((float)1.5300556E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)778474367, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)7372663302914146179L, &PH) ;
        p242_y_SET((float) -1.2526044E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t)299157718, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p243_y_SET((float) -3.2648658E38F, PH.base.pack) ;
        p243_z_SET((float)1.6978365E38F, PH.base.pack) ;
        {
            float q[] =  {-7.8360466E37F, -2.5262719E38F, -5.6275797E37F, 1.4040673E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_longitude_SET((int32_t)388498236, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)8310788440890370371L, &PH) ;
        p243_x_SET((float)1.5702829E38F, PH.base.pack) ;
        p243_approach_x_SET((float)3.178479E38F, PH.base.pack) ;
        p243_approach_y_SET((float)2.3838679E38F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.1789309E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)2051162958, PH.base.pack) ;
        p243_latitude_SET((int32_t)912516051, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)46426, PH.base.pack) ;
        p244_interval_us_SET((int32_t)138174084, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)48472, PH.base.pack) ;
        {
            char16_t* callsign = u"t";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_tslc_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p246_lat_SET((int32_t)1588805790, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)45342, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HEAVY, PH.base.pack) ;
        p246_altitude_SET((int32_t)522981379, PH.base.pack) ;
        p246_lon_SET((int32_t) -1891183009, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED), PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)27569, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)20804, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2115454360L, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
        p247_id_SET((uint32_t)474681608L, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)1.8177063E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)2.8895371E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.1923115E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)104, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)244, (uint8_t)26, (uint8_t)34, (uint8_t)201, (uint8_t)247, (uint8_t)172, (uint8_t)182, (uint8_t)40, (uint8_t)152, (uint8_t)110, (uint8_t)202, (uint8_t)181, (uint8_t)68, (uint8_t)174, (uint8_t)179, (uint8_t)74, (uint8_t)13, (uint8_t)10, (uint8_t)224, (uint8_t)2, (uint8_t)133, (uint8_t)157, (uint8_t)41, (uint8_t)103, (uint8_t)188, (uint8_t)80, (uint8_t)62, (uint8_t)161, (uint8_t)184, (uint8_t)190, (uint8_t)98, (uint8_t)1, (uint8_t)29, (uint8_t)67, (uint8_t)20, (uint8_t)234, (uint8_t)66, (uint8_t)37, (uint8_t)24, (uint8_t)221, (uint8_t)52, (uint8_t)116, (uint8_t)84, (uint8_t)255, (uint8_t)175, (uint8_t)48, (uint8_t)249, (uint8_t)191, (uint8_t)161, (uint8_t)231, (uint8_t)43, (uint8_t)134, (uint8_t)130, (uint8_t)86, (uint8_t)24, (uint8_t)136, (uint8_t)149, (uint8_t)94, (uint8_t)242, (uint8_t)254, (uint8_t)204, (uint8_t)197, (uint8_t)160, (uint8_t)157, (uint8_t)14, (uint8_t)153, (uint8_t)20, (uint8_t)20, (uint8_t)206, (uint8_t)31, (uint8_t)15, (uint8_t)76, (uint8_t)65, (uint8_t)179, (uint8_t)43, (uint8_t)152, (uint8_t)139, (uint8_t)124, (uint8_t)255, (uint8_t)40, (uint8_t)232, (uint8_t)227, (uint8_t)210, (uint8_t)70, (uint8_t)151, (uint8_t)33, (uint8_t)84, (uint8_t)127, (uint8_t)225, (uint8_t)18, (uint8_t)157, (uint8_t)164, (uint8_t)161, (uint8_t)50, (uint8_t)75, (uint8_t)183, (uint8_t)154, (uint8_t)3, (uint8_t)55, (uint8_t)110, (uint8_t)255, (uint8_t)144, (uint8_t)13, (uint8_t)207, (uint8_t)143, (uint8_t)236, (uint8_t)10, (uint8_t)43, (uint8_t)130, (uint8_t)30, (uint8_t)134, (uint8_t)199, (uint8_t)100, (uint8_t)218, (uint8_t)169, (uint8_t)158, (uint8_t)255, (uint8_t)128, (uint8_t)240, (uint8_t)103, (uint8_t)122, (uint8_t)52, (uint8_t)220, (uint8_t)163, (uint8_t)59, (uint8_t)238, (uint8_t)39, (uint8_t)255, (uint8_t)132, (uint8_t)171, (uint8_t)36, (uint8_t)148, (uint8_t)112, (uint8_t)3, (uint8_t)42, (uint8_t)234, (uint8_t)126, (uint8_t)244, (uint8_t)218, (uint8_t)118, (uint8_t)35, (uint8_t)220, (uint8_t)164, (uint8_t)123, (uint8_t)111, (uint8_t)129, (uint8_t)107, (uint8_t)19, (uint8_t)207, (uint8_t)21, (uint8_t)204, (uint8_t)90, (uint8_t)170, (uint8_t)148, (uint8_t)104, (uint8_t)21, (uint8_t)63, (uint8_t)81, (uint8_t)217, (uint8_t)11, (uint8_t)43, (uint8_t)64, (uint8_t)7, (uint8_t)53, (uint8_t)37, (uint8_t)48, (uint8_t)138, (uint8_t)86, (uint8_t)186, (uint8_t)35, (uint8_t)71, (uint8_t)140, (uint8_t)136, (uint8_t)184, (uint8_t)243, (uint8_t)251, (uint8_t)223, (uint8_t)239, (uint8_t)143, (uint8_t)127, (uint8_t)81, (uint8_t)19, (uint8_t)8, (uint8_t)166, (uint8_t)241, (uint8_t)54, (uint8_t)210, (uint8_t)91, (uint8_t)234, (uint8_t)254, (uint8_t)182, (uint8_t)214, (uint8_t)29, (uint8_t)76, (uint8_t)33, (uint8_t)228, (uint8_t)203, (uint8_t)73, (uint8_t)10, (uint8_t)11, (uint8_t)76, (uint8_t)130, (uint8_t)247, (uint8_t)189, (uint8_t)149, (uint8_t)27, (uint8_t)88, (uint8_t)167, (uint8_t)91, (uint8_t)99, (uint8_t)111, (uint8_t)225, (uint8_t)135, (uint8_t)159, (uint8_t)202, (uint8_t)81, (uint8_t)155, (uint8_t)29, (uint8_t)84, (uint8_t)116, (uint8_t)74, (uint8_t)33, (uint8_t)161, (uint8_t)210, (uint8_t)138, (uint8_t)41, (uint8_t)145, (uint8_t)184, (uint8_t)193, (uint8_t)205, (uint8_t)21, (uint8_t)167, (uint8_t)85, (uint8_t)171, (uint8_t)80, (uint8_t)220, (uint8_t)20, (uint8_t)16, (uint8_t)45, (uint8_t)128, (uint8_t)136, (uint8_t)16, (uint8_t)69, (uint8_t)246, (uint8_t)158, (uint8_t)64, (uint8_t)10, (uint8_t)105, (uint8_t)40};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t)77, (int8_t) -35, (int8_t) -7, (int8_t) -25, (int8_t)68, (int8_t)46, (int8_t) -70, (int8_t)73, (int8_t)0, (int8_t) -14, (int8_t) -55, (int8_t)22, (int8_t)101, (int8_t)20, (int8_t) -99, (int8_t)120, (int8_t)127, (int8_t)16, (int8_t) -90, (int8_t)105, (int8_t) -27, (int8_t)72, (int8_t) -107, (int8_t) -30, (int8_t) -80, (int8_t)27, (int8_t)22, (int8_t)110, (int8_t) -59, (int8_t)15, (int8_t)123, (int8_t) -92};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)63344, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_z_SET((float)1.8167766E38F, PH.base.pack) ;
        p250_x_SET((float)3.350157E38F, PH.base.pack) ;
        p250_y_SET((float) -1.7337507E38F, PH.base.pack) ;
        {
            char16_t* name = u"grsaj";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)5725698315186161870L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -2.9234035E38F, PH.base.pack) ;
        {
            char16_t* name = u"bwjbutJ";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)70550626L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"eyhf";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t)1039354930, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)1947673926L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"Ukynubuq";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float) -2.661974E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)344260765L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)101, (uint8_t)40, (uint8_t)156, (uint8_t)250, (uint8_t)145, (uint8_t)15, (uint8_t)64, (uint8_t)55, (uint8_t)213, (uint8_t)233, (uint8_t)84, (uint8_t)125, (uint8_t)211, (uint8_t)65, (uint8_t)41, (uint8_t)15, (uint8_t)239, (uint8_t)25, (uint8_t)176, (uint8_t)199, (uint8_t)135, (uint8_t)180, (uint8_t)249, (uint8_t)85, (uint8_t)17, (uint8_t)16, (uint8_t)139, (uint8_t)28, (uint8_t)152, (uint8_t)6, (uint8_t)10, (uint8_t)191};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)743517577514623283L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)2559170119L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)3890706026L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"qgxjutiqtefbwjwlipkkvhwp";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        {
            uint8_t model_name[] =  {(uint8_t)168, (uint8_t)65, (uint8_t)88, (uint8_t)88, (uint8_t)152, (uint8_t)53, (uint8_t)12, (uint8_t)229, (uint8_t)159, (uint8_t)103, (uint8_t)45, (uint8_t)95, (uint8_t)30, (uint8_t)24, (uint8_t)129, (uint8_t)237, (uint8_t)237, (uint8_t)62, (uint8_t)186, (uint8_t)66, (uint8_t)18, (uint8_t)173, (uint8_t)10, (uint8_t)18, (uint8_t)128, (uint8_t)231, (uint8_t)31, (uint8_t)23, (uint8_t)223, (uint8_t)62, (uint8_t)245, (uint8_t)140};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_resolution_v_SET((uint16_t)(uint16_t)50809, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)2701732863L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)64948, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO), PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p259_focal_length_SET((float)2.6299772E37F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"vvIowdIpawqfhbclmmxgasrzpTlasyvpczsfrulfVuuyAgubQvwcyoarSdipknqjL";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_time_boot_ms_SET((uint32_t)3297609011L, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)236, (uint8_t)159, (uint8_t)71, (uint8_t)227, (uint8_t)148, (uint8_t)123, (uint8_t)171, (uint8_t)86, (uint8_t)131, (uint8_t)207, (uint8_t)154, (uint8_t)232, (uint8_t)25, (uint8_t)45, (uint8_t)78, (uint8_t)64, (uint8_t)226, (uint8_t)168, (uint8_t)121, (uint8_t)76, (uint8_t)185, (uint8_t)235, (uint8_t)12, (uint8_t)81, (uint8_t)95, (uint8_t)115, (uint8_t)26, (uint8_t)134, (uint8_t)206, (uint8_t)238, (uint8_t)129, (uint8_t)150};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)13777, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -1.2184333E38F, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -2.3851868E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)248810581L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_read_speed_SET((float) -3.1717738E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p261_total_capacity_SET((float) -5.0129304E37F, PH.base.pack) ;
        p261_used_capacity_SET((float) -2.5679662E38F, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.4660218E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1066333929L, PH.base.pack) ;
        p261_write_speed_SET((float) -7.47629E37F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_time_boot_ms_SET((uint32_t)2757859543L, PH.base.pack) ;
        p262_available_capacity_SET((float)1.9551464E38F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)1738790857L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p262_image_interval_SET((float)2.2104105E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)298530228091122358L, PH.base.pack) ;
        {
            float q[] =  {-1.6814327E38F, -2.6137006E38F, -3.3365856E38F, 3.400568E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_lon_SET((int32_t) -1504634033, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1599595911, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1238207462L, PH.base.pack) ;
        p263_alt_SET((int32_t) -2146254574, PH.base.pack) ;
        p263_lat_SET((int32_t)33617688, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -742125986, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -113, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        {
            char16_t* file_url = u"XqdmckUgxwqnjofXeajHsfVbddeuhjndcgqwflladhbsvdrde";
            p263_file_url_SET_(file_url, &PH) ;
        }
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)3240022778L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)614549206019726109L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)6534873161212456747L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)4096324315966967708L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)3007528026L, PH.base.pack) ;
        p265_pitch_SET((float) -1.3190496E38F, PH.base.pack) ;
        p265_yaw_SET((float)8.696323E37F, PH.base.pack) ;
        p265_roll_SET((float) -7.0604254E37F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_length_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)43, (uint8_t)251, (uint8_t)65, (uint8_t)157, (uint8_t)181, (uint8_t)21, (uint8_t)110, (uint8_t)48, (uint8_t)17, (uint8_t)80, (uint8_t)179, (uint8_t)175, (uint8_t)15, (uint8_t)118, (uint8_t)5, (uint8_t)162, (uint8_t)185, (uint8_t)33, (uint8_t)218, (uint8_t)77, (uint8_t)239, (uint8_t)191, (uint8_t)164, (uint8_t)128, (uint8_t)58, (uint8_t)188, (uint8_t)49, (uint8_t)167, (uint8_t)136, (uint8_t)41, (uint8_t)13, (uint8_t)104, (uint8_t)151, (uint8_t)200, (uint8_t)30, (uint8_t)164, (uint8_t)136, (uint8_t)94, (uint8_t)109, (uint8_t)44, (uint8_t)187, (uint8_t)202, (uint8_t)131, (uint8_t)237, (uint8_t)243, (uint8_t)239, (uint8_t)106, (uint8_t)26, (uint8_t)105, (uint8_t)2, (uint8_t)223, (uint8_t)194, (uint8_t)191, (uint8_t)17, (uint8_t)179, (uint8_t)105, (uint8_t)246, (uint8_t)31, (uint8_t)143, (uint8_t)185, (uint8_t)51, (uint8_t)65, (uint8_t)172, (uint8_t)161, (uint8_t)112, (uint8_t)169, (uint8_t)43, (uint8_t)235, (uint8_t)235, (uint8_t)232, (uint8_t)170, (uint8_t)138, (uint8_t)90, (uint8_t)85, (uint8_t)149, (uint8_t)24, (uint8_t)239, (uint8_t)196, (uint8_t)239, (uint8_t)59, (uint8_t)202, (uint8_t)6, (uint8_t)60, (uint8_t)171, (uint8_t)37, (uint8_t)58, (uint8_t)212, (uint8_t)129, (uint8_t)80, (uint8_t)42, (uint8_t)72, (uint8_t)91, (uint8_t)192, (uint8_t)142, (uint8_t)120, (uint8_t)169, (uint8_t)100, (uint8_t)60, (uint8_t)30, (uint8_t)184, (uint8_t)242, (uint8_t)170, (uint8_t)246, (uint8_t)133, (uint8_t)39, (uint8_t)121, (uint8_t)211, (uint8_t)89, (uint8_t)84, (uint8_t)129, (uint8_t)33, (uint8_t)173, (uint8_t)81, (uint8_t)245, (uint8_t)101, (uint8_t)102, (uint8_t)141, (uint8_t)196, (uint8_t)47, (uint8_t)218, (uint8_t)127, (uint8_t)96, (uint8_t)209, (uint8_t)70, (uint8_t)205, (uint8_t)111, (uint8_t)144, (uint8_t)21, (uint8_t)10, (uint8_t)85, (uint8_t)240, (uint8_t)19, (uint8_t)208, (uint8_t)96, (uint8_t)62, (uint8_t)125, (uint8_t)158, (uint8_t)141, (uint8_t)205, (uint8_t)115, (uint8_t)128, (uint8_t)52, (uint8_t)65, (uint8_t)20, (uint8_t)229, (uint8_t)148, (uint8_t)102, (uint8_t)78, (uint8_t)172, (uint8_t)58, (uint8_t)64, (uint8_t)249, (uint8_t)211, (uint8_t)14, (uint8_t)163, (uint8_t)120, (uint8_t)105, (uint8_t)80, (uint8_t)22, (uint8_t)208, (uint8_t)170, (uint8_t)126, (uint8_t)10, (uint8_t)43, (uint8_t)12, (uint8_t)18, (uint8_t)30, (uint8_t)164, (uint8_t)231, (uint8_t)253, (uint8_t)202, (uint8_t)95, (uint8_t)178, (uint8_t)154, (uint8_t)48, (uint8_t)242, (uint8_t)89, (uint8_t)23, (uint8_t)111, (uint8_t)15, (uint8_t)143, (uint8_t)26, (uint8_t)132, (uint8_t)134, (uint8_t)21, (uint8_t)52, (uint8_t)139, (uint8_t)60, (uint8_t)91, (uint8_t)221, (uint8_t)105, (uint8_t)196, (uint8_t)135, (uint8_t)253, (uint8_t)139, (uint8_t)209, (uint8_t)38, (uint8_t)213, (uint8_t)247, (uint8_t)47, (uint8_t)82, (uint8_t)172, (uint8_t)191, (uint8_t)123, (uint8_t)220, (uint8_t)36, (uint8_t)65, (uint8_t)136, (uint8_t)58, (uint8_t)187, (uint8_t)167, (uint8_t)69, (uint8_t)178, (uint8_t)37, (uint8_t)151, (uint8_t)140, (uint8_t)226, (uint8_t)44, (uint8_t)50, (uint8_t)72, (uint8_t)16, (uint8_t)154, (uint8_t)77, (uint8_t)188, (uint8_t)67, (uint8_t)2, (uint8_t)56, (uint8_t)126, (uint8_t)152, (uint8_t)241, (uint8_t)128, (uint8_t)55, (uint8_t)159, (uint8_t)139, (uint8_t)87, (uint8_t)14, (uint8_t)128, (uint8_t)151, (uint8_t)27, (uint8_t)7, (uint8_t)32, (uint8_t)36, (uint8_t)148, (uint8_t)29, (uint8_t)238, (uint8_t)214, (uint8_t)16, (uint8_t)40, (uint8_t)8};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)16087, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)20, (uint8_t)147, (uint8_t)48, (uint8_t)199, (uint8_t)45, (uint8_t)150, (uint8_t)13, (uint8_t)181, (uint8_t)204, (uint8_t)68, (uint8_t)95, (uint8_t)69, (uint8_t)84, (uint8_t)122, (uint8_t)237, (uint8_t)89, (uint8_t)192, (uint8_t)32, (uint8_t)180, (uint8_t)72, (uint8_t)196, (uint8_t)227, (uint8_t)142, (uint8_t)178, (uint8_t)41, (uint8_t)233, (uint8_t)1, (uint8_t)134, (uint8_t)157, (uint8_t)157, (uint8_t)43, (uint8_t)175, (uint8_t)238, (uint8_t)24, (uint8_t)32, (uint8_t)127, (uint8_t)0, (uint8_t)121, (uint8_t)107, (uint8_t)70, (uint8_t)86, (uint8_t)147, (uint8_t)126, (uint8_t)141, (uint8_t)249, (uint8_t)9, (uint8_t)169, (uint8_t)14, (uint8_t)89, (uint8_t)241, (uint8_t)0, (uint8_t)187, (uint8_t)32, (uint8_t)232, (uint8_t)235, (uint8_t)244, (uint8_t)117, (uint8_t)106, (uint8_t)233, (uint8_t)227, (uint8_t)204, (uint8_t)120, (uint8_t)158, (uint8_t)16, (uint8_t)0, (uint8_t)203, (uint8_t)149, (uint8_t)52, (uint8_t)15, (uint8_t)115, (uint8_t)42, (uint8_t)68, (uint8_t)86, (uint8_t)193, (uint8_t)169, (uint8_t)209, (uint8_t)138, (uint8_t)8, (uint8_t)82, (uint8_t)190, (uint8_t)193, (uint8_t)202, (uint8_t)38, (uint8_t)181, (uint8_t)21, (uint8_t)235, (uint8_t)239, (uint8_t)101, (uint8_t)72, (uint8_t)167, (uint8_t)152, (uint8_t)92, (uint8_t)182, (uint8_t)71, (uint8_t)251, (uint8_t)59, (uint8_t)142, (uint8_t)215, (uint8_t)97, (uint8_t)101, (uint8_t)237, (uint8_t)113, (uint8_t)132, (uint8_t)138, (uint8_t)42, (uint8_t)118, (uint8_t)92, (uint8_t)106, (uint8_t)125, (uint8_t)245, (uint8_t)228, (uint8_t)254, (uint8_t)118, (uint8_t)238, (uint8_t)97, (uint8_t)50, (uint8_t)51, (uint8_t)243, (uint8_t)124, (uint8_t)72, (uint8_t)83, (uint8_t)26, (uint8_t)127, (uint8_t)162, (uint8_t)6, (uint8_t)227, (uint8_t)187, (uint8_t)134, (uint8_t)73, (uint8_t)84, (uint8_t)97, (uint8_t)173, (uint8_t)145, (uint8_t)101, (uint8_t)110, (uint8_t)204, (uint8_t)209, (uint8_t)123, (uint8_t)207, (uint8_t)110, (uint8_t)131, (uint8_t)11, (uint8_t)49, (uint8_t)192, (uint8_t)158, (uint8_t)51, (uint8_t)130, (uint8_t)65, (uint8_t)148, (uint8_t)172, (uint8_t)86, (uint8_t)84, (uint8_t)211, (uint8_t)7, (uint8_t)36, (uint8_t)123, (uint8_t)13, (uint8_t)66, (uint8_t)231, (uint8_t)87, (uint8_t)202, (uint8_t)129, (uint8_t)79, (uint8_t)225, (uint8_t)235, (uint8_t)55, (uint8_t)23, (uint8_t)87, (uint8_t)0, (uint8_t)255, (uint8_t)51, (uint8_t)238, (uint8_t)234, (uint8_t)209, (uint8_t)81, (uint8_t)130, (uint8_t)7, (uint8_t)165, (uint8_t)161, (uint8_t)154, (uint8_t)9, (uint8_t)155, (uint8_t)255, (uint8_t)90, (uint8_t)3, (uint8_t)9, (uint8_t)113, (uint8_t)240, (uint8_t)104, (uint8_t)56, (uint8_t)56, (uint8_t)245, (uint8_t)43, (uint8_t)224, (uint8_t)228, (uint8_t)154, (uint8_t)143, (uint8_t)61, (uint8_t)65, (uint8_t)114, (uint8_t)50, (uint8_t)131, (uint8_t)60, (uint8_t)31, (uint8_t)99, (uint8_t)47, (uint8_t)127, (uint8_t)164, (uint8_t)161, (uint8_t)190, (uint8_t)60, (uint8_t)11, (uint8_t)54, (uint8_t)8, (uint8_t)209, (uint8_t)217, (uint8_t)103, (uint8_t)190, (uint8_t)126, (uint8_t)64, (uint8_t)72, (uint8_t)174, (uint8_t)214, (uint8_t)98, (uint8_t)205, (uint8_t)32, (uint8_t)131, (uint8_t)112, (uint8_t)6, (uint8_t)226, (uint8_t)126, (uint8_t)243, (uint8_t)113, (uint8_t)255, (uint8_t)208, (uint8_t)25, (uint8_t)47, (uint8_t)162, (uint8_t)92, (uint8_t)136, (uint8_t)107, (uint8_t)174, (uint8_t)199, (uint8_t)81, (uint8_t)21, (uint8_t)121, (uint8_t)229, (uint8_t)172, (uint8_t)248};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)42905, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)64211, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_framerate_SET((float) -1.7293393E38F, PH.base.pack) ;
        {
            char16_t* uri = u"f";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_rotation_SET((uint16_t)(uint16_t)61539, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)46563, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)155490002L, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)60673, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_framerate_SET((float)9.149451E37F, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)23021, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)61282, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2379547223L, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)46988, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        {
            char16_t* uri = u"pfkvrzjiyrcrvbvMvydvsfqewstqbysxoljtmotiputrgjhFwCaRvsHjwcjxtxKnuzpxtyfpfcwxLmiunulgxtvvetsuerppltcatmvdgDkjjlxlyxigUudisijonmvzjmpzmQfvvtczvtovcgbvsbxdxbgsddeyrufkmri";
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
            char16_t* password = u"gmdweyfkpeGpiptcvxynwozQGmbcZFobjpszhgxuqVuirfzynfbGhEbgiwjwi";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"sv";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        {
            uint8_t spec_version_hash[] =  {(uint8_t)154, (uint8_t)246, (uint8_t)123, (uint8_t)195, (uint8_t)249, (uint8_t)29, (uint8_t)175, (uint8_t)9};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)16209, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)6413, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)60928, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)237, (uint8_t)131, (uint8_t)44, (uint8_t)149, (uint8_t)211, (uint8_t)229, (uint8_t)103, (uint8_t)98};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_uptime_sec_SET((uint32_t)16183552L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)34785, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)2136164674491232892L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_major_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3557761435L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)171, (uint8_t)65, (uint8_t)114, (uint8_t)22, (uint8_t)139, (uint8_t)153, (uint8_t)177, (uint8_t)12, (uint8_t)165, (uint8_t)76, (uint8_t)163, (uint8_t)36, (uint8_t)138, (uint8_t)118, (uint8_t)19, (uint8_t)222};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        {
            char16_t* name = u"mlvzjwtkpmqfohdzpuzcrzfngczgwdqizqsyagqrDPzqptzpDjsn";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_version_minor_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1916334047L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)5653438397623360828L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"xJnkyysxes";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t) -31542, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_count_SET((uint16_t)(uint16_t)2938, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)20897, PH.base.pack) ;
        {
            char16_t* param_value = u"dhChpmjkhosrsgiyecvpqtyprzjxHgkacaicpjaivcfNuusjukrDupnIqcuxzPhFHlrrvljhsczxslizmpdlojAgglnkudXxexcMecdckaim";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        {
            char16_t* param_id = u"cDuwep";
            p322_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        {
            char16_t* param_value = u"ujhbsiywzralibizslcfYapkprMvpxerKbiOopgyvaoWlsNpfcyfotylhbjgLzzmtuckejoqCjwThxun";
            p323_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"jqIteP";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"ryafWuvzcigzaci";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"ryv";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        {
            uint16_t distances[] =  {(uint16_t)30840, (uint16_t)61420, (uint16_t)20182, (uint16_t)47933, (uint16_t)41209, (uint16_t)55033, (uint16_t)28129, (uint16_t)40140, (uint16_t)6174, (uint16_t)35359, (uint16_t)47773, (uint16_t)41575, (uint16_t)23591, (uint16_t)36350, (uint16_t)63568, (uint16_t)60360, (uint16_t)33749, (uint16_t)33290, (uint16_t)32582, (uint16_t)60403, (uint16_t)39782, (uint16_t)16921, (uint16_t)9244, (uint16_t)7052, (uint16_t)25500, (uint16_t)50654, (uint16_t)13219, (uint16_t)11382, (uint16_t)35880, (uint16_t)63073, (uint16_t)14502, (uint16_t)19688, (uint16_t)40483, (uint16_t)42892, (uint16_t)41228, (uint16_t)47975, (uint16_t)23123, (uint16_t)5441, (uint16_t)64967, (uint16_t)51360, (uint16_t)45223, (uint16_t)154, (uint16_t)2248, (uint16_t)28093, (uint16_t)31446, (uint16_t)12279, (uint16_t)26998, (uint16_t)54199, (uint16_t)1979, (uint16_t)35699, (uint16_t)44497, (uint16_t)62363, (uint16_t)63198, (uint16_t)52726, (uint16_t)21599, (uint16_t)63941, (uint16_t)38000, (uint16_t)14633, (uint16_t)51267, (uint16_t)45970, (uint16_t)15274, (uint16_t)30165, (uint16_t)10493, (uint16_t)15494, (uint16_t)59671, (uint16_t)43969, (uint16_t)59577, (uint16_t)2354, (uint16_t)34572, (uint16_t)35964, (uint16_t)3206, (uint16_t)41918};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)8928771683732725207L, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)40218, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)44090, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


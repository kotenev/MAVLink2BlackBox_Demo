
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
    return  _en__P(get_bits(data, 40, 4));
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
            return e_MAV_CMD_MAV_CMD_DO_NOTHING;
        case 111:
            return e_MAV_CMD_MAV_CMD_RETURN_TO_BASE;
        case 112:
            return e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE;
        case 113:
            return e_MAV_CMD_MAV_CMD_TURN_LIGHT;
        case 114:
            return e_MAV_CMD_MAV_CMD_GET_MID_LEVEL_COMMANDS;
        case 115:
            return e_MAV_CMD_MAV_CMD_MIDLEVEL_STORAGE;
        case 116:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 117:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 130:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 131:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 132:
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
            return e_MAV_CMD_MAV_CMD_DO_NOTHING;
        case 111:
            return e_MAV_CMD_MAV_CMD_RETURN_TO_BASE;
        case 112:
            return e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE;
        case 113:
            return e_MAV_CMD_MAV_CMD_TURN_LIGHT;
        case 114:
            return e_MAV_CMD_MAV_CMD_GET_MID_LEVEL_COMMANDS;
        case 115:
            return e_MAV_CMD_MAV_CMD_MIDLEVEL_STORAGE;
        case 116:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 117:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 130:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 131:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 132:
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
            return e_MAV_CMD_MAV_CMD_DO_NOTHING;
        case 111:
            return e_MAV_CMD_MAV_CMD_RETURN_TO_BASE;
        case 112:
            return e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE;
        case 113:
            return e_MAV_CMD_MAV_CMD_TURN_LIGHT;
        case 114:
            return e_MAV_CMD_MAV_CMD_GET_MID_LEVEL_COMMANDS;
        case 115:
            return e_MAV_CMD_MAV_CMD_MIDLEVEL_STORAGE;
        case 116:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 117:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 130:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 131:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 132:
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
            return e_MAV_CMD_MAV_CMD_DO_NOTHING;
        case 111:
            return e_MAV_CMD_MAV_CMD_RETURN_TO_BASE;
        case 112:
            return e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE;
        case 113:
            return e_MAV_CMD_MAV_CMD_TURN_LIGHT;
        case 114:
            return e_MAV_CMD_MAV_CMD_GET_MID_LEVEL_COMMANDS;
        case 115:
            return e_MAV_CMD_MAV_CMD_MIDLEVEL_STORAGE;
        case 116:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 117:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 130:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 131:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 132:
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
            return e_MAV_CMD_MAV_CMD_DO_NOTHING;
        case 111:
            return e_MAV_CMD_MAV_CMD_RETURN_TO_BASE;
        case 112:
            return e_MAV_CMD_MAV_CMD_STOP_RETURN_TO_BASE;
        case 113:
            return e_MAV_CMD_MAV_CMD_TURN_LIGHT;
        case 114:
            return e_MAV_CMD_MAV_CMD_GET_MID_LEVEL_COMMANDS;
        case 115:
            return e_MAV_CMD_MAV_CMD_MIDLEVEL_STORAGE;
        case 116:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY;
        case 117:
            return e_MAV_CMD_MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
        case 118:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_1;
        case 119:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2;
        case 120:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_3;
        case 121:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4;
        case 122:
            return e_MAV_CMD_MAV_CMD_WAYPOINT_USER_5;
        case 123:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_1;
        case 124:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_2;
        case 125:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_3;
        case 126:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_4;
        case 127:
            return e_MAV_CMD_MAV_CMD_SPATIAL_USER_5;
        case 128:
            return e_MAV_CMD_MAV_CMD_USER_1;
        case 129:
            return e_MAV_CMD_MAV_CMD_USER_2;
        case 130:
            return e_MAV_CMD_MAV_CMD_USER_3;
        case 131:
            return e_MAV_CMD_MAV_CMD_USER_4;
        case 132:
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
INLINER void p170_batVolt_SET(uint16_t  src, Pack * dst)//Battery Voltage in millivolts
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p170_sensLoad_SET(uint8_t  src, Pack * dst)//Sensor DSC Load
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p170_ctrlLoad_SET(uint8_t  src, Pack * dst)//Control DSC Load
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
Pack * c_TEST_Channel_new_CPU_LOAD_170()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 170));
};
INLINER void p172_axBias_SET(float  src, Pack * dst)//Accelerometer X bias (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p172_ayBias_SET(float  src, Pack * dst)//Accelerometer Y bias (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p172_azBias_SET(float  src, Pack * dst)//Accelerometer Z bias (m/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p172_gxBias_SET(float  src, Pack * dst)//Gyro X bias (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p172_gyBias_SET(float  src, Pack * dst)//Gyro Y bias (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p172_gzBias_SET(float  src, Pack * dst)//Gyro Z bias (rad/s)
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
Pack * c_TEST_Channel_new_SENSOR_BIAS_172()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 172));
};
INLINER void p173_diagFl1_SET(float  src, Pack * dst)//Diagnostic float 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p173_diagFl2_SET(float  src, Pack * dst)//Diagnostic float 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p173_diagFl3_SET(float  src, Pack * dst)//Diagnostic float 3
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p173_diagSh1_SET(int16_t  src, Pack * dst)//Diagnostic short 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  12);
}
INLINER void p173_diagSh2_SET(int16_t  src, Pack * dst)//Diagnostic short 2
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  14);
}
INLINER void p173_diagSh3_SET(int16_t  src, Pack * dst)//Diagnostic short 3
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  16);
}
Pack * c_TEST_Channel_new_DIAGNOSTIC_173()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 173));
};
INLINER void p176_h_c_SET(uint16_t  src, Pack * dst)//Commanded altitude in 0.1 m
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p176_u_m_SET(float  src, Pack * dst)//Measured Airspeed prior to the nav filter in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p176_phi_c_SET(float  src, Pack * dst)//Commanded Roll
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
INLINER void p176_theta_c_SET(float  src, Pack * dst)//Commanded Pitch
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
INLINER void p176_psiDot_c_SET(float  src, Pack * dst)//Commanded Turn rate
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  14);
}
INLINER void p176_ay_body_SET(float  src, Pack * dst)//Y component of the body acceleration
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  18);
}
INLINER void p176_totalDist_SET(float  src, Pack * dst)//Total Distance to Run on this leg of Navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  22);
}
INLINER void p176_dist2Go_SET(float  src, Pack * dst)//Remaining distance to Run on this leg of Navigation
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  26);
}
INLINER void p176_fromWP_SET(uint8_t  src, Pack * dst)//Origin WP
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  30);
}
INLINER void p176_toWP_SET(uint8_t  src, Pack * dst)//Destination WP
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  31);
}
Pack * c_TEST_Channel_new_SLUGS_NAVIGATION_176()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 176));
};
INLINER void p177_fl_1_SET(float  src, Pack * dst)//Log value 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p177_fl_2_SET(float  src, Pack * dst)//Log value 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p177_fl_3_SET(float  src, Pack * dst)//Log value 3
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  8);
}
INLINER void p177_fl_4_SET(float  src, Pack * dst)//Log value 4
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  12);
}
INLINER void p177_fl_5_SET(float  src, Pack * dst)//Log value 5
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  16);
}
INLINER void p177_fl_6_SET(float  src, Pack * dst)//Log value 6
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  20);
}
Pack * c_TEST_Channel_new_DATA_LOG_177()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 177));
};
INLINER void p179_year_SET(uint8_t  src, Pack * dst)//Year reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p179_month_SET(uint8_t  src, Pack * dst)//Month reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p179_day_SET(uint8_t  src, Pack * dst)//Day reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p179_hour_SET(uint8_t  src, Pack * dst)//Hour reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p179_min_SET(uint8_t  src, Pack * dst)//Min reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p179_sec_SET(uint8_t  src, Pack * dst)//Sec reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  5);
}
INLINER void p179_clockStat_SET(uint8_t  src, Pack * dst)//Clock Status. See table 47 page 211 OEMStar Manual
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p179_visSat_SET(uint8_t  src, Pack * dst)//Visible satellites reported by Gps
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p179_useSat_SET(uint8_t  src, Pack * dst)//Used satellites in Solution
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p179_GppGl_SET(uint8_t  src, Pack * dst)//GPS+GLONASS satellites in Solution
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p179_sigUsedMask_SET(uint8_t  src, Pack * dst)//GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
INLINER void p179_percentUsed_SET(uint8_t  src, Pack * dst)//Percent used GPS
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  11);
}
Pack * c_TEST_Channel_new_GPS_DATE_TIME_179()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 179));
};
INLINER void p180_target_SET(uint8_t  src, Pack * dst)//The system setting the commands
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p180_hCommand_SET(float  src, Pack * dst)//Commanded Altitude in meters
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  1);
}
INLINER void p180_uCommand_SET(float  src, Pack * dst)//Commanded Airspeed in m/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER void p180_rCommand_SET(float  src, Pack * dst)//Commanded Turnrate in rad/s
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
Pack * c_TEST_Channel_new_MID_LVL_CMDS_180()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 180));
};
INLINER void p181_bitfieldPt_SET(uint16_t  src, Pack * dst)//Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p181_target_SET(uint8_t  src, Pack * dst)//The system setting the commands
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_CTRL_SRFC_PT_181()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 181));
};
INLINER void p184_target_SET(uint8_t  src, Pack * dst)//The system reporting the action
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p184_pan_SET(int8_t  src, Pack * dst)//Order the mount to pan: -1 left, 0 No pan motion, +1 right
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  1);
}
INLINER void p184_tilt_SET(int8_t  src, Pack * dst)//Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  2);
}
INLINER void p184_zoom_SET(int8_t  src, Pack * dst)//Order the zoom values 0 to 10
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  3);
}
/**
*Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home,
*	0 ignore*/
INLINER void p184_moveHome_SET(int8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  4);
}
Pack * c_TEST_Channel_new_SLUGS_CAMERA_ORDER_184()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 184));
};
INLINER void p185_target_SET(uint8_t  src, Pack * dst)//The system setting the commands
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p185_idSurface_SET(uint8_t  src, Pack * dst)//ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p185_mControl_SET(float  src, Pack * dst)//Pending
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  2);
}
INLINER void p185_bControl_SET(float  src, Pack * dst)//Order to origin
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  6);
}
Pack * c_TEST_Channel_new_CONTROL_SURFACE_185()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 185));
};
INLINER void p186_target_SET(uint8_t  src, Pack * dst)//The system reporting the action
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p186_latitude_SET(float  src, Pack * dst)//Mobile Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  1);
}
INLINER void p186_longitude_SET(float  src, Pack * dst)//Mobile Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
Pack * c_TEST_Channel_new_SLUGS_MOBILE_LOCATION_186()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 186));
};
INLINER void p188_target_SET(uint8_t  src, Pack * dst)//The system setting the commands
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p188_idOrder_SET(uint8_t  src, Pack * dst)//ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  1);
}
INLINER void p188_order_SET(uint8_t  src, Pack * dst)//1: up/on 2: down/off 3: auto/reset/no action
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
Pack * c_TEST_Channel_new_SLUGS_CONFIGURATION_CAMERA_188()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 188));
};
INLINER void p189_target_SET(uint8_t  src, Pack * dst)//The system reporting the action
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p189_latitude_SET(float  src, Pack * dst)//ISR Latitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  1);
}
INLINER void p189_longitude_SET(float  src, Pack * dst)//ISR Longitude
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER void p189_height_SET(float  src, Pack * dst)//ISR Height
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER void p189_option1_SET(uint8_t  src, Pack * dst)//Option 1
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  13);
}
INLINER void p189_option2_SET(uint8_t  src, Pack * dst)//Option 2
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  14);
}
INLINER void p189_option3_SET(uint8_t  src, Pack * dst)//Option 3
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  15);
}
Pack * c_TEST_Channel_new_ISR_LOCATION_189()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 189));
};
INLINER void p191_voltage_SET(uint16_t  src, Pack * dst)//Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm
*	(2) Distance in cm (3) Absolute valu*/
INLINER void p191_reading2_SET(uint16_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  2);
}
INLINER void p191_r2Type_SET(uint8_t  src, Pack * dst)//It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
Pack * c_TEST_Channel_new_VOLT_SENSOR_191()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 191));
};
INLINER void p192_zoom_SET(uint8_t  src, Pack * dst)//The actual Zoom Value
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p192_pan_SET(int16_t  src, Pack * dst)//The Pan value in 10ths of degree
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  1);
}
INLINER void p192_tilt_SET(int16_t  src, Pack * dst)//The Tilt value in 10ths of degree
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  3);
}
Pack * c_TEST_Channel_new_PTZ_STATUS_192()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 192));
};
INLINER void p193_target_SET(uint8_t  src, Pack * dst)//The ID system reporting the action
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  0);
}
INLINER void p193_latitude_SET(float  src, Pack * dst)//Latitude UAV
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  1);
}
INLINER void p193_longitude_SET(float  src, Pack * dst)//Longitude UAV
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
INLINER void p193_altitude_SET(float  src, Pack * dst)//Altitude UAV
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  9);
}
INLINER void p193_speed_SET(float  src, Pack * dst)//Speed UAV
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  13);
}
INLINER void p193_course_SET(float  src, Pack * dst)//Course UAV
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  17);
}
Pack * c_TEST_Channel_new_UAV_STATUS_193()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 193));
};
INLINER void p194_csFails_SET(uint16_t  src, Pack * dst)//Number of times checksum has failed
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
/**
*The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning
*	mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
INLINER void p194_gpsQuality_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  2);
}
INLINER void p194_msgsType_SET(uint8_t  src, Pack * dst)//Indicates if GN, GL or GP messages are being received
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  3);
}
INLINER void p194_posStatus_SET(uint8_t  src, Pack * dst)//A = data valid, V = data invalid
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  4);
}
INLINER void p194_magVar_SET(float  src, Pack * dst)//Magnetic variation, degrees
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  5);
}
/**
*Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation
*	(W) adds to True cours*/
INLINER void p194_magDir_SET(int8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  9);
}
/**
*Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
*	input; N-Data not vali*/
INLINER void p194_modeInd_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
Pack * c_TEST_Channel_new_STATUS_GPS_194()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 194));
};
INLINER void p195_csFails_SET(uint16_t  src, Pack * dst)//Times the CRC has failed since boot
{
    uint8_t * data = dst->data;
    set_bytes((src), 2, data,  0);
}
INLINER void p195_receiverStatus_SET(uint32_t  src, Pack * dst)//Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  2);
}
INLINER void p195_timeStatus_SET(uint8_t  src, Pack * dst)//The Time Status. See Table 8 page 27 Novatel OEMStar Manual
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  6);
}
INLINER void p195_solStatus_SET(uint8_t  src, Pack * dst)//solution Status. See table 44 page 197
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  7);
}
INLINER void p195_posType_SET(uint8_t  src, Pack * dst)//position type. See table 43 page 196
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p195_velType_SET(uint8_t  src, Pack * dst)//velocity type. See table 43 page 196
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p195_posSolAge_SET(float  src, Pack * dst)//Age of the position solution in seconds
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  10);
}
Pack * c_TEST_Channel_new_NOVATEL_DIAG_195()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 195));
};
INLINER void p196_float1_SET(float  src, Pack * dst)//Float field 1
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  0);
}
INLINER void p196_float2_SET(float  src, Pack * dst)//Float field 2
{
    uint8_t * data = dst->data;
    set_bytes(floatToIntBits(src), 4, data,  4);
}
INLINER void p196_int1_SET(int16_t  src, Pack * dst)//Int 16 field 1
{
    uint8_t * data = dst->data;
    set_bytes((uint16_t)(src), 2, data,  8);
}
INLINER void p196_char1_SET(int8_t  src, Pack * dst)//Int 8 field 1
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  10);
}
Pack * c_TEST_Channel_new_SENSOR_DIAG_196()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 196));
};
INLINER void p197_version_SET(uint32_t  src, Pack * dst)//The onboard software version
{
    uint8_t * data = dst->data;
    set_bytes((src), 4, data,  0);
}
Pack * c_TEST_Channel_new_BOOT_197()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 197));
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARMAZILA);
    assert(p0_custom_mode_GET(pack) == (uint32_t)3719103892L);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED));
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_GENERIC);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)42948);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)49779);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)41157);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)34508);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)28907);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)10537);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)57778);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -11320);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -76);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)36982);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2387949876L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3725336867496810487L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vx_GET(pack) == (float)8.391138E37F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)11983);
    assert(p3_y_GET(pack) == (float) -2.307896E37F);
    assert(p3_afz_GET(pack) == (float)1.3899332E37F);
    assert(p3_z_GET(pack) == (float) -1.6902929E38F);
    assert(p3_vz_GET(pack) == (float) -2.9239118E38F);
    assert(p3_vy_GET(pack) == (float) -3.0020945E36F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3689037570L);
    assert(p3_afy_GET(pack) == (float)2.9061957E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -8.210629E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p3_x_GET(pack) == (float)3.838724E37F);
    assert(p3_afx_GET(pack) == (float) -2.0581583E38F);
    assert(p3_yaw_GET(pack) == (float) -3.7749268E37F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)4033634906L);
    assert(p4_time_usec_GET(pack) == (uint64_t)4472129752107593931L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)228);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p5_passkey_LEN(ph) == 1);
    {
        char16_t * exemplary = u"k";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)96);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)162);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 3);
    {
        char16_t * exemplary = u"Akj";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)358387682L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p20_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"ddCwhyrjgm";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -24818);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)161);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float)1.0396985E38F);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"i";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)35125);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)36718);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"yLniIwdh";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p23_param_value_GET(pack) == (float) -6.8265094E37F);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)220772083);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)50401);
    assert(p24_lat_GET(pack) == (int32_t) -579352706);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)33701);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)498376605L);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)15267);
    assert(p24_h_acc_TRY(ph) == (uint32_t)2315055921L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)3091);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)4148086756L);
    assert(p24_lon_GET(pack) == (int32_t)681530387);
    assert(p24_alt_GET(pack) == (int32_t) -488137837);
    assert(p24_time_usec_GET(pack) == (uint64_t)2833712201556788843L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1352751066L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)250, (uint8_t)176, (uint8_t)192, (uint8_t)113, (uint8_t)104, (uint8_t)19, (uint8_t)75, (uint8_t)54, (uint8_t)118, (uint8_t)88, (uint8_t)132, (uint8_t)2, (uint8_t)190, (uint8_t)65, (uint8_t)100, (uint8_t)53, (uint8_t)20, (uint8_t)49, (uint8_t)145, (uint8_t)105} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)9, (uint8_t)102, (uint8_t)140, (uint8_t)182, (uint8_t)203, (uint8_t)71, (uint8_t)143, (uint8_t)91, (uint8_t)54, (uint8_t)95, (uint8_t)126, (uint8_t)237, (uint8_t)195, (uint8_t)73, (uint8_t)54, (uint8_t)128, (uint8_t)136, (uint8_t)120, (uint8_t)235, (uint8_t)221} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)63, (uint8_t)14, (uint8_t)13, (uint8_t)172, (uint8_t)237, (uint8_t)133, (uint8_t)19, (uint8_t)217, (uint8_t)102, (uint8_t)108, (uint8_t)20, (uint8_t)143, (uint8_t)117, (uint8_t)195, (uint8_t)103, (uint8_t)202, (uint8_t)194, (uint8_t)236, (uint8_t)243, (uint8_t)76} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)177);
    {
        uint8_t exemplary[] =  {(uint8_t)133, (uint8_t)40, (uint8_t)72, (uint8_t)50, (uint8_t)246, (uint8_t)125, (uint8_t)27, (uint8_t)30, (uint8_t)249, (uint8_t)52, (uint8_t)171, (uint8_t)17, (uint8_t)167, (uint8_t)159, (uint8_t)190, (uint8_t)158, (uint8_t)49, (uint8_t)139, (uint8_t)227, (uint8_t)84} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)87, (uint8_t)228, (uint8_t)54, (uint8_t)121, (uint8_t)60, (uint8_t)137, (uint8_t)226, (uint8_t)203, (uint8_t)150, (uint8_t)162, (uint8_t)12, (uint8_t)106, (uint8_t)113, (uint8_t)214, (uint8_t)78, (uint8_t)70, (uint8_t)47, (uint8_t)192, (uint8_t)177, (uint8_t)96} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -29305);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -30427);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)28488);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2891374336L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -4400);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -892);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)24115);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -9626);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)24669);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)21646);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)22989);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -15397);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -32713);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)32159);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -24);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)17429);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)8693);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)32020);
    assert(p27_time_usec_GET(pack) == (uint64_t)1588602573255574940L);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)18372);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)13230);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)6828);
    assert(p28_time_usec_GET(pack) == (uint64_t)4029815571497298959L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -31053);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -3838);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float)7.254957E37F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)24685);
    assert(p29_press_diff_GET(pack) == (float)1.2974039E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1079804440L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_roll_GET(pack) == (float) -2.4040507E38F);
    assert(p30_pitch_GET(pack) == (float) -2.2385716E38F);
    assert(p30_yawspeed_GET(pack) == (float)1.6061913E38F);
    assert(p30_rollspeed_GET(pack) == (float) -3.1649244E38F);
    assert(p30_pitchspeed_GET(pack) == (float)9.53682E37F);
    assert(p30_yaw_GET(pack) == (float)1.3333056E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2531410322L);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q4_GET(pack) == (float)4.566466E37F);
    assert(p31_yawspeed_GET(pack) == (float)2.0395575E38F);
    assert(p31_pitchspeed_GET(pack) == (float)2.334941E38F);
    assert(p31_q1_GET(pack) == (float)1.7044007E38F);
    assert(p31_q3_GET(pack) == (float) -7.1545556E37F);
    assert(p31_rollspeed_GET(pack) == (float)3.2549775E38F);
    assert(p31_q2_GET(pack) == (float)2.2624026E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)172059461L);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float) -3.284545E38F);
    assert(p32_vy_GET(pack) == (float)1.9499086E37F);
    assert(p32_y_GET(pack) == (float)7.033853E37F);
    assert(p32_x_GET(pack) == (float)1.0028484E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1011204102L);
    assert(p32_vx_GET(pack) == (float)8.553617E37F);
    assert(p32_vz_GET(pack) == (float) -1.1325224E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lat_GET(pack) == (int32_t)1203126958);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)2630871604L);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -2068);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)30838);
    assert(p33_relative_alt_GET(pack) == (int32_t) -2144769742);
    assert(p33_lon_GET(pack) == (int32_t) -2128025485);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)30398);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)51643);
    assert(p33_alt_GET(pack) == (int32_t)2019801833);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -26798);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -30990);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)23393);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2140927487L);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)19595);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -32004);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)32230);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -8691);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)16958);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)51537437L);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)6649);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)52725);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)56032);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)59099);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)55152);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)2575);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)31225);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)11384);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)61653);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)17379);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)36152);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)25568);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)41624);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)18696);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)60512);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)36930);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)21713);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)57212);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)28839);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)28149);
    assert(p36_time_usec_GET(pack) == (uint32_t)4160231113L);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)4112);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)60390);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)41218);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)36381);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -8021);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)6583);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)95);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -15546);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)5378);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)138);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p39_z_GET(pack) == (float) -2.6798888E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)20358);
    assert(p39_y_GET(pack) == (float)1.9739407E38F);
    assert(p39_x_GET(pack) == (float)2.4497228E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p39_param3_GET(pack) == (float) -2.7255285E38F);
    assert(p39_param2_GET(pack) == (float)1.5383006E37F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p39_param1_GET(pack) == (float) -2.8534981E38F);
    assert(p39_param4_GET(pack) == (float) -1.2890089E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)18441);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)24132);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)66);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)60427);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)43783);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)50966);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_time_usec_TRY(ph) == (uint64_t)8146539498836834588L);
    assert(p48_altitude_GET(pack) == (int32_t)1734322675);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p48_latitude_GET(pack) == (int32_t) -593619954);
    assert(p48_longitude_GET(pack) == (int32_t)421703150);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_altitude_GET(pack) == (int32_t)1517581668);
    assert(p49_longitude_GET(pack) == (int32_t)289700153);
    assert(p49_latitude_GET(pack) == (int32_t) -914710276);
    assert(p49_time_usec_TRY(ph) == (uint64_t)4333303715668388355L);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value_min_GET(pack) == (float)1.5954541E38F);
    assert(p50_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"ckfomkctv";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p50_param_value_max_GET(pack) == (float) -4.984017E37F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)18914);
    assert(p50_param_value0_GET(pack) == (float) -3.3265752E38F);
    assert(p50_scale_GET(pack) == (float)1.7068123E37F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)182);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)28167);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2y_GET(pack) == (float) -2.7555045E38F);
    assert(p54_p2x_GET(pack) == (float)1.2715543E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p54_p1z_GET(pack) == (float) -7.8805847E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p54_p1x_GET(pack) == (float) -2.0706393E38F);
    assert(p54_p1y_GET(pack) == (float)8.625633E37F);
    assert(p54_p2z_GET(pack) == (float) -9.668556E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)217);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float) -1.9432195E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p55_p2z_GET(pack) == (float)5.302376E37F);
    assert(p55_p2x_GET(pack) == (float) -7.8461067E37F);
    assert(p55_p2y_GET(pack) == (float) -1.0428094E38F);
    assert(p55_p1x_GET(pack) == (float) -5.152813E37F);
    assert(p55_p1z_GET(pack) == (float)1.8454202E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_pitchspeed_GET(pack) == (float) -1.945015E38F);
    {
        float exemplary[] =  {-4.6551066E37F, 1.841075E38F, 6.941192E37F, 3.276021E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.4194383E38F, 1.4848559E38F, 9.522011E37F, 3.3635226E38F, -7.0221003E37F, 1.0776621E38F, 3.2409662E38F, 9.75316E37F, 2.9010131E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float) -2.6474682E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)5066805255881247041L);
    assert(p61_rollspeed_GET(pack) == (float)8.435971E36F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_pitch_GET(pack) == (float) -3.1460175E38F);
    assert(p62_xtrack_error_GET(pack) == (float)2.4800661E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)22895);
    assert(p62_nav_roll_GET(pack) == (float) -1.6099812E38F);
    assert(p62_aspd_error_GET(pack) == (float) -3.2717275E38F);
    assert(p62_alt_error_GET(pack) == (float)1.5340445E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)24030);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -11323);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_time_usec_GET(pack) == (uint64_t)8286766569434826014L);
    {
        float exemplary[] =  {-1.1831602E38F, -2.4988223E38F, -3.6336779E37F, 2.6564572E38F, -4.5433115E37F, -1.5335536E38F, -1.7733213E38F, 1.6355652E38F, 2.6020445E38F, 2.4276087E37F, 4.8087976E37F, 2.7801853E38F, -2.1592935E38F, -2.4115243E38F, 1.3657578E38F, 2.3471015E38F, 1.194695E38F, -2.3611595E38F, 4.1540084E37F, 1.483012E38F, -3.017619E37F, -9.084857E37F, -5.860799E37F, 1.916093E37F, -1.2141741E38F, 7.1696746E36F, -2.0136236E38F, -2.3617542E38F, -1.959452E38F, 1.5492882E38F, 1.6430326E37F, 2.437152E38F, -2.361568E38F, 2.0557498E38F, -2.8419007E38F, 2.5946887E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_lat_GET(pack) == (int32_t)1182524535);
    assert(p63_lon_GET(pack) == (int32_t)2116211021);
    assert(p63_vy_GET(pack) == (float) -2.0846291E37F);
    assert(p63_vz_GET(pack) == (float)2.0329903E38F);
    assert(p63_alt_GET(pack) == (int32_t) -979911915);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p63_vx_GET(pack) == (float)1.0961677E38F);
    assert(p63_relative_alt_GET(pack) == (int32_t)1906468442);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.3166539E38F, -1.8977024E38F, 2.9289085E38F, -2.321433E38F, -1.3812012E38F, -1.1033297E37F, -1.6461832E38F, 3.9590544E37F, 4.8606587E37F, -2.5097064E38F, -2.6541164E38F, 1.116026E38F, -1.5841765E37F, 1.944132E38F, -1.7226147E38F, -1.791416E38F, 5.1104127E37F, 3.3856885E38F, -1.4527263E38F, 4.4977617E37F, 2.5039152E38F, -7.922659E37F, -3.180821E38F, -6.487857E37F, -6.0821115E37F, -1.5362652E38F, -7.283622E37F, -7.3133165E36F, 2.4728685E38F, 2.91134E38F, -2.5133544E38F, -2.7240676E38F, -7.560125E37F, -6.496929E37F, 3.2478496E38F, -7.247916E37F, 1.9309503E38F, 1.3519073E38F, -1.9343085E38F, 1.3657355E38F, -1.1997834E38F, -1.7771954E38F, -3.372647E38F, -1.5452897E38F, 2.4074796E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_az_GET(pack) == (float) -3.5485364E37F);
    assert(p64_vz_GET(pack) == (float) -1.8783617E38F);
    assert(p64_y_GET(pack) == (float)2.8792682E38F);
    assert(p64_ay_GET(pack) == (float) -1.8123822E38F);
    assert(p64_ax_GET(pack) == (float)1.5904215E38F);
    assert(p64_vy_GET(pack) == (float)1.0726692E38F);
    assert(p64_x_GET(pack) == (float) -2.9162785E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)6981860961411528392L);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p64_vx_GET(pack) == (float) -1.1086005E38F);
    assert(p64_z_GET(pack) == (float) -3.8922238E37F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)26677);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)35606);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)3363);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)147355654L);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)1129);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)740);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)58387);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)61142);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)49777);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)16063);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)25426);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)46933);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)56591);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)13700);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)48346);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)2994);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)61516);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)35376);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)8009);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)47836);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)19628);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)96);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -6962);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)6582);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)3990);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -12928);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)32509);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)22820);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)14992);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)19052);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)46826);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)54898);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)28946);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)27697);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11236);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float)3.26239E38F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_RELAY);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)58398);
    assert(p73_param2_GET(pack) == (float) -6.384359E37F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p73_x_GET(pack) == (int32_t) -793747902);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p73_z_GET(pack) == (float) -1.1412985E38F);
    assert(p73_y_GET(pack) == (int32_t)2124863764);
    assert(p73_param1_GET(pack) == (float) -5.423102E37F);
    assert(p73_param3_GET(pack) == (float)9.372113E37F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_airspeed_GET(pack) == (float) -9.178037E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -31192);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)56074);
    assert(p74_climb_GET(pack) == (float) -7.330907E36F);
    assert(p74_groundspeed_GET(pack) == (float)4.1548468E37F);
    assert(p74_alt_GET(pack) == (float) -2.4698687E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_y_GET(pack) == (int32_t)1526727718);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p75_x_GET(pack) == (int32_t)1428639671);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p75_param1_GET(pack) == (float)6.20393E37F);
    assert(p75_param2_GET(pack) == (float) -3.2904719E38F);
    assert(p75_param3_GET(pack) == (float)2.3660027E38F);
    assert(p75_z_GET(pack) == (float)1.2981831E37F);
    assert(p75_param4_GET(pack) == (float) -1.6360606E38F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float)2.578075E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p76_param5_GET(pack) == (float) -2.9537514E38F);
    assert(p76_param1_GET(pack) == (float)1.4856071E38F);
    assert(p76_param3_GET(pack) == (float)5.5115354E37F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p76_param6_GET(pack) == (float)2.9473333E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p76_param4_GET(pack) == (float) -3.094811E38F);
    assert(p76_param7_GET(pack) == (float)2.9112507E38F);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)61);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1392551444);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)160);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)48);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_roll_GET(pack) == (float)7.2859557E37F);
    assert(p81_thrust_GET(pack) == (float)4.932428E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3182681482L);
    assert(p81_yaw_GET(pack) == (float) -2.5769128E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p81_pitch_GET(pack) == (float)2.6687722E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)253);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_roll_rate_GET(pack) == (float) -4.5887187E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p82_body_pitch_rate_GET(pack) == (float)3.0949497E38F);
    {
        float exemplary[] =  {1.7856136E38F, 1.1177703E38F, -2.1939154E38F, -9.711216E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p82_thrust_GET(pack) == (float) -8.0850755E37F);
    assert(p82_body_yaw_rate_GET(pack) == (float)2.9282567E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1314195844L);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)166);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_yaw_rate_GET(pack) == (float) -6.956786E37F);
    {
        float exemplary[] =  {1.5998052E38F, 4.2528737E37F, -2.1010351E38F, 2.8941404E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p83_body_pitch_rate_GET(pack) == (float) -3.5508072E37F);
    assert(p83_thrust_GET(pack) == (float) -4.5504113E37F);
    assert(p83_body_roll_rate_GET(pack) == (float)8.4701054E36F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)301509195L);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p84_yaw_rate_GET(pack) == (float) -2.0364613E37F);
    assert(p84_vz_GET(pack) == (float)3.0032985E37F);
    assert(p84_z_GET(pack) == (float)2.0610686E38F);
    assert(p84_y_GET(pack) == (float) -7.0852846E36F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p84_afy_GET(pack) == (float) -3.2157586E38F);
    assert(p84_afx_GET(pack) == (float)2.3768617E38F);
    assert(p84_yaw_GET(pack) == (float) -1.8551701E38F);
    assert(p84_afz_GET(pack) == (float) -2.3037978E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3928596148L);
    assert(p84_vy_GET(pack) == (float) -1.1648928E38F);
    assert(p84_x_GET(pack) == (float) -1.3894088E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)25093);
    assert(p84_vx_GET(pack) == (float) -2.891129E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)327883862L);
    assert(p86_vx_GET(pack) == (float) -1.4340605E38F);
    assert(p86_afx_GET(pack) == (float)2.82545E38F);
    assert(p86_yaw_GET(pack) == (float) -3.0244525E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)82254974);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p86_vz_GET(pack) == (float) -4.907848E37F);
    assert(p86_alt_GET(pack) == (float)5.992558E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)119);
    assert(p86_afy_GET(pack) == (float)7.9075696E35F);
    assert(p86_afz_GET(pack) == (float)1.0707007E38F);
    assert(p86_lat_int_GET(pack) == (int32_t) -1032605769);
    assert(p86_vy_GET(pack) == (float) -2.1245477E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -1.7621854E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)48);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_vy_GET(pack) == (float)1.8570982E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)1580129291);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)26291);
    assert(p87_afx_GET(pack) == (float) -2.6932175E37F);
    assert(p87_yaw_GET(pack) == (float) -1.0385156E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)3574555239L);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p87_lat_int_GET(pack) == (int32_t)1114570200);
    assert(p87_vx_GET(pack) == (float) -1.9479516E38F);
    assert(p87_alt_GET(pack) == (float)8.951415E37F);
    assert(p87_yaw_rate_GET(pack) == (float)2.2591641E38F);
    assert(p87_afy_GET(pack) == (float) -2.0977096E38F);
    assert(p87_vz_GET(pack) == (float)1.9902068E38F);
    assert(p87_afz_GET(pack) == (float) -1.4837552E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)4292054032L);
    assert(p89_x_GET(pack) == (float)6.546275E36F);
    assert(p89_z_GET(pack) == (float)1.2250702E38F);
    assert(p89_roll_GET(pack) == (float)3.0922452E38F);
    assert(p89_pitch_GET(pack) == (float) -2.6308358E38F);
    assert(p89_y_GET(pack) == (float) -7.049925E37F);
    assert(p89_yaw_GET(pack) == (float) -1.4626095E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_rollspeed_GET(pack) == (float) -1.796483E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -21029);
    assert(p90_alt_GET(pack) == (int32_t) -1387664493);
    assert(p90_lon_GET(pack) == (int32_t) -2141237150);
    assert(p90_lat_GET(pack) == (int32_t)2060587564);
    assert(p90_time_usec_GET(pack) == (uint64_t)5883753331587557378L);
    assert(p90_pitch_GET(pack) == (float) -7.386345E37F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)31394);
    assert(p90_yaw_GET(pack) == (float) -2.338789E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -3769);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)21016);
    assert(p90_pitchspeed_GET(pack) == (float)3.2370736E38F);
    assert(p90_yawspeed_GET(pack) == (float)1.2936937E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -7784);
    assert(p90_roll_GET(pack) == (float) -1.3282148E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -7964);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p91_yaw_rudder_GET(pack) == (float) -2.586998E38F);
    assert(p91_aux1_GET(pack) == (float)1.0659376E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
    assert(p91_roll_ailerons_GET(pack) == (float) -2.2546685E38F);
    assert(p91_aux4_GET(pack) == (float) -2.308129E38F);
    assert(p91_aux3_GET(pack) == (float)1.168957E37F);
    assert(p91_pitch_elevator_GET(pack) == (float)2.490925E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)1633818521005320738L);
    assert(p91_aux2_GET(pack) == (float)1.2564779E38F);
    assert(p91_throttle_GET(pack) == (float) -3.246258E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)44697);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)18977);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)42493);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)1092);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)53254);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)11834);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)44240);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)20674);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)19140);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)64505);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)18694);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)10620);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p92_time_usec_GET(pack) == (uint64_t)9013610475954690904L);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)2499566766528808647L);
    {
        float exemplary[] =  {-3.1173938E38F, 1.490342E38F, -1.4687787E38F, -1.947081E38F, 2.9679264E38F, 2.6024552E38F, 1.7256043E38F, 1.913395E38F, 2.752434E38F, 2.777667E38F, -2.344676E38F, -3.1658805E38F, -2.1105192E38F, 1.6557489E38F, -7.696911E37F, -1.4337323E36F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)2008899600502709537L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -13661);
    assert(p100_flow_comp_m_x_GET(pack) == (float)2.3765563E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float) -1.6020259E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p100_ground_distance_GET(pack) == (float) -5.9747694E37F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p100_time_usec_GET(pack) == (uint64_t)5557575625258427076L);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.703189E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)5.0919796E37F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -17715);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_x_GET(pack) == (float)7.414084E37F);
    assert(p101_y_GET(pack) == (float) -2.3965111E38F);
    assert(p101_usec_GET(pack) == (uint64_t)6398444975680307066L);
    assert(p101_roll_GET(pack) == (float) -6.565082E37F);
    assert(p101_z_GET(pack) == (float)3.2275751E38F);
    assert(p101_pitch_GET(pack) == (float) -5.02411E37F);
    assert(p101_yaw_GET(pack) == (float) -8.3678595E37F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_x_GET(pack) == (float) -2.0341529E38F);
    assert(p102_z_GET(pack) == (float) -1.4616549E38F);
    assert(p102_y_GET(pack) == (float)1.9951484E38F);
    assert(p102_usec_GET(pack) == (uint64_t)7657209185573825373L);
    assert(p102_yaw_GET(pack) == (float) -2.169269E38F);
    assert(p102_roll_GET(pack) == (float) -1.5092566E38F);
    assert(p102_pitch_GET(pack) == (float) -3.2416377E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)2.0595304E36F);
    assert(p103_z_GET(pack) == (float) -1.2530242E38F);
    assert(p103_usec_GET(pack) == (uint64_t)3868285834544025560L);
    assert(p103_x_GET(pack) == (float) -8.866778E37F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_roll_GET(pack) == (float) -2.3380325E38F);
    assert(p104_y_GET(pack) == (float)2.9889686E38F);
    assert(p104_z_GET(pack) == (float)1.199164E38F);
    assert(p104_pitch_GET(pack) == (float)1.1174046E38F);
    assert(p104_yaw_GET(pack) == (float) -2.8799044E38F);
    assert(p104_usec_GET(pack) == (uint64_t)5262888741313546849L);
    assert(p104_x_GET(pack) == (float) -2.565745E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_xgyro_GET(pack) == (float) -2.6532888E38F);
    assert(p105_ygyro_GET(pack) == (float) -2.4566655E38F);
    assert(p105_yacc_GET(pack) == (float) -2.0835693E38F);
    assert(p105_zmag_GET(pack) == (float) -1.3641233E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -3.3966688E38F);
    assert(p105_xacc_GET(pack) == (float)3.0868266E38F);
    assert(p105_diff_pressure_GET(pack) == (float) -2.3497153E38F);
    assert(p105_ymag_GET(pack) == (float)5.015795E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)5327408275006765055L);
    assert(p105_zgyro_GET(pack) == (float) -2.3790774E38F);
    assert(p105_temperature_GET(pack) == (float) -2.1814135E37F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)52110);
    assert(p105_abs_pressure_GET(pack) == (float)3.342753E38F);
    assert(p105_xmag_GET(pack) == (float) -5.646849E37F);
    assert(p105_zacc_GET(pack) == (float) -2.259699E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_ygyro_GET(pack) == (float)5.96777E37F);
    assert(p106_integrated_x_GET(pack) == (float) -1.4039103E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1984009130L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -6848);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.0228954E37F);
    assert(p106_integrated_y_GET(pack) == (float)3.0870367E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p106_distance_GET(pack) == (float)2.4651928E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3024775187L);
    assert(p106_integrated_zgyro_GET(pack) == (float)1.361553E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p106_time_usec_GET(pack) == (uint64_t)6243455471927714390L);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float)3.9472163E37F);
    assert(p107_abs_pressure_GET(pack) == (float) -2.6555053E38F);
    assert(p107_ygyro_GET(pack) == (float)1.1085054E38F);
    assert(p107_xacc_GET(pack) == (float) -3.083923E38F);
    assert(p107_ymag_GET(pack) == (float) -3.1447807E38F);
    assert(p107_yacc_GET(pack) == (float)2.4409365E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.763801E37F);
    assert(p107_zacc_GET(pack) == (float)1.7751402E38F);
    assert(p107_diff_pressure_GET(pack) == (float)2.0887452E38F);
    assert(p107_xmag_GET(pack) == (float)1.2262534E38F);
    assert(p107_xgyro_GET(pack) == (float)1.3246188E38F);
    assert(p107_temperature_GET(pack) == (float) -1.103519E38F);
    assert(p107_zgyro_GET(pack) == (float) -3.0440869E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1236904693L);
    assert(p107_time_usec_GET(pack) == (uint64_t)3497980501262514681L);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_xgyro_GET(pack) == (float) -1.8602357E38F);
    assert(p108_zgyro_GET(pack) == (float) -8.385314E36F);
    assert(p108_ygyro_GET(pack) == (float)1.6533707E38F);
    assert(p108_vd_GET(pack) == (float) -2.8346521E38F);
    assert(p108_yacc_GET(pack) == (float) -2.6946765E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -4.508862E37F);
    assert(p108_lat_GET(pack) == (float)1.7019737E38F);
    assert(p108_vn_GET(pack) == (float)1.984986E38F);
    assert(p108_q3_GET(pack) == (float)1.1320683E38F);
    assert(p108_zacc_GET(pack) == (float)1.6822182E38F);
    assert(p108_yaw_GET(pack) == (float) -5.8753897E37F);
    assert(p108_q2_GET(pack) == (float) -1.0085493E38F);
    assert(p108_q4_GET(pack) == (float) -1.8148735E38F);
    assert(p108_pitch_GET(pack) == (float)2.1608536E38F);
    assert(p108_ve_GET(pack) == (float)9.623275E37F);
    assert(p108_alt_GET(pack) == (float)1.251144E38F);
    assert(p108_q1_GET(pack) == (float) -3.3345981E38F);
    assert(p108_roll_GET(pack) == (float)4.2068058E37F);
    assert(p108_lon_GET(pack) == (float) -4.1214196E37F);
    assert(p108_xacc_GET(pack) == (float) -3.239696E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.65495E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)10576);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)22456);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)147);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)79, (uint8_t)155, (uint8_t)220, (uint8_t)2, (uint8_t)115, (uint8_t)196, (uint8_t)129, (uint8_t)196, (uint8_t)183, (uint8_t)226, (uint8_t)22, (uint8_t)68, (uint8_t)169, (uint8_t)55, (uint8_t)83, (uint8_t)135, (uint8_t)75, (uint8_t)96, (uint8_t)55, (uint8_t)111, (uint8_t)128, (uint8_t)165, (uint8_t)49, (uint8_t)85, (uint8_t)102, (uint8_t)4, (uint8_t)26, (uint8_t)84, (uint8_t)33, (uint8_t)53, (uint8_t)0, (uint8_t)194, (uint8_t)235, (uint8_t)141, (uint8_t)13, (uint8_t)124, (uint8_t)66, (uint8_t)71, (uint8_t)32, (uint8_t)99, (uint8_t)75, (uint8_t)209, (uint8_t)220, (uint8_t)42, (uint8_t)106, (uint8_t)116, (uint8_t)104, (uint8_t)25, (uint8_t)183, (uint8_t)144, (uint8_t)219, (uint8_t)3, (uint8_t)142, (uint8_t)95, (uint8_t)100, (uint8_t)57, (uint8_t)62, (uint8_t)114, (uint8_t)209, (uint8_t)23, (uint8_t)64, (uint8_t)102, (uint8_t)243, (uint8_t)205, (uint8_t)23, (uint8_t)37, (uint8_t)71, (uint8_t)179, (uint8_t)65, (uint8_t)26, (uint8_t)229, (uint8_t)170, (uint8_t)10, (uint8_t)28, (uint8_t)201, (uint8_t)1, (uint8_t)192, (uint8_t)110, (uint8_t)189, (uint8_t)217, (uint8_t)211, (uint8_t)230, (uint8_t)108, (uint8_t)159, (uint8_t)83, (uint8_t)245, (uint8_t)85, (uint8_t)209, (uint8_t)28, (uint8_t)205, (uint8_t)47, (uint8_t)36, (uint8_t)49, (uint8_t)81, (uint8_t)161, (uint8_t)230, (uint8_t)177, (uint8_t)189, (uint8_t)236, (uint8_t)228, (uint8_t)241, (uint8_t)30, (uint8_t)61, (uint8_t)112, (uint8_t)233, (uint8_t)98, (uint8_t)222, (uint8_t)161, (uint8_t)96, (uint8_t)89, (uint8_t)174, (uint8_t)85, (uint8_t)145, (uint8_t)171, (uint8_t)183, (uint8_t)62, (uint8_t)71, (uint8_t)99, (uint8_t)155, (uint8_t)138, (uint8_t)50, (uint8_t)112, (uint8_t)98, (uint8_t)191, (uint8_t)32, (uint8_t)54, (uint8_t)250, (uint8_t)200, (uint8_t)167, (uint8_t)162, (uint8_t)239, (uint8_t)137, (uint8_t)16, (uint8_t)198, (uint8_t)208, (uint8_t)93, (uint8_t)246, (uint8_t)123, (uint8_t)39, (uint8_t)253, (uint8_t)46, (uint8_t)123, (uint8_t)104, (uint8_t)46, (uint8_t)131, (uint8_t)203, (uint8_t)62, (uint8_t)169, (uint8_t)255, (uint8_t)34, (uint8_t)96, (uint8_t)110, (uint8_t)235, (uint8_t)244, (uint8_t)66, (uint8_t)79, (uint8_t)57, (uint8_t)23, (uint8_t)18, (uint8_t)66, (uint8_t)232, (uint8_t)192, (uint8_t)37, (uint8_t)197, (uint8_t)161, (uint8_t)29, (uint8_t)252, (uint8_t)23, (uint8_t)53, (uint8_t)108, (uint8_t)68, (uint8_t)102, (uint8_t)134, (uint8_t)116, (uint8_t)0, (uint8_t)213, (uint8_t)136, (uint8_t)228, (uint8_t)23, (uint8_t)14, (uint8_t)242, (uint8_t)65, (uint8_t)136, (uint8_t)9, (uint8_t)136, (uint8_t)48, (uint8_t)237, (uint8_t)78, (uint8_t)35, (uint8_t)177, (uint8_t)18, (uint8_t)63, (uint8_t)253, (uint8_t)134, (uint8_t)121, (uint8_t)147, (uint8_t)151, (uint8_t)180, (uint8_t)180, (uint8_t)147, (uint8_t)99, (uint8_t)240, (uint8_t)163, (uint8_t)228, (uint8_t)183, (uint8_t)161, (uint8_t)110, (uint8_t)103, (uint8_t)81, (uint8_t)24, (uint8_t)28, (uint8_t)91, (uint8_t)59, (uint8_t)232, (uint8_t)180, (uint8_t)106, (uint8_t)69, (uint8_t)242, (uint8_t)239, (uint8_t)128, (uint8_t)229, (uint8_t)252, (uint8_t)65, (uint8_t)12, (uint8_t)202, (uint8_t)47, (uint8_t)216, (uint8_t)108, (uint8_t)215, (uint8_t)145, (uint8_t)195, (uint8_t)26, (uint8_t)6, (uint8_t)196, (uint8_t)57, (uint8_t)233, (uint8_t)15, (uint8_t)48, (uint8_t)164, (uint8_t)15, (uint8_t)33, (uint8_t)200, (uint8_t)27, (uint8_t)229, (uint8_t)33, (uint8_t)14, (uint8_t)63, (uint8_t)127, (uint8_t)216, (uint8_t)90, (uint8_t)201} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t) -7518042076245032089L);
    assert(p111_ts1_GET(pack) == (int64_t) -6521012519485536926L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)6412232158866052472L);
    assert(p112_seq_GET(pack) == (uint32_t)3413108836L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)25683);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)37490);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)56578);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)51872);
    assert(p113_time_usec_GET(pack) == (uint64_t)6695839315138885465L);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -7687);
    assert(p113_alt_GET(pack) == (int32_t)1698394982);
    assert(p113_lon_GET(pack) == (int32_t)1348572436);
    assert(p113_lat_GET(pack) == (int32_t) -985472097);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)413);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -32211);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)30000);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2828737808L);
    assert(p114_integrated_x_GET(pack) == (float) -2.2074703E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float)1.7231077E38F);
    assert(p114_integrated_y_GET(pack) == (float)1.8664173E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p114_integrated_xgyro_GET(pack) == (float) -3.0351792E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p114_time_usec_GET(pack) == (uint64_t)2485775584177143598L);
    assert(p114_distance_GET(pack) == (float)3.3627166E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.1208134E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1204599219L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)5066);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)16599);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)8531);
    assert(p115_yawspeed_GET(pack) == (float)2.1073369E38F);
    assert(p115_lat_GET(pack) == (int32_t) -101551254);
    assert(p115_time_usec_GET(pack) == (uint64_t)286251839477431411L);
    assert(p115_alt_GET(pack) == (int32_t)1322358730);
    assert(p115_lon_GET(pack) == (int32_t) -437800818);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)56161);
    assert(p115_rollspeed_GET(pack) == (float) -2.0791662E38F);
    assert(p115_pitchspeed_GET(pack) == (float)1.6015384E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)9715);
    {
        float exemplary[] =  {-2.1573367E38F, -1.796702E38F, 4.9040458E36F, 1.5565634E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -775);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -4105);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -6107);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)10054);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)6547);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)27999);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)1366183339L);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)6279);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -25569);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)8982);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)599);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -6614);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -15357);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)13937);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)12058);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)191);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)49101);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)5075);
    assert(p118_size_GET(pack) == (uint32_t)773196716L);
    assert(p118_time_utc_GET(pack) == (uint32_t)3459459763L);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)19089);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)25276);
    assert(p119_count_GET(pack) == (uint32_t)2395286891L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p119_ofs_GET(pack) == (uint32_t)660334345L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)848742864L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)21665);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)250);
    {
        uint8_t exemplary[] =  {(uint8_t)227, (uint8_t)30, (uint8_t)36, (uint8_t)93, (uint8_t)205, (uint8_t)146, (uint8_t)18, (uint8_t)184, (uint8_t)110, (uint8_t)144, (uint8_t)84, (uint8_t)232, (uint8_t)121, (uint8_t)81, (uint8_t)247, (uint8_t)165, (uint8_t)199, (uint8_t)171, (uint8_t)6, (uint8_t)194, (uint8_t)24, (uint8_t)160, (uint8_t)75, (uint8_t)215, (uint8_t)163, (uint8_t)149, (uint8_t)169, (uint8_t)255, (uint8_t)3, (uint8_t)206, (uint8_t)203, (uint8_t)56, (uint8_t)204, (uint8_t)49, (uint8_t)12, (uint8_t)181, (uint8_t)176, (uint8_t)6, (uint8_t)48, (uint8_t)63, (uint8_t)4, (uint8_t)4, (uint8_t)89, (uint8_t)53, (uint8_t)144, (uint8_t)237, (uint8_t)215, (uint8_t)194, (uint8_t)50, (uint8_t)17, (uint8_t)9, (uint8_t)30, (uint8_t)52, (uint8_t)111, (uint8_t)66, (uint8_t)55, (uint8_t)131, (uint8_t)30, (uint8_t)248, (uint8_t)49, (uint8_t)167, (uint8_t)26, (uint8_t)192, (uint8_t)194, (uint8_t)188, (uint8_t)210, (uint8_t)20, (uint8_t)210, (uint8_t)152, (uint8_t)5, (uint8_t)49, (uint8_t)175, (uint8_t)3, (uint8_t)190, (uint8_t)224, (uint8_t)30, (uint8_t)153, (uint8_t)192, (uint8_t)191, (uint8_t)13, (uint8_t)30, (uint8_t)42, (uint8_t)170, (uint8_t)107, (uint8_t)179, (uint8_t)42, (uint8_t)217, (uint8_t)186, (uint8_t)123, (uint8_t)157} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)255);
    {
        uint8_t exemplary[] =  {(uint8_t)159, (uint8_t)25, (uint8_t)82, (uint8_t)119, (uint8_t)19, (uint8_t)245, (uint8_t)1, (uint8_t)208, (uint8_t)47, (uint8_t)157, (uint8_t)89, (uint8_t)170, (uint8_t)94, (uint8_t)12, (uint8_t)12, (uint8_t)71, (uint8_t)141, (uint8_t)165, (uint8_t)54, (uint8_t)137, (uint8_t)199, (uint8_t)234, (uint8_t)0, (uint8_t)245, (uint8_t)222, (uint8_t)109, (uint8_t)158, (uint8_t)78, (uint8_t)39, (uint8_t)3, (uint8_t)55, (uint8_t)132, (uint8_t)66, (uint8_t)196, (uint8_t)57, (uint8_t)154, (uint8_t)118, (uint8_t)130, (uint8_t)126, (uint8_t)61, (uint8_t)165, (uint8_t)230, (uint8_t)173, (uint8_t)89, (uint8_t)57, (uint8_t)113, (uint8_t)84, (uint8_t)235, (uint8_t)209, (uint8_t)236, (uint8_t)87, (uint8_t)170, (uint8_t)48, (uint8_t)180, (uint8_t)6, (uint8_t)74, (uint8_t)138, (uint8_t)26, (uint8_t)101, (uint8_t)232, (uint8_t)205, (uint8_t)31, (uint8_t)126, (uint8_t)24, (uint8_t)197, (uint8_t)172, (uint8_t)68, (uint8_t)167, (uint8_t)130, (uint8_t)113, (uint8_t)100, (uint8_t)203, (uint8_t)81, (uint8_t)99, (uint8_t)82, (uint8_t)45, (uint8_t)219, (uint8_t)211, (uint8_t)57, (uint8_t)61, (uint8_t)78, (uint8_t)32, (uint8_t)200, (uint8_t)133, (uint8_t)241, (uint8_t)122, (uint8_t)241, (uint8_t)60, (uint8_t)217, (uint8_t)138, (uint8_t)9, (uint8_t)168, (uint8_t)255, (uint8_t)12, (uint8_t)118, (uint8_t)234, (uint8_t)150, (uint8_t)64, (uint8_t)219, (uint8_t)77, (uint8_t)17, (uint8_t)22, (uint8_t)50, (uint8_t)168, (uint8_t)121, (uint8_t)27, (uint8_t)167, (uint8_t)0, (uint8_t)7, (uint8_t)17} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_time_usec_GET(pack) == (uint64_t)6255810399688821307L);
    assert(p124_alt_GET(pack) == (int32_t)1320078496);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)40410);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_dgps_age_GET(pack) == (uint32_t)3267072070L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)35761);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)48208);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p124_lat_GET(pack) == (int32_t) -409528298);
    assert(p124_lon_GET(pack) == (int32_t) -414116519);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)39442);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED));
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)45860);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)11695);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p126_baudrate_GET(pack) == (uint32_t)1078530722L);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)26283);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING));
    {
        uint8_t exemplary[] =  {(uint8_t)37, (uint8_t)183, (uint8_t)3, (uint8_t)220, (uint8_t)53, (uint8_t)167, (uint8_t)166, (uint8_t)203, (uint8_t)172, (uint8_t)170, (uint8_t)244, (uint8_t)128, (uint8_t)73, (uint8_t)228, (uint8_t)162, (uint8_t)224, (uint8_t)18, (uint8_t)120, (uint8_t)80, (uint8_t)190, (uint8_t)78, (uint8_t)13, (uint8_t)124, (uint8_t)246, (uint8_t)65, (uint8_t)181, (uint8_t)235, (uint8_t)1, (uint8_t)6, (uint8_t)67, (uint8_t)117, (uint8_t)99, (uint8_t)11, (uint8_t)112, (uint8_t)210, (uint8_t)117, (uint8_t)63, (uint8_t)113, (uint8_t)12, (uint8_t)31, (uint8_t)173, (uint8_t)87, (uint8_t)49, (uint8_t)110, (uint8_t)19, (uint8_t)154, (uint8_t)114, (uint8_t)9, (uint8_t)133, (uint8_t)207, (uint8_t)205, (uint8_t)88, (uint8_t)0, (uint8_t)54, (uint8_t)96, (uint8_t)173, (uint8_t)25, (uint8_t)238, (uint8_t)176, (uint8_t)147, (uint8_t)231, (uint8_t)111, (uint8_t)174, (uint8_t)187, (uint8_t)4, (uint8_t)233, (uint8_t)242, (uint8_t)103, (uint8_t)16, (uint8_t)119} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_tow_GET(pack) == (uint32_t)624118737L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)3992493200L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -234162584);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -702188722);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1266712628);
    assert(p127_accuracy_GET(pack) == (uint32_t)886624138L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)33311);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -1129655576);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)1228148472);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)30218);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)1794090091L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)1650103125);
    assert(p128_tow_GET(pack) == (uint32_t)2315013519L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1013020325);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1758564639);
    assert(p128_accuracy_GET(pack) == (uint32_t)2428918696L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -943);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -4778);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)5630);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -12241);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)895988823L);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -9821);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -30448);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)18114);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)17482);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -25915);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)9263);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)58901);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p130_size_GET(pack) == (uint32_t)374300074L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)15142);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)208, (uint8_t)213, (uint8_t)90, (uint8_t)196, (uint8_t)202, (uint8_t)180, (uint8_t)31, (uint8_t)78, (uint8_t)100, (uint8_t)0, (uint8_t)9, (uint8_t)141, (uint8_t)149, (uint8_t)212, (uint8_t)58, (uint8_t)167, (uint8_t)84, (uint8_t)182, (uint8_t)109, (uint8_t)110, (uint8_t)41, (uint8_t)224, (uint8_t)71, (uint8_t)66, (uint8_t)230, (uint8_t)67, (uint8_t)53, (uint8_t)186, (uint8_t)137, (uint8_t)248, (uint8_t)125, (uint8_t)30, (uint8_t)123, (uint8_t)55, (uint8_t)136, (uint8_t)124, (uint8_t)219, (uint8_t)102, (uint8_t)170, (uint8_t)70, (uint8_t)15, (uint8_t)20, (uint8_t)205, (uint8_t)162, (uint8_t)58, (uint8_t)246, (uint8_t)16, (uint8_t)110, (uint8_t)89, (uint8_t)185, (uint8_t)230, (uint8_t)14, (uint8_t)194, (uint8_t)17, (uint8_t)93, (uint8_t)164, (uint8_t)186, (uint8_t)69, (uint8_t)49, (uint8_t)140, (uint8_t)21, (uint8_t)226, (uint8_t)95, (uint8_t)86, (uint8_t)57, (uint8_t)218, (uint8_t)128, (uint8_t)202, (uint8_t)212, (uint8_t)103, (uint8_t)199, (uint8_t)142, (uint8_t)222, (uint8_t)38, (uint8_t)34, (uint8_t)140, (uint8_t)155, (uint8_t)249, (uint8_t)113, (uint8_t)61, (uint8_t)12, (uint8_t)254, (uint8_t)53, (uint8_t)25, (uint8_t)185, (uint8_t)226, (uint8_t)73, (uint8_t)2, (uint8_t)38, (uint8_t)56, (uint8_t)122, (uint8_t)13, (uint8_t)210, (uint8_t)93, (uint8_t)109, (uint8_t)108, (uint8_t)129, (uint8_t)49, (uint8_t)44, (uint8_t)119, (uint8_t)137, (uint8_t)172, (uint8_t)189, (uint8_t)198, (uint8_t)1, (uint8_t)207, (uint8_t)79, (uint8_t)239, (uint8_t)41, (uint8_t)213, (uint8_t)111, (uint8_t)165, (uint8_t)194, (uint8_t)119, (uint8_t)78, (uint8_t)103, (uint8_t)161, (uint8_t)34, (uint8_t)42, (uint8_t)220, (uint8_t)112, (uint8_t)120, (uint8_t)34, (uint8_t)74, (uint8_t)45, (uint8_t)4, (uint8_t)248, (uint8_t)254, (uint8_t)157, (uint8_t)196, (uint8_t)33, (uint8_t)84, (uint8_t)72, (uint8_t)112, (uint8_t)138, (uint8_t)105, (uint8_t)128, (uint8_t)86, (uint8_t)152, (uint8_t)197, (uint8_t)184, (uint8_t)224, (uint8_t)125, (uint8_t)48, (uint8_t)236, (uint8_t)48, (uint8_t)107, (uint8_t)150, (uint8_t)165, (uint8_t)143, (uint8_t)35, (uint8_t)81, (uint8_t)237, (uint8_t)235, (uint8_t)156, (uint8_t)65, (uint8_t)182, (uint8_t)125, (uint8_t)73, (uint8_t)0, (uint8_t)45, (uint8_t)136, (uint8_t)1, (uint8_t)124, (uint8_t)43, (uint8_t)1, (uint8_t)85, (uint8_t)209, (uint8_t)58, (uint8_t)54, (uint8_t)142, (uint8_t)5, (uint8_t)21, (uint8_t)225, (uint8_t)172, (uint8_t)30, (uint8_t)41, (uint8_t)144, (uint8_t)151, (uint8_t)181, (uint8_t)16, (uint8_t)55, (uint8_t)176, (uint8_t)230, (uint8_t)87, (uint8_t)200, (uint8_t)240, (uint8_t)92, (uint8_t)155, (uint8_t)191, (uint8_t)213, (uint8_t)173, (uint8_t)169, (uint8_t)39, (uint8_t)144, (uint8_t)38, (uint8_t)225, (uint8_t)250, (uint8_t)126, (uint8_t)103, (uint8_t)87, (uint8_t)235, (uint8_t)156, (uint8_t)102, (uint8_t)42, (uint8_t)215, (uint8_t)184, (uint8_t)11, (uint8_t)63, (uint8_t)15, (uint8_t)204, (uint8_t)111, (uint8_t)71, (uint8_t)113, (uint8_t)43, (uint8_t)18, (uint8_t)150, (uint8_t)234, (uint8_t)158, (uint8_t)127, (uint8_t)216, (uint8_t)227, (uint8_t)127, (uint8_t)252, (uint8_t)168, (uint8_t)123, (uint8_t)208, (uint8_t)228, (uint8_t)5, (uint8_t)98, (uint8_t)64, (uint8_t)24, (uint8_t)179, (uint8_t)164, (uint8_t)129, (uint8_t)21, (uint8_t)239, (uint8_t)222, (uint8_t)146, (uint8_t)168, (uint8_t)220, (uint8_t)76, (uint8_t)154, (uint8_t)110, (uint8_t)3, (uint8_t)202, (uint8_t)171, (uint8_t)100, (uint8_t)124, (uint8_t)48, (uint8_t)0, (uint8_t)110} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)15552);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_270);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)15752);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2905154918L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)8900);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)38359);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)26557);
    assert(p133_lon_GET(pack) == (int32_t) -1424407746);
    assert(p133_mask_GET(pack) == (uint64_t)2191441364107831369L);
    assert(p133_lat_GET(pack) == (int32_t)2129197718);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lon_GET(pack) == (int32_t)314718699);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)20864);
    assert(p134_lat_GET(pack) == (int32_t)132995115);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)143);
    {
        int16_t exemplary[] =  {(int16_t) -8808, (int16_t) -8283, (int16_t)30294, (int16_t)7532, (int16_t) -115, (int16_t) -26668, (int16_t)32005, (int16_t)3451, (int16_t)7980, (int16_t) -32428, (int16_t)32062, (int16_t)11547, (int16_t)13928, (int16_t)21574, (int16_t)815, (int16_t)27358} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t) -1694972628);
    assert(p135_lon_GET(pack) == (int32_t) -57571477);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)51408);
    assert(p136_current_height_GET(pack) == (float) -3.2028582E38F);
    assert(p136_terrain_height_GET(pack) == (float) -5.894707E37F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)61533);
    assert(p136_lat_GET(pack) == (int32_t)1782495982);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)27287);
    assert(p136_lon_GET(pack) == (int32_t)258694502);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)3.3457875E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)29333);
    assert(p137_press_diff_GET(pack) == (float) -6.037726E37F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)1809440919L);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float) -2.5245982E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)5700794980596979842L);
    assert(p138_z_GET(pack) == (float) -1.7813355E38F);
    {
        float exemplary[] =  {-3.647309E37F, -2.3497543E38F, 7.6665764E37F, -1.4766395E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_y_GET(pack) == (float) -4.864497E37F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)6592813568357024449L);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)85);
    {
        float exemplary[] =  {-2.217441E38F, -1.0152843E38F, -2.794164E38F, -3.3208996E38F, 6.8186634E36F, 2.0835507E38F, 1.5865358E38F, -1.0463256E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)7166657120244584223L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)163);
    {
        float exemplary[] =  {3.0582261E38F, -2.12855E38F, -1.792377E38F, 4.805427E37F, 2.9343377E38F, 1.3483068E38F, 2.9879506E38F, -1.5830491E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float)1.2762322E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)2.3296213E37F);
    assert(p141_altitude_local_GET(pack) == (float)1.9491576E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -4.637614E37F);
    assert(p141_altitude_relative_GET(pack) == (float) -2.8852612E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)5561552114062385192L);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.5537455E37F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)13, (uint8_t)212, (uint8_t)143, (uint8_t)223, (uint8_t)237, (uint8_t)142, (uint8_t)161, (uint8_t)250, (uint8_t)204, (uint8_t)63, (uint8_t)169, (uint8_t)183, (uint8_t)165, (uint8_t)188, (uint8_t)142, (uint8_t)122, (uint8_t)222, (uint8_t)87, (uint8_t)129, (uint8_t)49, (uint8_t)38, (uint8_t)52, (uint8_t)200, (uint8_t)75, (uint8_t)242, (uint8_t)115, (uint8_t)172, (uint8_t)109, (uint8_t)114, (uint8_t)121, (uint8_t)197, (uint8_t)9, (uint8_t)38, (uint8_t)147, (uint8_t)146, (uint8_t)104, (uint8_t)67, (uint8_t)209, (uint8_t)204, (uint8_t)242, (uint8_t)234, (uint8_t)242, (uint8_t)88, (uint8_t)170, (uint8_t)28, (uint8_t)146, (uint8_t)0, (uint8_t)187, (uint8_t)217, (uint8_t)249, (uint8_t)75, (uint8_t)115, (uint8_t)17, (uint8_t)59, (uint8_t)45, (uint8_t)54, (uint8_t)182, (uint8_t)136, (uint8_t)165, (uint8_t)118, (uint8_t)57, (uint8_t)94, (uint8_t)55, (uint8_t)142, (uint8_t)85, (uint8_t)222, (uint8_t)14, (uint8_t)219, (uint8_t)129, (uint8_t)157, (uint8_t)123, (uint8_t)148, (uint8_t)241, (uint8_t)125, (uint8_t)45, (uint8_t)139, (uint8_t)130, (uint8_t)123, (uint8_t)106, (uint8_t)215, (uint8_t)34, (uint8_t)132, (uint8_t)254, (uint8_t)120, (uint8_t)172, (uint8_t)119, (uint8_t)168, (uint8_t)46, (uint8_t)42, (uint8_t)63, (uint8_t)155, (uint8_t)231, (uint8_t)60, (uint8_t)29, (uint8_t)163, (uint8_t)89, (uint8_t)127, (uint8_t)89, (uint8_t)16, (uint8_t)119, (uint8_t)149, (uint8_t)169, (uint8_t)223, (uint8_t)26, (uint8_t)100, (uint8_t)31, (uint8_t)170, (uint8_t)19, (uint8_t)203, (uint8_t)161, (uint8_t)104, (uint8_t)62, (uint8_t)168, (uint8_t)35, (uint8_t)23, (uint8_t)158, (uint8_t)235, (uint8_t)52, (uint8_t)189, (uint8_t)188} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)94);
    {
        uint8_t exemplary[] =  {(uint8_t)47, (uint8_t)49, (uint8_t)5, (uint8_t)58, (uint8_t)56, (uint8_t)110, (uint8_t)136, (uint8_t)236, (uint8_t)212, (uint8_t)71, (uint8_t)130, (uint8_t)26, (uint8_t)202, (uint8_t)173, (uint8_t)150, (uint8_t)192, (uint8_t)191, (uint8_t)26, (uint8_t)91, (uint8_t)121, (uint8_t)95, (uint8_t)228, (uint8_t)73, (uint8_t)221, (uint8_t)165, (uint8_t)55, (uint8_t)68, (uint8_t)216, (uint8_t)14, (uint8_t)110, (uint8_t)123, (uint8_t)56, (uint8_t)200, (uint8_t)202, (uint8_t)147, (uint8_t)218, (uint8_t)220, (uint8_t)34, (uint8_t)27, (uint8_t)102, (uint8_t)185, (uint8_t)254, (uint8_t)212, (uint8_t)131, (uint8_t)53, (uint8_t)10, (uint8_t)26, (uint8_t)165, (uint8_t)92, (uint8_t)211, (uint8_t)103, (uint8_t)90, (uint8_t)31, (uint8_t)138, (uint8_t)74, (uint8_t)236, (uint8_t)178, (uint8_t)118, (uint8_t)224, (uint8_t)122, (uint8_t)73, (uint8_t)187, (uint8_t)6, (uint8_t)57, (uint8_t)94, (uint8_t)225, (uint8_t)48, (uint8_t)39, (uint8_t)51, (uint8_t)135, (uint8_t)60, (uint8_t)115, (uint8_t)84, (uint8_t)63, (uint8_t)203, (uint8_t)16, (uint8_t)8, (uint8_t)46, (uint8_t)38, (uint8_t)243, (uint8_t)242, (uint8_t)153, (uint8_t)135, (uint8_t)204, (uint8_t)163, (uint8_t)6, (uint8_t)186, (uint8_t)152, (uint8_t)201, (uint8_t)84, (uint8_t)47, (uint8_t)32, (uint8_t)218, (uint8_t)78, (uint8_t)70, (uint8_t)155, (uint8_t)231, (uint8_t)155, (uint8_t)64, (uint8_t)30, (uint8_t)154, (uint8_t)97, (uint8_t)186, (uint8_t)247, (uint8_t)197, (uint8_t)136, (uint8_t)249, (uint8_t)211, (uint8_t)190, (uint8_t)184, (uint8_t)166, (uint8_t)231, (uint8_t)135, (uint8_t)130, (uint8_t)175, (uint8_t)8, (uint8_t)145, (uint8_t)107, (uint8_t)0, (uint8_t)158} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)47);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)3635556367L);
    assert(p143_press_abs_GET(pack) == (float) -2.5283293E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)22268);
    assert(p143_press_diff_GET(pack) == (float)3.267028E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.9020164E38F, 2.4460359E38F, -1.180927E37F, -1.2886351E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)89);
    {
        float exemplary[] =  {-1.8181487E38F, 2.8767687E37F, 4.9051633E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -6.6188743E37F);
    {
        float exemplary[] =  {-2.0172284E38F, -2.2732275E38F, 3.062219E36F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)3676090886080731472L);
    assert(p144_lon_GET(pack) == (int32_t)2136275444);
    assert(p144_custom_state_GET(pack) == (uint64_t)7731858259760842996L);
    {
        float exemplary[] =  {5.1646727E37F, 1.1015106E38F, -4.677033E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -1405692460);
    {
        float exemplary[] =  {-1.1132608E38F, 3.1018802E38F, -3.2896561E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.4235881E38F, 5.5510856E37F, -1.726105E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float)1.5940383E38F);
    {
        float exemplary[] =  {-1.6049656E38F, 1.419711E38F, 6.444722E37F, -2.7566553E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float) -7.426149E37F);
    assert(p146_z_pos_GET(pack) == (float) -2.4375771E38F);
    assert(p146_z_vel_GET(pack) == (float)6.3229834E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)3107479848229421350L);
    assert(p146_pitch_rate_GET(pack) == (float) -3.090332E38F);
    {
        float exemplary[] =  {-1.0421911E38F, 2.4738706E38F, -1.2263813E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float) -1.344342E36F);
    assert(p146_x_vel_GET(pack) == (float)2.5006242E38F);
    assert(p146_y_pos_GET(pack) == (float)5.5902423E37F);
    assert(p146_yaw_rate_GET(pack) == (float)3.2743486E38F);
    assert(p146_x_acc_GET(pack) == (float)5.998521E37F);
    assert(p146_z_acc_GET(pack) == (float) -2.3808233E38F);
    assert(p146_x_pos_GET(pack) == (float) -1.1833206E38F);
    assert(p146_roll_rate_GET(pack) == (float)3.1179682E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -31869);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    {
        uint16_t exemplary[] =  {(uint16_t)28873, (uint16_t)62281, (uint16_t)56585, (uint16_t)10276, (uint16_t)19143, (uint16_t)17871, (uint16_t)31019, (uint16_t)6804, (uint16_t)22989, (uint16_t)29694} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_consumed_GET(pack) == (int32_t) -1294115361);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)32424);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1261267142);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -84);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)187);
    {
        uint8_t exemplary[] =  {(uint8_t)248, (uint8_t)95, (uint8_t)226, (uint8_t)179, (uint8_t)79, (uint8_t)18, (uint8_t)3, (uint8_t)172, (uint8_t)221, (uint8_t)249, (uint8_t)65, (uint8_t)215, (uint8_t)101, (uint8_t)68, (uint8_t)103, (uint8_t)66, (uint8_t)69, (uint8_t)152} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2269762600L);
    {
        uint8_t exemplary[] =  {(uint8_t)169, (uint8_t)222, (uint8_t)18, (uint8_t)81, (uint8_t)225, (uint8_t)241, (uint8_t)69, (uint8_t)89} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)8800815950528474068L);
    {
        uint8_t exemplary[] =  {(uint8_t)17, (uint8_t)76, (uint8_t)213, (uint8_t)21, (uint8_t)15, (uint8_t)101, (uint8_t)99, (uint8_t)236} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)89, (uint8_t)107, (uint8_t)106, (uint8_t)94, (uint8_t)120, (uint8_t)35, (uint8_t)135} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)4195);
    assert(p148_board_version_GET(pack) == (uint32_t)797478935L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)179415996L);
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY));
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1112654780L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_angle_x_GET(pack) == (float) -2.2936823E38F);
    {
        float exemplary[] =  {3.066418E37F, 2.094703E38F, 3.1274165E37F, 2.8510571E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_z_TRY(ph) == (float)4.516127E37F);
    assert(p149_time_usec_GET(pack) == (uint64_t)4729231930566739083L);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p149_distance_GET(pack) == (float)1.1006665E38F);
    assert(p149_size_x_GET(pack) == (float)9.424388E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)3);
    assert(p149_y_TRY(ph) == (float) -1.1763967E38F);
    assert(p149_size_y_GET(pack) == (float) -2.262753E38F);
    assert(p149_angle_y_GET(pack) == (float)2.2759952E37F);
    assert(p149_x_TRY(ph) == (float) -5.498283E37F);
};


void c_CommunicationChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_ctrlLoad_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p170_batVolt_GET(pack) == (uint16_t)(uint16_t)17291);
    assert(p170_sensLoad_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_CommunicationChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_ayBias_GET(pack) == (float) -1.3463437E38F);
    assert(p172_gxBias_GET(pack) == (float) -1.1179866E38F);
    assert(p172_gyBias_GET(pack) == (float)6.9777624E37F);
    assert(p172_azBias_GET(pack) == (float) -1.6882394E37F);
    assert(p172_axBias_GET(pack) == (float) -4.1323194E36F);
    assert(p172_gzBias_GET(pack) == (float) -1.9887842E38F);
};


void c_CommunicationChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_diagFl1_GET(pack) == (float) -1.1183116E38F);
    assert(p173_diagSh1_GET(pack) == (int16_t)(int16_t)8001);
    assert(p173_diagFl2_GET(pack) == (float) -1.3661112E38F);
    assert(p173_diagSh2_GET(pack) == (int16_t)(int16_t) -14014);
    assert(p173_diagFl3_GET(pack) == (float) -1.7033938E37F);
    assert(p173_diagSh3_GET(pack) == (int16_t)(int16_t)21160);
};


void c_CommunicationChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_phi_c_GET(pack) == (float) -1.9709586E38F);
    assert(p176_fromWP_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p176_u_m_GET(pack) == (float)2.1413178E38F);
    assert(p176_psiDot_c_GET(pack) == (float) -1.8198232E38F);
    assert(p176_toWP_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p176_ay_body_GET(pack) == (float) -3.9920008E37F);
    assert(p176_h_c_GET(pack) == (uint16_t)(uint16_t)21155);
    assert(p176_totalDist_GET(pack) == (float) -2.7879488E38F);
    assert(p176_dist2Go_GET(pack) == (float) -3.2713117E38F);
    assert(p176_theta_c_GET(pack) == (float) -2.1365563E38F);
};


void c_CommunicationChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_fl_2_GET(pack) == (float) -5.089991E37F);
    assert(p177_fl_6_GET(pack) == (float) -2.7682974E38F);
    assert(p177_fl_1_GET(pack) == (float)1.822784E38F);
    assert(p177_fl_5_GET(pack) == (float) -3.2218441E38F);
    assert(p177_fl_3_GET(pack) == (float)1.0619669E38F);
    assert(p177_fl_4_GET(pack) == (float)2.0712196E38F);
};


void c_CommunicationChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_year_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p179_hour_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p179_day_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p179_useSat_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p179_sigUsedMask_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p179_visSat_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p179_min_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p179_clockStat_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p179_month_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p179_GppGl_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p179_sec_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p179_percentUsed_GET(pack) == (uint8_t)(uint8_t)153);
};


void c_CommunicationChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_hCommand_GET(pack) == (float)3.034143E38F);
    assert(p180_uCommand_GET(pack) == (float) -3.2398561E38F);
    assert(p180_target_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p180_rCommand_GET(pack) == (float) -1.8757963E38F);
};


void c_CommunicationChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_target_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p181_bitfieldPt_GET(pack) == (uint16_t)(uint16_t)41839);
};


void c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_zoom_GET(pack) == (int8_t)(int8_t)117);
    assert(p184_pan_GET(pack) == (int8_t)(int8_t) -119);
    assert(p184_tilt_GET(pack) == (int8_t)(int8_t) -14);
    assert(p184_target_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p184_moveHome_GET(pack) == (int8_t)(int8_t) -6);
};


void c_CommunicationChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_mControl_GET(pack) == (float) -2.9351894E38F);
    assert(p185_target_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p185_idSurface_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p185_bControl_GET(pack) == (float) -2.8952645E37F);
};


void c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_latitude_GET(pack) == (float) -1.6528221E38F);
    assert(p186_target_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p186_longitude_GET(pack) == (float) -4.1761459E37F);
};


void c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_idOrder_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p188_order_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p188_target_GET(pack) == (uint8_t)(uint8_t)81);
};


void c_CommunicationChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    assert(p189_height_GET(pack) == (float)2.6341958E38F);
    assert(p189_option2_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p189_target_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p189_option1_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p189_latitude_GET(pack) == (float) -2.3901422E38F);
    assert(p189_longitude_GET(pack) == (float) -2.6601863E38F);
    assert(p189_option3_GET(pack) == (uint8_t)(uint8_t)244);
};


void c_CommunicationChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_reading2_GET(pack) == (uint16_t)(uint16_t)9884);
    assert(p191_r2Type_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p191_voltage_GET(pack) == (uint16_t)(uint16_t)9679);
};


void c_CommunicationChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_zoom_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p192_pan_GET(pack) == (int16_t)(int16_t) -864);
    assert(p192_tilt_GET(pack) == (int16_t)(int16_t)23241);
};


void c_CommunicationChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_latitude_GET(pack) == (float) -2.2501577E38F);
    assert(p193_altitude_GET(pack) == (float)1.5352169E38F);
    assert(p193_course_GET(pack) == (float)2.4969291E38F);
    assert(p193_target_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p193_longitude_GET(pack) == (float) -2.7200977E38F);
    assert(p193_speed_GET(pack) == (float) -2.2001652E38F);
};


void c_CommunicationChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_posStatus_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p194_modeInd_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p194_magDir_GET(pack) == (int8_t)(int8_t)81);
    assert(p194_msgsType_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p194_csFails_GET(pack) == (uint16_t)(uint16_t)2533);
    assert(p194_gpsQuality_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p194_magVar_GET(pack) == (float)2.7204934E38F);
};


void c_CommunicationChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    assert(p195_velType_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p195_solStatus_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p195_csFails_GET(pack) == (uint16_t)(uint16_t)42185);
    assert(p195_posType_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p195_timeStatus_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p195_receiverStatus_GET(pack) == (uint32_t)3528696415L);
    assert(p195_posSolAge_GET(pack) == (float)1.4680518E38F);
};


void c_CommunicationChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    assert(p196_char1_GET(pack) == (int8_t)(int8_t) -68);
    assert(p196_float1_GET(pack) == (float)1.6403435E38F);
    assert(p196_int1_GET(pack) == (int16_t)(int16_t)23095);
    assert(p196_float2_GET(pack) == (float) -1.0698434E38F);
};


void c_CommunicationChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    assert(p197_version_GET(pack) == (uint32_t)1382537809L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -9.537695E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)2.761047E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)3.3083626E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL));
    assert(p230_tas_ratio_GET(pack) == (float) -1.6931808E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -6.821018E37F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.160803E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)1102387136783656079L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)3.2512514E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.3948145E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_time_usec_GET(pack) == (uint64_t)5499659925661754800L);
    assert(p231_var_vert_GET(pack) == (float)2.1098241E38F);
    assert(p231_wind_x_GET(pack) == (float) -1.848325E38F);
    assert(p231_wind_alt_GET(pack) == (float) -9.070492E37F);
    assert(p231_vert_accuracy_GET(pack) == (float)5.395813E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float)1.7917647E38F);
    assert(p231_wind_y_GET(pack) == (float)1.7634478E38F);
    assert(p231_var_horiz_GET(pack) == (float)1.3086165E38F);
    assert(p231_wind_z_GET(pack) == (float) -1.5790453E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_hdop_GET(pack) == (float) -1.4453898E38F);
    assert(p232_speed_accuracy_GET(pack) == (float)1.5531745E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p232_lat_GET(pack) == (int32_t) -299430801);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
    assert(p232_horiz_accuracy_GET(pack) == (float) -2.697582E37F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)531651080L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p232_ve_GET(pack) == (float)5.8628424E37F);
    assert(p232_vdop_GET(pack) == (float) -1.2735507E38F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)34940);
    assert(p232_vert_accuracy_GET(pack) == (float) -2.5179407E38F);
    assert(p232_lon_GET(pack) == (int32_t)509311143);
    assert(p232_vd_GET(pack) == (float) -3.3100371E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)1379202724664413818L);
    assert(p232_vn_GET(pack) == (float) -3.3015844E38F);
    assert(p232_alt_GET(pack) == (float)2.2659952E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)244, (uint8_t)172, (uint8_t)135, (uint8_t)19, (uint8_t)122, (uint8_t)2, (uint8_t)183, (uint8_t)37, (uint8_t)116, (uint8_t)110, (uint8_t)200, (uint8_t)116, (uint8_t)116, (uint8_t)67, (uint8_t)219, (uint8_t)219, (uint8_t)28, (uint8_t)188, (uint8_t)31, (uint8_t)110, (uint8_t)208, (uint8_t)50, (uint8_t)86, (uint8_t)180, (uint8_t)57, (uint8_t)218, (uint8_t)60, (uint8_t)236, (uint8_t)21, (uint8_t)178, (uint8_t)199, (uint8_t)56, (uint8_t)214, (uint8_t)240, (uint8_t)1, (uint8_t)68, (uint8_t)229, (uint8_t)90, (uint8_t)183, (uint8_t)159, (uint8_t)7, (uint8_t)25, (uint8_t)173, (uint8_t)225, (uint8_t)218, (uint8_t)54, (uint8_t)101, (uint8_t)214, (uint8_t)224, (uint8_t)33, (uint8_t)159, (uint8_t)86, (uint8_t)31, (uint8_t)196, (uint8_t)169, (uint8_t)87, (uint8_t)11, (uint8_t)104, (uint8_t)219, (uint8_t)169, (uint8_t)223, (uint8_t)253, (uint8_t)161, (uint8_t)102, (uint8_t)55, (uint8_t)122, (uint8_t)172, (uint8_t)141, (uint8_t)241, (uint8_t)39, (uint8_t)242, (uint8_t)7, (uint8_t)161, (uint8_t)220, (uint8_t)251, (uint8_t)2, (uint8_t)145, (uint8_t)142, (uint8_t)99, (uint8_t)19, (uint8_t)56, (uint8_t)97, (uint8_t)116, (uint8_t)186, (uint8_t)136, (uint8_t)26, (uint8_t)17, (uint8_t)163, (uint8_t)91, (uint8_t)188, (uint8_t)3, (uint8_t)188, (uint8_t)242, (uint8_t)254, (uint8_t)126, (uint8_t)169, (uint8_t)202, (uint8_t)232, (uint8_t)73, (uint8_t)191, (uint8_t)66, (uint8_t)41, (uint8_t)43, (uint8_t)218, (uint8_t)33, (uint8_t)221, (uint8_t)90, (uint8_t)58, (uint8_t)18, (uint8_t)112, (uint8_t)98, (uint8_t)151, (uint8_t)166, (uint8_t)180, (uint8_t)214, (uint8_t)249, (uint8_t)161, (uint8_t)219, (uint8_t)149, (uint8_t)35, (uint8_t)49, (uint8_t)237, (uint8_t)3, (uint8_t)44, (uint8_t)211, (uint8_t)193, (uint8_t)23, (uint8_t)54, (uint8_t)111, (uint8_t)27, (uint8_t)75, (uint8_t)88, (uint8_t)98, (uint8_t)55, (uint8_t)223, (uint8_t)40, (uint8_t)187, (uint8_t)195, (uint8_t)175, (uint8_t)180, (uint8_t)139, (uint8_t)30, (uint8_t)84, (uint8_t)136, (uint8_t)64, (uint8_t)109, (uint8_t)43, (uint8_t)241, (uint8_t)227, (uint8_t)77, (uint8_t)109, (uint8_t)198, (uint8_t)237, (uint8_t)43, (uint8_t)185, (uint8_t)63, (uint8_t)65, (uint8_t)39, (uint8_t)34, (uint8_t)238, (uint8_t)147, (uint8_t)76, (uint8_t)166, (uint8_t)89, (uint8_t)42, (uint8_t)225, (uint8_t)179, (uint8_t)95, (uint8_t)194, (uint8_t)11, (uint8_t)241, (uint8_t)128, (uint8_t)145, (uint8_t)50, (uint8_t)235, (uint8_t)68, (uint8_t)159, (uint8_t)205, (uint8_t)88, (uint8_t)252} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)24745);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -68);
    assert(p234_custom_mode_GET(pack) == (uint32_t)2331271178L);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -22054);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)21216);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -22702);
    assert(p234_longitude_GET(pack) == (int32_t) -1540720433);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -26099);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p234_latitude_GET(pack) == (int32_t)443466467);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)57003);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -10);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)72);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED));
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)12);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)35885);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)363923887L);
    assert(p241_time_usec_GET(pack) == (uint64_t)102150766099705419L);
    assert(p241_vibration_y_GET(pack) == (float) -2.4024721E38F);
    assert(p241_vibration_x_GET(pack) == (float) -8.515018E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)338702951L);
    assert(p241_vibration_z_GET(pack) == (float)2.8215945E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2723536242L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-2.241348E38F, -1.4646941E38F, -1.554121E38F, 2.0830424E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t)1899541490);
    assert(p242_altitude_GET(pack) == (int32_t) -1231311054);
    assert(p242_z_GET(pack) == (float)1.3292455E38F);
    assert(p242_approach_z_GET(pack) == (float)2.6822118E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)2058490015811971295L);
    assert(p242_approach_x_GET(pack) == (float)3.192588E36F);
    assert(p242_y_GET(pack) == (float) -2.196132E38F);
    assert(p242_approach_y_GET(pack) == (float) -3.2049142E38F);
    assert(p242_x_GET(pack) == (float)6.9779435E37F);
    assert(p242_latitude_GET(pack) == (int32_t) -1529311154);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_x_GET(pack) == (float) -1.7285609E38F);
    assert(p243_approach_y_GET(pack) == (float)3.0097032E37F);
    assert(p243_y_GET(pack) == (float)1.7071341E38F);
    assert(p243_altitude_GET(pack) == (int32_t)700910554);
    assert(p243_longitude_GET(pack) == (int32_t) -246239054);
    assert(p243_approach_x_GET(pack) == (float) -1.9379621E38F);
    assert(p243_latitude_GET(pack) == (int32_t) -716059091);
    assert(p243_z_GET(pack) == (float)8.915454E37F);
    assert(p243_approach_z_GET(pack) == (float) -1.0257324E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p243_time_usec_TRY(ph) == (uint64_t)8412700632347070341L);
    {
        float exemplary[] =  {-1.6627198E38F, 4.6506455E37F, -4.6419905E37F, 2.2674176E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -1815403127);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)19708);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)14554);
    assert(p246_lat_GET(pack) == (int32_t)5219191);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                                    e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN));
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)20109);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -21430);
    assert(p246_lon_GET(pack) == (int32_t)1740151320);
    assert(p246_altitude_GET(pack) == (int32_t)1461243741);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)52172);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)352631844L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_callsign_LEN(ph) == 4);
    {
        char16_t * exemplary = u"qrxr";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)67);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)2501673281L);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -9.081996E37F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)3.3540406E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -1.3410168E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)90);
    {
        uint8_t exemplary[] =  {(uint8_t)44, (uint8_t)200, (uint8_t)95, (uint8_t)149, (uint8_t)170, (uint8_t)30, (uint8_t)172, (uint8_t)227, (uint8_t)19, (uint8_t)200, (uint8_t)69, (uint8_t)209, (uint8_t)205, (uint8_t)239, (uint8_t)25, (uint8_t)112, (uint8_t)64, (uint8_t)69, (uint8_t)220, (uint8_t)191, (uint8_t)185, (uint8_t)66, (uint8_t)58, (uint8_t)124, (uint8_t)66, (uint8_t)146, (uint8_t)106, (uint8_t)3, (uint8_t)173, (uint8_t)46, (uint8_t)75, (uint8_t)66, (uint8_t)50, (uint8_t)110, (uint8_t)69, (uint8_t)250, (uint8_t)66, (uint8_t)166, (uint8_t)211, (uint8_t)157, (uint8_t)60, (uint8_t)117, (uint8_t)102, (uint8_t)144, (uint8_t)55, (uint8_t)224, (uint8_t)209, (uint8_t)229, (uint8_t)15, (uint8_t)166, (uint8_t)209, (uint8_t)72, (uint8_t)46, (uint8_t)76, (uint8_t)164, (uint8_t)211, (uint8_t)60, (uint8_t)16, (uint8_t)7, (uint8_t)115, (uint8_t)186, (uint8_t)232, (uint8_t)146, (uint8_t)158, (uint8_t)117, (uint8_t)247, (uint8_t)18, (uint8_t)158, (uint8_t)140, (uint8_t)228, (uint8_t)204, (uint8_t)235, (uint8_t)151, (uint8_t)240, (uint8_t)218, (uint8_t)16, (uint8_t)103, (uint8_t)236, (uint8_t)119, (uint8_t)184, (uint8_t)137, (uint8_t)42, (uint8_t)146, (uint8_t)78, (uint8_t)80, (uint8_t)197, (uint8_t)30, (uint8_t)125, (uint8_t)224, (uint8_t)55, (uint8_t)167, (uint8_t)229, (uint8_t)188, (uint8_t)92, (uint8_t)33, (uint8_t)185, (uint8_t)19, (uint8_t)248, (uint8_t)61, (uint8_t)179, (uint8_t)186, (uint8_t)146, (uint8_t)78, (uint8_t)255, (uint8_t)152, (uint8_t)15, (uint8_t)215, (uint8_t)207, (uint8_t)199, (uint8_t)172, (uint8_t)69, (uint8_t)29, (uint8_t)198, (uint8_t)96, (uint8_t)62, (uint8_t)145, (uint8_t)87, (uint8_t)180, (uint8_t)94, (uint8_t)122, (uint8_t)3, (uint8_t)43, (uint8_t)84, (uint8_t)252, (uint8_t)134, (uint8_t)218, (uint8_t)188, (uint8_t)103, (uint8_t)205, (uint8_t)112, (uint8_t)219, (uint8_t)217, (uint8_t)10, (uint8_t)105, (uint8_t)162, (uint8_t)60, (uint8_t)138, (uint8_t)75, (uint8_t)171, (uint8_t)209, (uint8_t)9, (uint8_t)117, (uint8_t)155, (uint8_t)85, (uint8_t)142, (uint8_t)81, (uint8_t)38, (uint8_t)151, (uint8_t)90, (uint8_t)161, (uint8_t)85, (uint8_t)187, (uint8_t)241, (uint8_t)24, (uint8_t)245, (uint8_t)9, (uint8_t)52, (uint8_t)63, (uint8_t)94, (uint8_t)175, (uint8_t)90, (uint8_t)73, (uint8_t)77, (uint8_t)169, (uint8_t)6, (uint8_t)51, (uint8_t)212, (uint8_t)210, (uint8_t)75, (uint8_t)3, (uint8_t)29, (uint8_t)137, (uint8_t)66, (uint8_t)94, (uint8_t)125, (uint8_t)29, (uint8_t)76, (uint8_t)132, (uint8_t)3, (uint8_t)11, (uint8_t)163, (uint8_t)57, (uint8_t)151, (uint8_t)177, (uint8_t)60, (uint8_t)254, (uint8_t)148, (uint8_t)1, (uint8_t)7, (uint8_t)184, (uint8_t)95, (uint8_t)177, (uint8_t)240, (uint8_t)176, (uint8_t)68, (uint8_t)122, (uint8_t)249, (uint8_t)55, (uint8_t)145, (uint8_t)56, (uint8_t)132, (uint8_t)146, (uint8_t)140, (uint8_t)38, (uint8_t)43, (uint8_t)169, (uint8_t)125, (uint8_t)125, (uint8_t)216, (uint8_t)9, (uint8_t)68, (uint8_t)188, (uint8_t)91, (uint8_t)97, (uint8_t)35, (uint8_t)227, (uint8_t)96, (uint8_t)142, (uint8_t)74, (uint8_t)151, (uint8_t)72, (uint8_t)107, (uint8_t)6, (uint8_t)192, (uint8_t)72, (uint8_t)5, (uint8_t)201, (uint8_t)2, (uint8_t)31, (uint8_t)46, (uint8_t)223, (uint8_t)97, (uint8_t)233, (uint8_t)128, (uint8_t)178, (uint8_t)119, (uint8_t)138, (uint8_t)147, (uint8_t)241, (uint8_t)42, (uint8_t)72, (uint8_t)37, (uint8_t)26, (uint8_t)131, (uint8_t)126, (uint8_t)220, (uint8_t)254, (uint8_t)176, (uint8_t)26} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)22652);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)22052);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)255);
    {
        int8_t exemplary[] =  {(int8_t)17, (int8_t)111, (int8_t) -127, (int8_t)19, (int8_t) -81, (int8_t) -67, (int8_t) -53, (int8_t) -37, (int8_t)118, (int8_t)118, (int8_t)8, (int8_t)14, (int8_t) -16, (int8_t) -64, (int8_t) -44, (int8_t)7, (int8_t) -56, (int8_t)25, (int8_t) -26, (int8_t)89, (int8_t)43, (int8_t) -68, (int8_t)20, (int8_t) -109, (int8_t)44, (int8_t) -76, (int8_t)47, (int8_t) -79, (int8_t)51, (int8_t)59, (int8_t) -115, (int8_t) -114} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_z_GET(pack) == (float)2.6624003E38F);
    assert(p250_x_GET(pack) == (float) -1.2869257E38F);
    assert(p250_y_GET(pack) == (float) -2.734662E38F);
    assert(p250_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"voxxlO";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)6636738375043034924L);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)1451395895L);
    assert(p251_value_GET(pack) == (float)2.9730526E38F);
    assert(p251_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"cqxmu";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1987941009L);
    assert(p252_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"p";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_value_GET(pack) == (int32_t)1836418142);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 45);
    {
        char16_t * exemplary = u"yqunhyhavikspzUcktdeBmzVcPkeqlYHujcgqtlqiczky";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_ALERT);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)13700161L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p254_value_GET(pack) == (float) -2.3185873E38F);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)7069931489992666373L);
    {
        uint8_t exemplary[] =  {(uint8_t)81, (uint8_t)164, (uint8_t)31, (uint8_t)163, (uint8_t)239, (uint8_t)1, (uint8_t)204, (uint8_t)17, (uint8_t)104, (uint8_t)7, (uint8_t)94, (uint8_t)20, (uint8_t)201, (uint8_t)36, (uint8_t)36, (uint8_t)178, (uint8_t)156, (uint8_t)212, (uint8_t)46, (uint8_t)74, (uint8_t)92, (uint8_t)37, (uint8_t)8, (uint8_t)220, (uint8_t)28, (uint8_t)24, (uint8_t)194, (uint8_t)102, (uint8_t)194, (uint8_t)143, (uint8_t)107, (uint8_t)62} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)95);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1517509949L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)248432746L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 27);
    {
        char16_t * exemplary = u"iSgokgdivoqizsvUwrbckaxdear";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 54);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_v_GET(pack) == (float)1.5449298E38F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)57112);
    assert(p259_sensor_size_h_GET(pack) == (float)2.1811795E37F);
    assert(p259_cam_definition_uri_LEN(ph) == 76);
    {
        char16_t * exemplary = u"bjSirfjyKfcxljqyonqmtSsoezoxtbpzdyjhvzgrnsnkKHgtpOzpqtrpknlyfMywKbwueebcdYlh";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 152);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)33424);
    assert(p259_focal_length_GET(pack) == (float) -3.3786545E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)98, (uint8_t)145, (uint8_t)178, (uint8_t)196, (uint8_t)157, (uint8_t)34, (uint8_t)59, (uint8_t)240, (uint8_t)181, (uint8_t)206, (uint8_t)42, (uint8_t)70, (uint8_t)58, (uint8_t)80, (uint8_t)206, (uint8_t)205, (uint8_t)27, (uint8_t)21, (uint8_t)20, (uint8_t)66, (uint8_t)54, (uint8_t)204, (uint8_t)138, (uint8_t)202, (uint8_t)100, (uint8_t)73, (uint8_t)178, (uint8_t)32, (uint8_t)168, (uint8_t)87, (uint8_t)116, (uint8_t)6} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)28, (uint8_t)130, (uint8_t)16, (uint8_t)48, (uint8_t)224, (uint8_t)69, (uint8_t)242, (uint8_t)28, (uint8_t)84, (uint8_t)111, (uint8_t)79, (uint8_t)33, (uint8_t)28, (uint8_t)127, (uint8_t)144, (uint8_t)35, (uint8_t)51, (uint8_t)0, (uint8_t)226, (uint8_t)237, (uint8_t)117, (uint8_t)104, (uint8_t)98, (uint8_t)53, (uint8_t)134, (uint8_t)159, (uint8_t)82, (uint8_t)6, (uint8_t)160, (uint8_t)8, (uint8_t)165, (uint8_t)171} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p259_firmware_version_GET(pack) == (uint32_t)2397919213L);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES));
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1984102429L);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)58094);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)361653200L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_available_capacity_GET(pack) == (float) -1.3453559E37F);
    assert(p261_write_speed_GET(pack) == (float)6.554171E37F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p261_used_capacity_GET(pack) == (float)1.3585224E38F);
    assert(p261_read_speed_GET(pack) == (float) -1.7810572E38F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p261_total_capacity_GET(pack) == (float)1.1795783E37F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2664488226L);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p262_available_capacity_GET(pack) == (float)2.9173583E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2571320694L);
    assert(p262_image_interval_GET(pack) == (float) -2.1807178E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)4260244672L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_time_utc_GET(pack) == (uint64_t)6264861376725511357L);
    assert(p263_relative_alt_GET(pack) == (int32_t) -872125824);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)884502443L);
    assert(p263_lon_GET(pack) == (int32_t) -939901184);
    assert(p263_image_index_GET(pack) == (int32_t) -181579124);
    assert(p263_alt_GET(pack) == (int32_t) -2070631102);
    assert(p263_lat_GET(pack) == (int32_t) -1641479557);
    assert(p263_file_url_LEN(ph) == 165);
    {
        char16_t * exemplary = u"npTyvAtqjtrqchqypbweinzMdjTuyRFivfrbkzinvofGejwzrwpgqjhHurhOevcSPqjQsnxdjcocmopkkVlwtqLrcswxichnttaixqigjnBvmjgvyraEncpMwlcqprkdsjnsgyBhaixownjhbyygovXfjznfMicleprhr";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 330);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -80);
    {
        float exemplary[] =  {9.043213E36F, 9.618361E37F, 2.504039E38F, -5.3613477E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)8297060963220899421L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3250446628L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)8723619008563835978L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)393866942971931543L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float) -1.1503992E38F);
    assert(p265_pitch_GET(pack) == (float) -2.5912871E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1968370552L);
    assert(p265_roll_GET(pack) == (float)1.9176168E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)49);
    {
        uint8_t exemplary[] =  {(uint8_t)60, (uint8_t)20, (uint8_t)192, (uint8_t)120, (uint8_t)58, (uint8_t)38, (uint8_t)63, (uint8_t)74, (uint8_t)111, (uint8_t)42, (uint8_t)244, (uint8_t)96, (uint8_t)99, (uint8_t)130, (uint8_t)82, (uint8_t)52, (uint8_t)75, (uint8_t)129, (uint8_t)65, (uint8_t)200, (uint8_t)201, (uint8_t)24, (uint8_t)193, (uint8_t)94, (uint8_t)132, (uint8_t)230, (uint8_t)39, (uint8_t)175, (uint8_t)49, (uint8_t)142, (uint8_t)153, (uint8_t)203, (uint8_t)73, (uint8_t)8, (uint8_t)228, (uint8_t)204, (uint8_t)24, (uint8_t)182, (uint8_t)23, (uint8_t)67, (uint8_t)16, (uint8_t)158, (uint8_t)236, (uint8_t)22, (uint8_t)222, (uint8_t)115, (uint8_t)214, (uint8_t)73, (uint8_t)252, (uint8_t)180, (uint8_t)190, (uint8_t)232, (uint8_t)25, (uint8_t)166, (uint8_t)238, (uint8_t)189, (uint8_t)234, (uint8_t)58, (uint8_t)98, (uint8_t)45, (uint8_t)53, (uint8_t)175, (uint8_t)119, (uint8_t)207, (uint8_t)25, (uint8_t)191, (uint8_t)196, (uint8_t)75, (uint8_t)232, (uint8_t)249, (uint8_t)159, (uint8_t)75, (uint8_t)150, (uint8_t)104, (uint8_t)34, (uint8_t)246, (uint8_t)71, (uint8_t)92, (uint8_t)164, (uint8_t)54, (uint8_t)92, (uint8_t)71, (uint8_t)98, (uint8_t)61, (uint8_t)159, (uint8_t)130, (uint8_t)125, (uint8_t)192, (uint8_t)73, (uint8_t)97, (uint8_t)77, (uint8_t)136, (uint8_t)110, (uint8_t)185, (uint8_t)202, (uint8_t)108, (uint8_t)142, (uint8_t)51, (uint8_t)123, (uint8_t)156, (uint8_t)158, (uint8_t)142, (uint8_t)178, (uint8_t)55, (uint8_t)121, (uint8_t)228, (uint8_t)170, (uint8_t)248, (uint8_t)255, (uint8_t)198, (uint8_t)129, (uint8_t)31, (uint8_t)32, (uint8_t)43, (uint8_t)76, (uint8_t)108, (uint8_t)131, (uint8_t)142, (uint8_t)172, (uint8_t)206, (uint8_t)14, (uint8_t)174, (uint8_t)126, (uint8_t)58, (uint8_t)143, (uint8_t)61, (uint8_t)26, (uint8_t)102, (uint8_t)100, (uint8_t)207, (uint8_t)248, (uint8_t)190, (uint8_t)2, (uint8_t)111, (uint8_t)182, (uint8_t)82, (uint8_t)185, (uint8_t)205, (uint8_t)209, (uint8_t)75, (uint8_t)250, (uint8_t)237, (uint8_t)116, (uint8_t)75, (uint8_t)175, (uint8_t)19, (uint8_t)251, (uint8_t)58, (uint8_t)106, (uint8_t)148, (uint8_t)13, (uint8_t)211, (uint8_t)173, (uint8_t)3, (uint8_t)161, (uint8_t)229, (uint8_t)202, (uint8_t)183, (uint8_t)133, (uint8_t)147, (uint8_t)95, (uint8_t)225, (uint8_t)251, (uint8_t)240, (uint8_t)245, (uint8_t)161, (uint8_t)54, (uint8_t)1, (uint8_t)16, (uint8_t)4, (uint8_t)209, (uint8_t)222, (uint8_t)174, (uint8_t)31, (uint8_t)248, (uint8_t)48, (uint8_t)188, (uint8_t)59, (uint8_t)49, (uint8_t)113, (uint8_t)159, (uint8_t)27, (uint8_t)34, (uint8_t)26, (uint8_t)250, (uint8_t)28, (uint8_t)119, (uint8_t)157, (uint8_t)174, (uint8_t)247, (uint8_t)24, (uint8_t)34, (uint8_t)165, (uint8_t)44, (uint8_t)0, (uint8_t)18, (uint8_t)12, (uint8_t)181, (uint8_t)88, (uint8_t)61, (uint8_t)116, (uint8_t)81, (uint8_t)145, (uint8_t)30, (uint8_t)89, (uint8_t)205, (uint8_t)98, (uint8_t)15, (uint8_t)24, (uint8_t)82, (uint8_t)206, (uint8_t)172, (uint8_t)227, (uint8_t)66, (uint8_t)96, (uint8_t)77, (uint8_t)58, (uint8_t)162, (uint8_t)83, (uint8_t)213, (uint8_t)179, (uint8_t)170, (uint8_t)38, (uint8_t)155, (uint8_t)42, (uint8_t)60, (uint8_t)173, (uint8_t)133, (uint8_t)71, (uint8_t)173, (uint8_t)86, (uint8_t)18, (uint8_t)250, (uint8_t)143, (uint8_t)69, (uint8_t)119, (uint8_t)206, (uint8_t)198, (uint8_t)189, (uint8_t)156, (uint8_t)135, (uint8_t)7, (uint8_t)167, (uint8_t)58, (uint8_t)224, (uint8_t)86, (uint8_t)160, (uint8_t)75, (uint8_t)205} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)50732);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)195);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)16880);
    {
        uint8_t exemplary[] =  {(uint8_t)207, (uint8_t)141, (uint8_t)161, (uint8_t)64, (uint8_t)170, (uint8_t)138, (uint8_t)10, (uint8_t)120, (uint8_t)217, (uint8_t)91, (uint8_t)114, (uint8_t)102, (uint8_t)241, (uint8_t)58, (uint8_t)69, (uint8_t)194, (uint8_t)246, (uint8_t)77, (uint8_t)36, (uint8_t)186, (uint8_t)43, (uint8_t)4, (uint8_t)197, (uint8_t)147, (uint8_t)118, (uint8_t)189, (uint8_t)38, (uint8_t)52, (uint8_t)48, (uint8_t)126, (uint8_t)111, (uint8_t)76, (uint8_t)204, (uint8_t)96, (uint8_t)226, (uint8_t)159, (uint8_t)212, (uint8_t)100, (uint8_t)47, (uint8_t)140, (uint8_t)224, (uint8_t)23, (uint8_t)232, (uint8_t)70, (uint8_t)150, (uint8_t)6, (uint8_t)122, (uint8_t)213, (uint8_t)224, (uint8_t)53, (uint8_t)230, (uint8_t)74, (uint8_t)48, (uint8_t)245, (uint8_t)16, (uint8_t)26, (uint8_t)77, (uint8_t)4, (uint8_t)147, (uint8_t)67, (uint8_t)151, (uint8_t)197, (uint8_t)121, (uint8_t)117, (uint8_t)44, (uint8_t)127, (uint8_t)39, (uint8_t)200, (uint8_t)203, (uint8_t)237, (uint8_t)234, (uint8_t)186, (uint8_t)172, (uint8_t)239, (uint8_t)206, (uint8_t)186, (uint8_t)135, (uint8_t)190, (uint8_t)177, (uint8_t)135, (uint8_t)195, (uint8_t)245, (uint8_t)53, (uint8_t)250, (uint8_t)79, (uint8_t)221, (uint8_t)93, (uint8_t)148, (uint8_t)19, (uint8_t)156, (uint8_t)94, (uint8_t)104, (uint8_t)114, (uint8_t)233, (uint8_t)9, (uint8_t)212, (uint8_t)75, (uint8_t)245, (uint8_t)199, (uint8_t)255, (uint8_t)165, (uint8_t)240, (uint8_t)143, (uint8_t)155, (uint8_t)244, (uint8_t)153, (uint8_t)92, (uint8_t)102, (uint8_t)144, (uint8_t)135, (uint8_t)227, (uint8_t)72, (uint8_t)142, (uint8_t)43, (uint8_t)2, (uint8_t)225, (uint8_t)48, (uint8_t)66, (uint8_t)228, (uint8_t)105, (uint8_t)74, (uint8_t)189, (uint8_t)133, (uint8_t)35, (uint8_t)133, (uint8_t)213, (uint8_t)207, (uint8_t)176, (uint8_t)1, (uint8_t)47, (uint8_t)158, (uint8_t)144, (uint8_t)149, (uint8_t)44, (uint8_t)170, (uint8_t)75, (uint8_t)214, (uint8_t)232, (uint8_t)234, (uint8_t)40, (uint8_t)159, (uint8_t)21, (uint8_t)184, (uint8_t)68, (uint8_t)246, (uint8_t)64, (uint8_t)58, (uint8_t)126, (uint8_t)166, (uint8_t)76, (uint8_t)161, (uint8_t)205, (uint8_t)37, (uint8_t)62, (uint8_t)167, (uint8_t)30, (uint8_t)89, (uint8_t)116, (uint8_t)165, (uint8_t)251, (uint8_t)211, (uint8_t)150, (uint8_t)204, (uint8_t)149, (uint8_t)155, (uint8_t)66, (uint8_t)240, (uint8_t)42, (uint8_t)245, (uint8_t)25, (uint8_t)52, (uint8_t)243, (uint8_t)178, (uint8_t)183, (uint8_t)75, (uint8_t)63, (uint8_t)230, (uint8_t)89, (uint8_t)174, (uint8_t)5, (uint8_t)242, (uint8_t)50, (uint8_t)22, (uint8_t)78, (uint8_t)155, (uint8_t)190, (uint8_t)193, (uint8_t)89, (uint8_t)159, (uint8_t)135, (uint8_t)156, (uint8_t)146, (uint8_t)227, (uint8_t)136, (uint8_t)212, (uint8_t)163, (uint8_t)248, (uint8_t)5, (uint8_t)6, (uint8_t)124, (uint8_t)157, (uint8_t)13, (uint8_t)151, (uint8_t)247, (uint8_t)59, (uint8_t)84, (uint8_t)154, (uint8_t)176, (uint8_t)179, (uint8_t)164, (uint8_t)226, (uint8_t)207, (uint8_t)228, (uint8_t)71, (uint8_t)109, (uint8_t)77, (uint8_t)164, (uint8_t)60, (uint8_t)162, (uint8_t)69, (uint8_t)238, (uint8_t)29, (uint8_t)34, (uint8_t)9, (uint8_t)241, (uint8_t)129, (uint8_t)110, (uint8_t)194, (uint8_t)36, (uint8_t)47, (uint8_t)104, (uint8_t)66, (uint8_t)124, (uint8_t)61, (uint8_t)217, (uint8_t)202, (uint8_t)208, (uint8_t)16, (uint8_t)254, (uint8_t)211, (uint8_t)49, (uint8_t)42, (uint8_t)75, (uint8_t)254, (uint8_t)255, (uint8_t)55, (uint8_t)25, (uint8_t)195, (uint8_t)165} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)6480);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_uri_LEN(ph) == 123);
    {
        char16_t * exemplary = u"ndmvqiqbkwzvfudveShcqsgdufQltdrgpgyhetetontkdpwtptnxikzkBkewoudadruyzfzmoishnoqggjispmRbuaswkzUrmxayrqfQntqgxhqdpwnQfuqfbsd";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 246);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float) -2.8526972E38F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)54097);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)1315);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)1556);
    assert(p269_bitrate_GET(pack) == (uint32_t)1760251105L);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)37532);
    assert(p270_bitrate_GET(pack) == (uint32_t)534572900L);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)18379);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)22842);
    assert(p270_framerate_GET(pack) == (float)3.6937987E37F);
    assert(p270_uri_LEN(ph) == 71);
    {
        char16_t * exemplary = u"LWwtbOapescxrqxHyuydcdzikoifeukxjknorpkOxgvdLFnsfcoaoeptcpyGeokuurwAmyq";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 142);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 29);
    {
        char16_t * exemplary = u"fuqcphazqamGgggeeaeitiAovvhsv";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 58);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 15);
    {
        char16_t * exemplary = u"czbomrilwzSfbKV";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)139, (uint8_t)183, (uint8_t)49, (uint8_t)246, (uint8_t)18, (uint8_t)115, (uint8_t)60} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)26561);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)34863);
    {
        uint8_t exemplary[] =  {(uint8_t)210, (uint8_t)141, (uint8_t)172, (uint8_t)219, (uint8_t)67, (uint8_t)12, (uint8_t)161, (uint8_t)147} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)28061);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1217254271L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)16592);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING);
    assert(p310_time_usec_GET(pack) == (uint64_t)5333500663778041370L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2600777009L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)745403811L);
    assert(p311_time_usec_GET(pack) == (uint64_t)615327438749491127L);
    {
        uint8_t exemplary[] =  {(uint8_t)203, (uint8_t)20, (uint8_t)2, (uint8_t)223, (uint8_t)93, (uint8_t)26, (uint8_t)62, (uint8_t)81, (uint8_t)138, (uint8_t)43, (uint8_t)143, (uint8_t)156, (uint8_t)120, (uint8_t)62, (uint8_t)147, (uint8_t)98} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_name_LEN(ph) == 55);
    {
        char16_t * exemplary = u"ljowpkuSqqsgqgbwujnmjcndxfffqfpMcTeammYgwFMskxxaJefljhV";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)28639);
    assert(p320_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"vuveuhzn";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)13);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)36817);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)12828);
    assert(p322_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"vweykdrgsS";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p322_param_value_LEN(ph) == 71);
    {
        char16_t * exemplary = u"nemaydqdjceVqRzwfalffcXgaxiqihiswfwabmptfsmtqnfouCydiuXsbjmVvcvwfkfytpD";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 142);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"dvhzgtoeaowpl";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_value_LEN(ph) == 90);
    {
        char16_t * exemplary = u"negybpKaUsizrabggfnCyprwexrcmcjuvbbikzjbylosqqILuhfkvwztogSnrlfubzDjaerxAcxlcvyimuwhSonPza";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
    assert(p324_param_value_LEN(ph) == 55);
    {
        char16_t * exemplary = u"RDijgcmtptuzMmjcemfyorfjyresfjmfdkpREckronxexbtTelvupdj";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"gdmxycw";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)36144);
    {
        uint16_t exemplary[] =  {(uint16_t)35547, (uint16_t)16697, (uint16_t)17731, (uint16_t)48042, (uint16_t)64701, (uint16_t)4580, (uint16_t)55232, (uint16_t)65160, (uint16_t)49744, (uint16_t)59657, (uint16_t)32268, (uint16_t)25632, (uint16_t)17206, (uint16_t)62478, (uint16_t)24094, (uint16_t)45392, (uint16_t)54233, (uint16_t)31202, (uint16_t)44901, (uint16_t)21861, (uint16_t)29632, (uint16_t)30207, (uint16_t)50913, (uint16_t)72, (uint16_t)34364, (uint16_t)60588, (uint16_t)45897, (uint16_t)38773, (uint16_t)50260, (uint16_t)2699, (uint16_t)55723, (uint16_t)53582, (uint16_t)52249, (uint16_t)36371, (uint16_t)31192, (uint16_t)10840, (uint16_t)11093, (uint16_t)21960, (uint16_t)22371, (uint16_t)58154, (uint16_t)12591, (uint16_t)14220, (uint16_t)19339, (uint16_t)21994, (uint16_t)6727, (uint16_t)38994, (uint16_t)37678, (uint16_t)60140, (uint16_t)44239, (uint16_t)61270, (uint16_t)4398, (uint16_t)7985, (uint16_t)63764, (uint16_t)38638, (uint16_t)40058, (uint16_t)45021, (uint16_t)1288, (uint16_t)33708, (uint16_t)37447, (uint16_t)43715, (uint16_t)28880, (uint16_t)25526, (uint16_t)463, (uint16_t)54819, (uint16_t)31382, (uint16_t)50787, (uint16_t)47446, (uint16_t)39182, (uint16_t)12233, (uint16_t)15493, (uint16_t)11132, (uint16_t)4287} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)42708);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p330_time_usec_GET(pack) == (uint64_t)7682875618822851657L);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)64);
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
            case 82:
                if(pack == NULL) return c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82();
                c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&ph, pack);
                break;
            case 83:
                if(pack == NULL) return c_CommunicationChannel_new_ATTITUDE_TARGET_83();
                c_TEST_Channel_on_ATTITUDE_TARGET_83(&ph, pack);
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
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_ARMAZILA, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)3719103892L, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_GENERIC, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_voltage_battery_SET((uint16_t)(uint16_t)28907, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)57778, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -76, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)34508, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)42948, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)41157, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)49779, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)10537, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -11320, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)36982, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2387949876L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3725336867496810487L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float) -2.9239118E38F, PH.base.pack) ;
        p3_yaw_SET((float) -3.7749268E37F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p3_vy_SET((float) -3.0020945E36F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)11983, PH.base.pack) ;
        p3_afy_SET((float)2.9061957E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3689037570L, PH.base.pack) ;
        p3_x_SET((float)3.838724E37F, PH.base.pack) ;
        p3_afz_SET((float)1.3899332E37F, PH.base.pack) ;
        p3_vx_SET((float)8.391138E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -8.210629E37F, PH.base.pack) ;
        p3_z_SET((float) -1.6902929E38F, PH.base.pack) ;
        p3_y_SET((float) -2.307896E37F, PH.base.pack) ;
        p3_afx_SET((float) -2.0581583E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)4472129752107593931L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p4_seq_SET((uint32_t)4033634906L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        {
            char16_t* passkey = u"k";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_gcs_system_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"Akj";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)358387682L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t) -24818, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        {
            char16_t* param_id = u"ddCwhyrjgm";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"i";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)35125, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)36718, PH.base.pack) ;
        p22_param_value_SET((float)1.0396985E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p23_param_value_SET((float) -6.8265094E37F, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
        {
            char16_t* param_id = u"yLniIwdh";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_time_usec_SET((uint64_t)2833712201556788843L, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)4148086756L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p24_alt_SET((int32_t) -488137837, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)220772083, &PH) ;
        p24_v_acc_SET((uint32_t)1352751066L, &PH) ;
        p24_lat_SET((int32_t) -579352706, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)50401, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)2315055921L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)15267, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)498376605L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)33701, PH.base.pack) ;
        p24_lon_SET((int32_t)681530387, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)3091, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)250, (uint8_t)176, (uint8_t)192, (uint8_t)113, (uint8_t)104, (uint8_t)19, (uint8_t)75, (uint8_t)54, (uint8_t)118, (uint8_t)88, (uint8_t)132, (uint8_t)2, (uint8_t)190, (uint8_t)65, (uint8_t)100, (uint8_t)53, (uint8_t)20, (uint8_t)49, (uint8_t)145, (uint8_t)105};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        {
            uint8_t satellite_used[] =  {(uint8_t)87, (uint8_t)228, (uint8_t)54, (uint8_t)121, (uint8_t)60, (uint8_t)137, (uint8_t)226, (uint8_t)203, (uint8_t)150, (uint8_t)162, (uint8_t)12, (uint8_t)106, (uint8_t)113, (uint8_t)214, (uint8_t)78, (uint8_t)70, (uint8_t)47, (uint8_t)192, (uint8_t)177, (uint8_t)96};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)9, (uint8_t)102, (uint8_t)140, (uint8_t)182, (uint8_t)203, (uint8_t)71, (uint8_t)143, (uint8_t)91, (uint8_t)54, (uint8_t)95, (uint8_t)126, (uint8_t)237, (uint8_t)195, (uint8_t)73, (uint8_t)54, (uint8_t)128, (uint8_t)136, (uint8_t)120, (uint8_t)235, (uint8_t)221};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)63, (uint8_t)14, (uint8_t)13, (uint8_t)172, (uint8_t)237, (uint8_t)133, (uint8_t)19, (uint8_t)217, (uint8_t)102, (uint8_t)108, (uint8_t)20, (uint8_t)143, (uint8_t)117, (uint8_t)195, (uint8_t)103, (uint8_t)202, (uint8_t)194, (uint8_t)236, (uint8_t)243, (uint8_t)76};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)133, (uint8_t)40, (uint8_t)72, (uint8_t)50, (uint8_t)246, (uint8_t)125, (uint8_t)27, (uint8_t)30, (uint8_t)249, (uint8_t)52, (uint8_t)171, (uint8_t)17, (uint8_t)167, (uint8_t)159, (uint8_t)190, (uint8_t)158, (uint8_t)49, (uint8_t)139, (uint8_t)227, (uint8_t)84};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_yacc_SET((int16_t)(int16_t) -29305, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -4400, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -892, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)21646, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -9626, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)28488, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -30427, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)24669, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)2891374336L, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)24115, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_ymag_SET((int16_t)(int16_t)32159, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -32713, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -24, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)8693, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)18372, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)1588602573255574940L, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)32020, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)22989, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -15397, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)17429, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -3838, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)13230, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)6828, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)4029815571497298959L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -31053, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)24685, PH.base.pack) ;
        p29_press_diff_SET((float)1.2974039E38F, PH.base.pack) ;
        p29_press_abs_SET((float)7.254957E37F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1079804440L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_rollspeed_SET((float) -3.1649244E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)9.53682E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2531410322L, PH.base.pack) ;
        p30_yawspeed_SET((float)1.6061913E38F, PH.base.pack) ;
        p30_pitch_SET((float) -2.2385716E38F, PH.base.pack) ;
        p30_roll_SET((float) -2.4040507E38F, PH.base.pack) ;
        p30_yaw_SET((float)1.3333056E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q4_SET((float)4.566466E37F, PH.base.pack) ;
        p31_q3_SET((float) -7.1545556E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)3.2549775E38F, PH.base.pack) ;
        p31_yawspeed_SET((float)2.0395575E38F, PH.base.pack) ;
        p31_q2_SET((float)2.2624026E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)2.334941E38F, PH.base.pack) ;
        p31_q1_SET((float)1.7044007E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)172059461L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vy_SET((float)1.9499086E37F, PH.base.pack) ;
        p32_z_SET((float) -3.284545E38F, PH.base.pack) ;
        p32_x_SET((float)1.0028484E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1011204102L, PH.base.pack) ;
        p32_y_SET((float)7.033853E37F, PH.base.pack) ;
        p32_vz_SET((float) -1.1325224E38F, PH.base.pack) ;
        p32_vx_SET((float)8.553617E37F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_vx_SET((int16_t)(int16_t)30398, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)30838, PH.base.pack) ;
        p33_lat_SET((int32_t)1203126958, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -2144769742, PH.base.pack) ;
        p33_alt_SET((int32_t)2019801833, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)2630871604L, PH.base.pack) ;
        p33_lon_SET((int32_t) -2128025485, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)51643, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -2068, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan1_scaled_SET((int16_t)(int16_t) -30990, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2140927487L, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -8691, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)23393, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)19595, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)16958, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -32004, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -26798, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)32230, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan5_raw_SET((uint16_t)(uint16_t)11384, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)56032, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)59099, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)6649, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)51537437L, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)2575, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)31225, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)52725, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)55152, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo9_raw_SET((uint16_t)(uint16_t)21713, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)41624, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)60512, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)61653, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)36930, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)17379, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)41218, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)60390, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)28839, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)25568, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)57212, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)36152, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)18696, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)4160231113L, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)36381, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)28149, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)4112, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -8021, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)6583, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -15546, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)5378, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_y_SET((float)1.9739407E38F, PH.base.pack) ;
        p39_param1_SET((float) -2.8534981E38F, PH.base.pack) ;
        p39_param4_SET((float) -1.2890089E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)20358, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p39_param3_SET((float) -2.7255285E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p39_z_SET((float) -2.6798888E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p39_x_SET((float)2.4497228E38F, PH.base.pack) ;
        p39_param2_SET((float)1.5383006E37F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)18441, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)24132, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)60427, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)43783, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)50966, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)8146539498836834588L, &PH) ;
        p48_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p48_altitude_SET((int32_t)1734322675, PH.base.pack) ;
        p48_longitude_SET((int32_t)421703150, PH.base.pack) ;
        p48_latitude_SET((int32_t) -593619954, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)4333303715668388355L, &PH) ;
        p49_longitude_SET((int32_t)289700153, PH.base.pack) ;
        p49_altitude_SET((int32_t)1517581668, PH.base.pack) ;
        p49_latitude_SET((int32_t) -914710276, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_index_SET((int16_t)(int16_t)18914, PH.base.pack) ;
        {
            char16_t* param_id = u"ckfomkctv";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p50_param_value0_SET((float) -3.3265752E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -4.984017E37F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p50_scale_SET((float)1.7068123E37F, PH.base.pack) ;
        p50_param_value_min_SET((float)1.5954541E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)28167, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p54_p2x_SET((float)1.2715543E38F, PH.base.pack) ;
        p54_p2z_SET((float) -9.668556E37F, PH.base.pack) ;
        p54_p1y_SET((float)8.625633E37F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p54_p2y_SET((float) -2.7555045E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p54_p1z_SET((float) -7.8805847E37F, PH.base.pack) ;
        p54_p1x_SET((float) -2.0706393E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1x_SET((float) -5.152813E37F, PH.base.pack) ;
        p55_p2z_SET((float)5.302376E37F, PH.base.pack) ;
        p55_p1z_SET((float)1.8454202E38F, PH.base.pack) ;
        p55_p2y_SET((float) -1.0428094E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p55_p2x_SET((float) -7.8461067E37F, PH.base.pack) ;
        p55_p1y_SET((float) -1.9432195E37F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_pitchspeed_SET((float) -1.945015E38F, PH.base.pack) ;
        p61_yawspeed_SET((float) -2.6474682E38F, PH.base.pack) ;
        {
            float q[] =  {-4.6551066E37F, 1.841075E38F, 6.941192E37F, 3.276021E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)5066805255881247041L, PH.base.pack) ;
        {
            float covariance[] =  {-1.4194383E38F, 1.4848559E38F, 9.522011E37F, 3.3635226E38F, -7.0221003E37F, 1.0776621E38F, 3.2409662E38F, 9.75316E37F, 2.9010131E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)8.435971E36F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_roll_SET((float) -1.6099812E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -11323, PH.base.pack) ;
        p62_alt_error_SET((float)1.5340445E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -3.1460175E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)24030, PH.base.pack) ;
        p62_aspd_error_SET((float) -3.2717275E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)22895, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.4800661E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lon_SET((int32_t)2116211021, PH.base.pack) ;
        p63_vy_SET((float) -2.0846291E37F, PH.base.pack) ;
        p63_vz_SET((float)2.0329903E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)8286766569434826014L, PH.base.pack) ;
        {
            float covariance[] =  {-1.1831602E38F, -2.4988223E38F, -3.6336779E37F, 2.6564572E38F, -4.5433115E37F, -1.5335536E38F, -1.7733213E38F, 1.6355652E38F, 2.6020445E38F, 2.4276087E37F, 4.8087976E37F, 2.7801853E38F, -2.1592935E38F, -2.4115243E38F, 1.3657578E38F, 2.3471015E38F, 1.194695E38F, -2.3611595E38F, 4.1540084E37F, 1.483012E38F, -3.017619E37F, -9.084857E37F, -5.860799E37F, 1.916093E37F, -1.2141741E38F, 7.1696746E36F, -2.0136236E38F, -2.3617542E38F, -1.959452E38F, 1.5492882E38F, 1.6430326E37F, 2.437152E38F, -2.361568E38F, 2.0557498E38F, -2.8419007E38F, 2.5946887E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_alt_SET((int32_t) -979911915, PH.base.pack) ;
        p63_lat_SET((int32_t)1182524535, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1906468442, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_vx_SET((float)1.0961677E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_z_SET((float) -3.8922238E37F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)6981860961411528392L, PH.base.pack) ;
        p64_ay_SET((float) -1.8123822E38F, PH.base.pack) ;
        p64_vx_SET((float) -1.1086005E38F, PH.base.pack) ;
        p64_vz_SET((float) -1.8783617E38F, PH.base.pack) ;
        p64_ax_SET((float)1.5904215E38F, PH.base.pack) ;
        p64_x_SET((float) -2.9162785E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p64_az_SET((float) -3.5485364E37F, PH.base.pack) ;
        p64_vy_SET((float)1.0726692E38F, PH.base.pack) ;
        {
            float covariance[] =  {3.3166539E38F, -1.8977024E38F, 2.9289085E38F, -2.321433E38F, -1.3812012E38F, -1.1033297E37F, -1.6461832E38F, 3.9590544E37F, 4.8606587E37F, -2.5097064E38F, -2.6541164E38F, 1.116026E38F, -1.5841765E37F, 1.944132E38F, -1.7226147E38F, -1.791416E38F, 5.1104127E37F, 3.3856885E38F, -1.4527263E38F, 4.4977617E37F, 2.5039152E38F, -7.922659E37F, -3.180821E38F, -6.487857E37F, -6.0821115E37F, -1.5362652E38F, -7.283622E37F, -7.3133165E36F, 2.4728685E38F, 2.91134E38F, -2.5133544E38F, -2.7240676E38F, -7.560125E37F, -6.496929E37F, 3.2478496E38F, -7.247916E37F, 1.9309503E38F, 1.3519073E38F, -1.9343085E38F, 1.3657355E38F, -1.1997834E38F, -1.7771954E38F, -3.372647E38F, -1.5452897E38F, 2.4074796E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float)2.8792682E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan14_raw_SET((uint16_t)(uint16_t)25426, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)3363, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)16063, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)740, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)35376, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)61516, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)58387, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)147355654L, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)13700, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)35606, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)26677, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)8009, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)1129, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)56591, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)61142, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)48346, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)46933, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)2994, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)49777, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)47836, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)19628, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t) -12928, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -6962, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)6582, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)32509, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)3990, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_target_component_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)19052, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)11236, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)46826, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)22820, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)27697, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)54898, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)28946, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)14992, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_current_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p73_x_SET((int32_t) -793747902, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)58398, PH.base.pack) ;
        p73_param4_SET((float)3.26239E38F, PH.base.pack) ;
        p73_y_SET((int32_t)2124863764, PH.base.pack) ;
        p73_z_SET((float) -1.1412985E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_RELAY, PH.base.pack) ;
        p73_param1_SET((float) -5.423102E37F, PH.base.pack) ;
        p73_param3_SET((float)9.372113E37F, PH.base.pack) ;
        p73_param2_SET((float) -6.384359E37F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float) -9.178037E37F, PH.base.pack) ;
        p74_alt_SET((float) -2.4698687E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)4.1548468E37F, PH.base.pack) ;
        p74_climb_SET((float) -7.330907E36F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -31192, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)56074, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_y_SET((int32_t)1526727718, PH.base.pack) ;
        p75_x_SET((int32_t)1428639671, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p75_param3_SET((float)2.3660027E38F, PH.base.pack) ;
        p75_param1_SET((float)6.20393E37F, PH.base.pack) ;
        p75_param2_SET((float) -3.2904719E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_RESET_CAMERA_SETTINGS, PH.base.pack) ;
        p75_z_SET((float)1.2981831E37F, PH.base.pack) ;
        p75_param4_SET((float) -1.6360606E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_confirmation_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p76_param6_SET((float)2.9473333E38F, PH.base.pack) ;
        p76_param7_SET((float)2.9112507E38F, PH.base.pack) ;
        p76_param3_SET((float)5.5115354E37F, PH.base.pack) ;
        p76_param4_SET((float) -3.094811E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, PH.base.pack) ;
        p76_param2_SET((float)2.578075E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p76_param5_SET((float) -2.9537514E38F, PH.base.pack) ;
        p76_param1_SET((float)1.4856071E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_system_SET((uint8_t)(uint8_t)160, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        p77_result_param2_SET((int32_t) -1392551444, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)61, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONFIGURE, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)48, &PH) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_yaw_SET((float) -2.5769128E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p81_pitch_SET((float)2.6687722E38F, PH.base.pack) ;
        p81_thrust_SET((float)4.932428E37F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3182681482L, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p81_roll_SET((float)7.2859557E37F, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)1314195844L, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)2.9282567E38F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)3.0949497E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p82_body_roll_rate_SET((float) -4.5887187E37F, PH.base.pack) ;
        p82_thrust_SET((float) -8.0850755E37F, PH.base.pack) ;
        {
            float q[] =  {1.7856136E38F, 1.1177703E38F, -2.1939154E38F, -9.711216E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        {
            float q[] =  {1.5998052E38F, 4.2528737E37F, -2.1010351E38F, 2.8941404E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p83_body_roll_rate_SET((float)8.4701054E36F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -6.956786E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)301509195L, PH.base.pack) ;
        p83_body_pitch_rate_SET((float) -3.5508072E37F, PH.base.pack) ;
        p83_thrust_SET((float) -4.5504113E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vx_SET((float) -2.891129E38F, PH.base.pack) ;
        p84_vz_SET((float)3.0032985E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p84_afx_SET((float)2.3768617E38F, PH.base.pack) ;
        p84_z_SET((float)2.0610686E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3928596148L, PH.base.pack) ;
        p84_yaw_rate_SET((float) -2.0364613E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)25093, PH.base.pack) ;
        p84_yaw_SET((float) -1.8551701E38F, PH.base.pack) ;
        p84_afy_SET((float) -3.2157586E38F, PH.base.pack) ;
        p84_afz_SET((float) -2.3037978E37F, PH.base.pack) ;
        p84_y_SET((float) -7.0852846E36F, PH.base.pack) ;
        p84_x_SET((float) -1.3894088E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p84_vy_SET((float) -1.1648928E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_alt_SET((float)5.992558E37F, PH.base.pack) ;
        p86_afx_SET((float)2.82545E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1032605769, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p86_afz_SET((float)1.0707007E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)82254974, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)119, PH.base.pack) ;
        p86_vx_SET((float) -1.4340605E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -1.7621854E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p86_vy_SET((float) -2.1245477E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)327883862L, PH.base.pack) ;
        p86_vz_SET((float) -4.907848E37F, PH.base.pack) ;
        p86_yaw_SET((float) -3.0244525E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p86_afy_SET((float)7.9075696E35F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_time_boot_ms_SET((uint32_t)3574555239L, PH.base.pack) ;
        p87_alt_SET((float)8.951415E37F, PH.base.pack) ;
        p87_afy_SET((float) -2.0977096E38F, PH.base.pack) ;
        p87_afx_SET((float) -2.6932175E37F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)26291, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1114570200, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p87_yaw_SET((float) -1.0385156E38F, PH.base.pack) ;
        p87_vy_SET((float)1.8570982E38F, PH.base.pack) ;
        p87_vx_SET((float) -1.9479516E38F, PH.base.pack) ;
        p87_vz_SET((float)1.9902068E38F, PH.base.pack) ;
        p87_afz_SET((float) -1.4837552E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)2.2591641E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)1580129291, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_roll_SET((float)3.0922452E38F, PH.base.pack) ;
        p89_pitch_SET((float) -2.6308358E38F, PH.base.pack) ;
        p89_yaw_SET((float) -1.4626095E38F, PH.base.pack) ;
        p89_y_SET((float) -7.049925E37F, PH.base.pack) ;
        p89_z_SET((float)1.2250702E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)4292054032L, PH.base.pack) ;
        p89_x_SET((float)6.546275E36F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_lat_SET((int32_t)2060587564, PH.base.pack) ;
        p90_roll_SET((float) -1.3282148E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -21029, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)31394, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)5883753331587557378L, PH.base.pack) ;
        p90_pitch_SET((float) -7.386345E37F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -7964, PH.base.pack) ;
        p90_yaw_SET((float) -2.338789E38F, PH.base.pack) ;
        p90_lon_SET((int32_t) -2141237150, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -7784, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -3769, PH.base.pack) ;
        p90_rollspeed_SET((float) -1.796483E38F, PH.base.pack) ;
        p90_alt_SET((int32_t) -1387664493, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)21016, PH.base.pack) ;
        p90_yawspeed_SET((float)1.2936937E38F, PH.base.pack) ;
        p90_pitchspeed_SET((float)3.2370736E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_nav_mode_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float)2.490925E38F, PH.base.pack) ;
        p91_aux3_SET((float)1.168957E37F, PH.base.pack) ;
        p91_aux1_SET((float)1.0659376E38F, PH.base.pack) ;
        p91_aux4_SET((float) -2.308129E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -2.2546685E38F, PH.base.pack) ;
        p91_throttle_SET((float) -3.246258E38F, PH.base.pack) ;
        p91_aux2_SET((float)1.2564779E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)1633818521005320738L, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -2.586998E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan11_raw_SET((uint16_t)(uint16_t)53254, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)20674, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)19140, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)9013610475954690904L, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)64505, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)44240, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)44697, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)42493, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)1092, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)18694, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)10620, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)11834, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)18977, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)2008899600502709537L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)2499566766528808647L, PH.base.pack) ;
        {
            float controls[] =  {-3.1173938E38F, 1.490342E38F, -1.4687787E38F, -1.947081E38F, 2.9679264E38F, 2.6024552E38F, 1.7256043E38F, 1.913395E38F, 2.752434E38F, 2.777667E38F, -2.344676E38F, -3.1658805E38F, -2.1105192E38F, 1.6557489E38F, -7.696911E37F, -1.4337323E36F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.703189E38F, &PH) ;
        p100_quality_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -1.6020259E38F, &PH) ;
        p100_time_usec_SET((uint64_t)5557575625258427076L, PH.base.pack) ;
        p100_ground_distance_SET((float) -5.9747694E37F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)2.3765563E38F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -13661, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)5.0919796E37F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -17715, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_pitch_SET((float) -5.02411E37F, PH.base.pack) ;
        p101_roll_SET((float) -6.565082E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)6398444975680307066L, PH.base.pack) ;
        p101_y_SET((float) -2.3965111E38F, PH.base.pack) ;
        p101_x_SET((float)7.414084E37F, PH.base.pack) ;
        p101_yaw_SET((float) -8.3678595E37F, PH.base.pack) ;
        p101_z_SET((float)3.2275751E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float)1.9951484E38F, PH.base.pack) ;
        p102_pitch_SET((float) -3.2416377E38F, PH.base.pack) ;
        p102_x_SET((float) -2.0341529E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)7657209185573825373L, PH.base.pack) ;
        p102_z_SET((float) -1.4616549E38F, PH.base.pack) ;
        p102_roll_SET((float) -1.5092566E38F, PH.base.pack) ;
        p102_yaw_SET((float) -2.169269E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_y_SET((float)2.0595304E36F, PH.base.pack) ;
        p103_z_SET((float) -1.2530242E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)3868285834544025560L, PH.base.pack) ;
        p103_x_SET((float) -8.866778E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_roll_SET((float) -2.3380325E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)5262888741313546849L, PH.base.pack) ;
        p104_pitch_SET((float)1.1174046E38F, PH.base.pack) ;
        p104_z_SET((float)1.199164E38F, PH.base.pack) ;
        p104_x_SET((float) -2.565745E38F, PH.base.pack) ;
        p104_y_SET((float)2.9889686E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.8799044E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zacc_SET((float) -2.259699E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.3641233E38F, PH.base.pack) ;
        p105_xgyro_SET((float) -2.6532888E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.3790774E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)5327408275006765055L, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)52110, PH.base.pack) ;
        p105_ygyro_SET((float) -2.4566655E38F, PH.base.pack) ;
        p105_temperature_SET((float) -2.1814135E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float)3.342753E38F, PH.base.pack) ;
        p105_yacc_SET((float) -2.0835693E38F, PH.base.pack) ;
        p105_ymag_SET((float)5.015795E37F, PH.base.pack) ;
        p105_xmag_SET((float) -5.646849E37F, PH.base.pack) ;
        p105_xacc_SET((float)3.0868266E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -3.3966688E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -2.3497153E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_y_SET((float)3.0870367E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -6848, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.0228954E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)5.96777E37F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1984009130L, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)1.361553E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6243455471927714390L, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.4039103E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3024775187L, PH.base.pack) ;
        p106_distance_SET((float)2.4651928E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_ygyro_SET((float)1.1085054E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)1236904693L, PH.base.pack) ;
        p107_temperature_SET((float) -1.103519E38F, PH.base.pack) ;
        p107_zmag_SET((float)3.9472163E37F, PH.base.pack) ;
        p107_ymag_SET((float) -3.1447807E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)3497980501262514681L, PH.base.pack) ;
        p107_xacc_SET((float) -3.083923E38F, PH.base.pack) ;
        p107_xmag_SET((float)1.2262534E38F, PH.base.pack) ;
        p107_yacc_SET((float)2.4409365E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -2.6555053E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -3.0440869E38F, PH.base.pack) ;
        p107_xgyro_SET((float)1.3246188E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)2.0887452E38F, PH.base.pack) ;
        p107_zacc_SET((float)1.7751402E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.763801E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_q3_SET((float)1.1320683E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.1608536E38F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.8602357E38F, PH.base.pack) ;
        p108_xacc_SET((float) -3.239696E38F, PH.base.pack) ;
        p108_yacc_SET((float) -2.6946765E38F, PH.base.pack) ;
        p108_lon_SET((float) -4.1214196E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.65495E38F, PH.base.pack) ;
        p108_ygyro_SET((float)1.6533707E38F, PH.base.pack) ;
        p108_zacc_SET((float)1.6822182E38F, PH.base.pack) ;
        p108_vn_SET((float)1.984986E38F, PH.base.pack) ;
        p108_vd_SET((float) -2.8346521E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -8.385314E36F, PH.base.pack) ;
        p108_ve_SET((float)9.623275E37F, PH.base.pack) ;
        p108_q2_SET((float) -1.0085493E38F, PH.base.pack) ;
        p108_alt_SET((float)1.251144E38F, PH.base.pack) ;
        p108_lat_SET((float)1.7019737E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -4.508862E37F, PH.base.pack) ;
        p108_q4_SET((float) -1.8148735E38F, PH.base.pack) ;
        p108_roll_SET((float)4.2068058E37F, PH.base.pack) ;
        p108_q1_SET((float) -3.3345981E38F, PH.base.pack) ;
        p108_yaw_SET((float) -5.8753897E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rssi_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)10576, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)22456, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)79, (uint8_t)155, (uint8_t)220, (uint8_t)2, (uint8_t)115, (uint8_t)196, (uint8_t)129, (uint8_t)196, (uint8_t)183, (uint8_t)226, (uint8_t)22, (uint8_t)68, (uint8_t)169, (uint8_t)55, (uint8_t)83, (uint8_t)135, (uint8_t)75, (uint8_t)96, (uint8_t)55, (uint8_t)111, (uint8_t)128, (uint8_t)165, (uint8_t)49, (uint8_t)85, (uint8_t)102, (uint8_t)4, (uint8_t)26, (uint8_t)84, (uint8_t)33, (uint8_t)53, (uint8_t)0, (uint8_t)194, (uint8_t)235, (uint8_t)141, (uint8_t)13, (uint8_t)124, (uint8_t)66, (uint8_t)71, (uint8_t)32, (uint8_t)99, (uint8_t)75, (uint8_t)209, (uint8_t)220, (uint8_t)42, (uint8_t)106, (uint8_t)116, (uint8_t)104, (uint8_t)25, (uint8_t)183, (uint8_t)144, (uint8_t)219, (uint8_t)3, (uint8_t)142, (uint8_t)95, (uint8_t)100, (uint8_t)57, (uint8_t)62, (uint8_t)114, (uint8_t)209, (uint8_t)23, (uint8_t)64, (uint8_t)102, (uint8_t)243, (uint8_t)205, (uint8_t)23, (uint8_t)37, (uint8_t)71, (uint8_t)179, (uint8_t)65, (uint8_t)26, (uint8_t)229, (uint8_t)170, (uint8_t)10, (uint8_t)28, (uint8_t)201, (uint8_t)1, (uint8_t)192, (uint8_t)110, (uint8_t)189, (uint8_t)217, (uint8_t)211, (uint8_t)230, (uint8_t)108, (uint8_t)159, (uint8_t)83, (uint8_t)245, (uint8_t)85, (uint8_t)209, (uint8_t)28, (uint8_t)205, (uint8_t)47, (uint8_t)36, (uint8_t)49, (uint8_t)81, (uint8_t)161, (uint8_t)230, (uint8_t)177, (uint8_t)189, (uint8_t)236, (uint8_t)228, (uint8_t)241, (uint8_t)30, (uint8_t)61, (uint8_t)112, (uint8_t)233, (uint8_t)98, (uint8_t)222, (uint8_t)161, (uint8_t)96, (uint8_t)89, (uint8_t)174, (uint8_t)85, (uint8_t)145, (uint8_t)171, (uint8_t)183, (uint8_t)62, (uint8_t)71, (uint8_t)99, (uint8_t)155, (uint8_t)138, (uint8_t)50, (uint8_t)112, (uint8_t)98, (uint8_t)191, (uint8_t)32, (uint8_t)54, (uint8_t)250, (uint8_t)200, (uint8_t)167, (uint8_t)162, (uint8_t)239, (uint8_t)137, (uint8_t)16, (uint8_t)198, (uint8_t)208, (uint8_t)93, (uint8_t)246, (uint8_t)123, (uint8_t)39, (uint8_t)253, (uint8_t)46, (uint8_t)123, (uint8_t)104, (uint8_t)46, (uint8_t)131, (uint8_t)203, (uint8_t)62, (uint8_t)169, (uint8_t)255, (uint8_t)34, (uint8_t)96, (uint8_t)110, (uint8_t)235, (uint8_t)244, (uint8_t)66, (uint8_t)79, (uint8_t)57, (uint8_t)23, (uint8_t)18, (uint8_t)66, (uint8_t)232, (uint8_t)192, (uint8_t)37, (uint8_t)197, (uint8_t)161, (uint8_t)29, (uint8_t)252, (uint8_t)23, (uint8_t)53, (uint8_t)108, (uint8_t)68, (uint8_t)102, (uint8_t)134, (uint8_t)116, (uint8_t)0, (uint8_t)213, (uint8_t)136, (uint8_t)228, (uint8_t)23, (uint8_t)14, (uint8_t)242, (uint8_t)65, (uint8_t)136, (uint8_t)9, (uint8_t)136, (uint8_t)48, (uint8_t)237, (uint8_t)78, (uint8_t)35, (uint8_t)177, (uint8_t)18, (uint8_t)63, (uint8_t)253, (uint8_t)134, (uint8_t)121, (uint8_t)147, (uint8_t)151, (uint8_t)180, (uint8_t)180, (uint8_t)147, (uint8_t)99, (uint8_t)240, (uint8_t)163, (uint8_t)228, (uint8_t)183, (uint8_t)161, (uint8_t)110, (uint8_t)103, (uint8_t)81, (uint8_t)24, (uint8_t)28, (uint8_t)91, (uint8_t)59, (uint8_t)232, (uint8_t)180, (uint8_t)106, (uint8_t)69, (uint8_t)242, (uint8_t)239, (uint8_t)128, (uint8_t)229, (uint8_t)252, (uint8_t)65, (uint8_t)12, (uint8_t)202, (uint8_t)47, (uint8_t)216, (uint8_t)108, (uint8_t)215, (uint8_t)145, (uint8_t)195, (uint8_t)26, (uint8_t)6, (uint8_t)196, (uint8_t)57, (uint8_t)233, (uint8_t)15, (uint8_t)48, (uint8_t)164, (uint8_t)15, (uint8_t)33, (uint8_t)200, (uint8_t)27, (uint8_t)229, (uint8_t)33, (uint8_t)14, (uint8_t)63, (uint8_t)127, (uint8_t)216, (uint8_t)90, (uint8_t)201};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t) -6521012519485536926L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -7518042076245032089L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)3413108836L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)6412232158866052472L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_ve_SET((int16_t)(int16_t) -7687, PH.base.pack) ;
        p113_lon_SET((int32_t)1348572436, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p113_alt_SET((int32_t)1698394982, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)56578, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)25683, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)51872, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)37490, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)413, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -32211, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p113_lat_SET((int32_t) -985472097, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)6695839315138885465L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_x_SET((float) -2.2074703E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.7231077E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)30000, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1204599219L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2828737808L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.1208134E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)2485775584177143598L, PH.base.pack) ;
        p114_distance_SET((float)3.3627166E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -3.0351792E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)1.8664173E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_zacc_SET((int16_t)(int16_t)16599, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)286251839477431411L, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -775, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -4105, PH.base.pack) ;
        p115_rollspeed_SET((float) -2.0791662E38F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)9715, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)8531, PH.base.pack) ;
        p115_pitchspeed_SET((float)1.6015384E38F, PH.base.pack) ;
        p115_alt_SET((int32_t)1322358730, PH.base.pack) ;
        p115_lon_SET((int32_t) -437800818, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)5066, PH.base.pack) ;
        p115_lat_SET((int32_t) -101551254, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -6107, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.1573367E38F, -1.796702E38F, 4.9040458E36F, 1.5565634E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_ind_airspeed_SET((uint16_t)(uint16_t)56161, PH.base.pack) ;
        p115_yawspeed_SET((float)2.1073369E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_zgyro_SET((int16_t)(int16_t)6279, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -25569, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -6614, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -15357, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)27999, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)10054, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)6547, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)8982, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)599, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)1366183339L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)13937, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)12058, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_id_SET((uint16_t)(uint16_t)5075, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)49101, PH.base.pack) ;
        p118_size_SET((uint32_t)773196716L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)19089, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)3459459763L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p119_count_SET((uint32_t)2395286891L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)25276, PH.base.pack) ;
        p119_ofs_SET((uint32_t)660334345L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)848742864L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)21665, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)227, (uint8_t)30, (uint8_t)36, (uint8_t)93, (uint8_t)205, (uint8_t)146, (uint8_t)18, (uint8_t)184, (uint8_t)110, (uint8_t)144, (uint8_t)84, (uint8_t)232, (uint8_t)121, (uint8_t)81, (uint8_t)247, (uint8_t)165, (uint8_t)199, (uint8_t)171, (uint8_t)6, (uint8_t)194, (uint8_t)24, (uint8_t)160, (uint8_t)75, (uint8_t)215, (uint8_t)163, (uint8_t)149, (uint8_t)169, (uint8_t)255, (uint8_t)3, (uint8_t)206, (uint8_t)203, (uint8_t)56, (uint8_t)204, (uint8_t)49, (uint8_t)12, (uint8_t)181, (uint8_t)176, (uint8_t)6, (uint8_t)48, (uint8_t)63, (uint8_t)4, (uint8_t)4, (uint8_t)89, (uint8_t)53, (uint8_t)144, (uint8_t)237, (uint8_t)215, (uint8_t)194, (uint8_t)50, (uint8_t)17, (uint8_t)9, (uint8_t)30, (uint8_t)52, (uint8_t)111, (uint8_t)66, (uint8_t)55, (uint8_t)131, (uint8_t)30, (uint8_t)248, (uint8_t)49, (uint8_t)167, (uint8_t)26, (uint8_t)192, (uint8_t)194, (uint8_t)188, (uint8_t)210, (uint8_t)20, (uint8_t)210, (uint8_t)152, (uint8_t)5, (uint8_t)49, (uint8_t)175, (uint8_t)3, (uint8_t)190, (uint8_t)224, (uint8_t)30, (uint8_t)153, (uint8_t)192, (uint8_t)191, (uint8_t)13, (uint8_t)30, (uint8_t)42, (uint8_t)170, (uint8_t)107, (uint8_t)179, (uint8_t)42, (uint8_t)217, (uint8_t)186, (uint8_t)123, (uint8_t)157};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)159, (uint8_t)25, (uint8_t)82, (uint8_t)119, (uint8_t)19, (uint8_t)245, (uint8_t)1, (uint8_t)208, (uint8_t)47, (uint8_t)157, (uint8_t)89, (uint8_t)170, (uint8_t)94, (uint8_t)12, (uint8_t)12, (uint8_t)71, (uint8_t)141, (uint8_t)165, (uint8_t)54, (uint8_t)137, (uint8_t)199, (uint8_t)234, (uint8_t)0, (uint8_t)245, (uint8_t)222, (uint8_t)109, (uint8_t)158, (uint8_t)78, (uint8_t)39, (uint8_t)3, (uint8_t)55, (uint8_t)132, (uint8_t)66, (uint8_t)196, (uint8_t)57, (uint8_t)154, (uint8_t)118, (uint8_t)130, (uint8_t)126, (uint8_t)61, (uint8_t)165, (uint8_t)230, (uint8_t)173, (uint8_t)89, (uint8_t)57, (uint8_t)113, (uint8_t)84, (uint8_t)235, (uint8_t)209, (uint8_t)236, (uint8_t)87, (uint8_t)170, (uint8_t)48, (uint8_t)180, (uint8_t)6, (uint8_t)74, (uint8_t)138, (uint8_t)26, (uint8_t)101, (uint8_t)232, (uint8_t)205, (uint8_t)31, (uint8_t)126, (uint8_t)24, (uint8_t)197, (uint8_t)172, (uint8_t)68, (uint8_t)167, (uint8_t)130, (uint8_t)113, (uint8_t)100, (uint8_t)203, (uint8_t)81, (uint8_t)99, (uint8_t)82, (uint8_t)45, (uint8_t)219, (uint8_t)211, (uint8_t)57, (uint8_t)61, (uint8_t)78, (uint8_t)32, (uint8_t)200, (uint8_t)133, (uint8_t)241, (uint8_t)122, (uint8_t)241, (uint8_t)60, (uint8_t)217, (uint8_t)138, (uint8_t)9, (uint8_t)168, (uint8_t)255, (uint8_t)12, (uint8_t)118, (uint8_t)234, (uint8_t)150, (uint8_t)64, (uint8_t)219, (uint8_t)77, (uint8_t)17, (uint8_t)22, (uint8_t)50, (uint8_t)168, (uint8_t)121, (uint8_t)27, (uint8_t)167, (uint8_t)0, (uint8_t)7, (uint8_t)17};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)35761, PH.base.pack) ;
        p124_lon_SET((int32_t) -414116519, PH.base.pack) ;
        p124_lat_SET((int32_t) -409528298, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)39442, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)3267072070L, PH.base.pack) ;
        p124_alt_SET((int32_t)1320078496, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)48208, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)40410, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6255810399688821307L, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED), PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)11695, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)45860, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING), PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)26283, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)37, (uint8_t)183, (uint8_t)3, (uint8_t)220, (uint8_t)53, (uint8_t)167, (uint8_t)166, (uint8_t)203, (uint8_t)172, (uint8_t)170, (uint8_t)244, (uint8_t)128, (uint8_t)73, (uint8_t)228, (uint8_t)162, (uint8_t)224, (uint8_t)18, (uint8_t)120, (uint8_t)80, (uint8_t)190, (uint8_t)78, (uint8_t)13, (uint8_t)124, (uint8_t)246, (uint8_t)65, (uint8_t)181, (uint8_t)235, (uint8_t)1, (uint8_t)6, (uint8_t)67, (uint8_t)117, (uint8_t)99, (uint8_t)11, (uint8_t)112, (uint8_t)210, (uint8_t)117, (uint8_t)63, (uint8_t)113, (uint8_t)12, (uint8_t)31, (uint8_t)173, (uint8_t)87, (uint8_t)49, (uint8_t)110, (uint8_t)19, (uint8_t)154, (uint8_t)114, (uint8_t)9, (uint8_t)133, (uint8_t)207, (uint8_t)205, (uint8_t)88, (uint8_t)0, (uint8_t)54, (uint8_t)96, (uint8_t)173, (uint8_t)25, (uint8_t)238, (uint8_t)176, (uint8_t)147, (uint8_t)231, (uint8_t)111, (uint8_t)174, (uint8_t)187, (uint8_t)4, (uint8_t)233, (uint8_t)242, (uint8_t)103, (uint8_t)16, (uint8_t)119};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_baudrate_SET((uint32_t)1078530722L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_b_mm_SET((int32_t) -234162584, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)886624138L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -702188722, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)33311, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1266712628, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -1129655576, PH.base.pack) ;
        p127_tow_SET((uint32_t)624118737L, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)3992493200L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_tow_SET((uint32_t)2315013519L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1013020325, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2428918696L, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)1650103125, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)1794090091L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)1228148472, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1758564639, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)30218, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ygyro_SET((int16_t)(int16_t) -12241, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)5630, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -9821, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -25915, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)17482, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -943, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)895988823L, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)18114, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -30448, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -4778, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_type_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)15142, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)58901, PH.base.pack) ;
        p130_size_SET((uint32_t)374300074L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)9263, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)50, (uint8_t)208, (uint8_t)213, (uint8_t)90, (uint8_t)196, (uint8_t)202, (uint8_t)180, (uint8_t)31, (uint8_t)78, (uint8_t)100, (uint8_t)0, (uint8_t)9, (uint8_t)141, (uint8_t)149, (uint8_t)212, (uint8_t)58, (uint8_t)167, (uint8_t)84, (uint8_t)182, (uint8_t)109, (uint8_t)110, (uint8_t)41, (uint8_t)224, (uint8_t)71, (uint8_t)66, (uint8_t)230, (uint8_t)67, (uint8_t)53, (uint8_t)186, (uint8_t)137, (uint8_t)248, (uint8_t)125, (uint8_t)30, (uint8_t)123, (uint8_t)55, (uint8_t)136, (uint8_t)124, (uint8_t)219, (uint8_t)102, (uint8_t)170, (uint8_t)70, (uint8_t)15, (uint8_t)20, (uint8_t)205, (uint8_t)162, (uint8_t)58, (uint8_t)246, (uint8_t)16, (uint8_t)110, (uint8_t)89, (uint8_t)185, (uint8_t)230, (uint8_t)14, (uint8_t)194, (uint8_t)17, (uint8_t)93, (uint8_t)164, (uint8_t)186, (uint8_t)69, (uint8_t)49, (uint8_t)140, (uint8_t)21, (uint8_t)226, (uint8_t)95, (uint8_t)86, (uint8_t)57, (uint8_t)218, (uint8_t)128, (uint8_t)202, (uint8_t)212, (uint8_t)103, (uint8_t)199, (uint8_t)142, (uint8_t)222, (uint8_t)38, (uint8_t)34, (uint8_t)140, (uint8_t)155, (uint8_t)249, (uint8_t)113, (uint8_t)61, (uint8_t)12, (uint8_t)254, (uint8_t)53, (uint8_t)25, (uint8_t)185, (uint8_t)226, (uint8_t)73, (uint8_t)2, (uint8_t)38, (uint8_t)56, (uint8_t)122, (uint8_t)13, (uint8_t)210, (uint8_t)93, (uint8_t)109, (uint8_t)108, (uint8_t)129, (uint8_t)49, (uint8_t)44, (uint8_t)119, (uint8_t)137, (uint8_t)172, (uint8_t)189, (uint8_t)198, (uint8_t)1, (uint8_t)207, (uint8_t)79, (uint8_t)239, (uint8_t)41, (uint8_t)213, (uint8_t)111, (uint8_t)165, (uint8_t)194, (uint8_t)119, (uint8_t)78, (uint8_t)103, (uint8_t)161, (uint8_t)34, (uint8_t)42, (uint8_t)220, (uint8_t)112, (uint8_t)120, (uint8_t)34, (uint8_t)74, (uint8_t)45, (uint8_t)4, (uint8_t)248, (uint8_t)254, (uint8_t)157, (uint8_t)196, (uint8_t)33, (uint8_t)84, (uint8_t)72, (uint8_t)112, (uint8_t)138, (uint8_t)105, (uint8_t)128, (uint8_t)86, (uint8_t)152, (uint8_t)197, (uint8_t)184, (uint8_t)224, (uint8_t)125, (uint8_t)48, (uint8_t)236, (uint8_t)48, (uint8_t)107, (uint8_t)150, (uint8_t)165, (uint8_t)143, (uint8_t)35, (uint8_t)81, (uint8_t)237, (uint8_t)235, (uint8_t)156, (uint8_t)65, (uint8_t)182, (uint8_t)125, (uint8_t)73, (uint8_t)0, (uint8_t)45, (uint8_t)136, (uint8_t)1, (uint8_t)124, (uint8_t)43, (uint8_t)1, (uint8_t)85, (uint8_t)209, (uint8_t)58, (uint8_t)54, (uint8_t)142, (uint8_t)5, (uint8_t)21, (uint8_t)225, (uint8_t)172, (uint8_t)30, (uint8_t)41, (uint8_t)144, (uint8_t)151, (uint8_t)181, (uint8_t)16, (uint8_t)55, (uint8_t)176, (uint8_t)230, (uint8_t)87, (uint8_t)200, (uint8_t)240, (uint8_t)92, (uint8_t)155, (uint8_t)191, (uint8_t)213, (uint8_t)173, (uint8_t)169, (uint8_t)39, (uint8_t)144, (uint8_t)38, (uint8_t)225, (uint8_t)250, (uint8_t)126, (uint8_t)103, (uint8_t)87, (uint8_t)235, (uint8_t)156, (uint8_t)102, (uint8_t)42, (uint8_t)215, (uint8_t)184, (uint8_t)11, (uint8_t)63, (uint8_t)15, (uint8_t)204, (uint8_t)111, (uint8_t)71, (uint8_t)113, (uint8_t)43, (uint8_t)18, (uint8_t)150, (uint8_t)234, (uint8_t)158, (uint8_t)127, (uint8_t)216, (uint8_t)227, (uint8_t)127, (uint8_t)252, (uint8_t)168, (uint8_t)123, (uint8_t)208, (uint8_t)228, (uint8_t)5, (uint8_t)98, (uint8_t)64, (uint8_t)24, (uint8_t)179, (uint8_t)164, (uint8_t)129, (uint8_t)21, (uint8_t)239, (uint8_t)222, (uint8_t)146, (uint8_t)168, (uint8_t)220, (uint8_t)76, (uint8_t)154, (uint8_t)110, (uint8_t)3, (uint8_t)202, (uint8_t)171, (uint8_t)100, (uint8_t)124, (uint8_t)48, (uint8_t)0, (uint8_t)110};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)15552, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_270, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)38359, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)8900, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)15752, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2905154918L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t)2129197718, PH.base.pack) ;
        p133_lon_SET((int32_t) -1424407746, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)26557, PH.base.pack) ;
        p133_mask_SET((uint64_t)2191441364107831369L, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t)314718699, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p134_lat_SET((int32_t)132995115, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)20864, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -8808, (int16_t) -8283, (int16_t)30294, (int16_t)7532, (int16_t) -115, (int16_t) -26668, (int16_t)32005, (int16_t)3451, (int16_t)7980, (int16_t) -32428, (int16_t)32062, (int16_t)11547, (int16_t)13928, (int16_t)21574, (int16_t)815, (int16_t)27358};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -1694972628, PH.base.pack) ;
        p135_lon_SET((int32_t) -57571477, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_lon_SET((int32_t)258694502, PH.base.pack) ;
        p136_current_height_SET((float) -3.2028582E38F, PH.base.pack) ;
        p136_terrain_height_SET((float) -5.894707E37F, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)51408, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)61533, PH.base.pack) ;
        p136_lat_SET((int32_t)1782495982, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)27287, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t)29333, PH.base.pack) ;
        p137_press_diff_SET((float) -6.037726E37F, PH.base.pack) ;
        p137_press_abs_SET((float)3.3457875E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)1809440919L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {-3.647309E37F, -2.3497543E38F, 7.6665764E37F, -1.4766395E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -2.5245982E38F, PH.base.pack) ;
        p138_y_SET((float) -4.864497E37F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)5700794980596979842L, PH.base.pack) ;
        p138_z_SET((float) -1.7813355E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-2.217441E38F, -1.0152843E38F, -2.794164E38F, -3.3208996E38F, 6.8186634E36F, 2.0835507E38F, 1.5865358E38F, -1.0463256E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_time_usec_SET((uint64_t)6592813568357024449L, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {3.0582261E38F, -2.12855E38F, -1.792377E38F, 4.805427E37F, 2.9343377E38F, 1.3483068E38F, 2.9879506E38F, -1.5830491E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)7166657120244584223L, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_time_usec_SET((uint64_t)5561552114062385192L, PH.base.pack) ;
        p141_altitude_amsl_SET((float)2.3296213E37F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.5537455E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)1.2762322E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.8852612E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -4.637614E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)1.9491576E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)47, (uint8_t)49, (uint8_t)5, (uint8_t)58, (uint8_t)56, (uint8_t)110, (uint8_t)136, (uint8_t)236, (uint8_t)212, (uint8_t)71, (uint8_t)130, (uint8_t)26, (uint8_t)202, (uint8_t)173, (uint8_t)150, (uint8_t)192, (uint8_t)191, (uint8_t)26, (uint8_t)91, (uint8_t)121, (uint8_t)95, (uint8_t)228, (uint8_t)73, (uint8_t)221, (uint8_t)165, (uint8_t)55, (uint8_t)68, (uint8_t)216, (uint8_t)14, (uint8_t)110, (uint8_t)123, (uint8_t)56, (uint8_t)200, (uint8_t)202, (uint8_t)147, (uint8_t)218, (uint8_t)220, (uint8_t)34, (uint8_t)27, (uint8_t)102, (uint8_t)185, (uint8_t)254, (uint8_t)212, (uint8_t)131, (uint8_t)53, (uint8_t)10, (uint8_t)26, (uint8_t)165, (uint8_t)92, (uint8_t)211, (uint8_t)103, (uint8_t)90, (uint8_t)31, (uint8_t)138, (uint8_t)74, (uint8_t)236, (uint8_t)178, (uint8_t)118, (uint8_t)224, (uint8_t)122, (uint8_t)73, (uint8_t)187, (uint8_t)6, (uint8_t)57, (uint8_t)94, (uint8_t)225, (uint8_t)48, (uint8_t)39, (uint8_t)51, (uint8_t)135, (uint8_t)60, (uint8_t)115, (uint8_t)84, (uint8_t)63, (uint8_t)203, (uint8_t)16, (uint8_t)8, (uint8_t)46, (uint8_t)38, (uint8_t)243, (uint8_t)242, (uint8_t)153, (uint8_t)135, (uint8_t)204, (uint8_t)163, (uint8_t)6, (uint8_t)186, (uint8_t)152, (uint8_t)201, (uint8_t)84, (uint8_t)47, (uint8_t)32, (uint8_t)218, (uint8_t)78, (uint8_t)70, (uint8_t)155, (uint8_t)231, (uint8_t)155, (uint8_t)64, (uint8_t)30, (uint8_t)154, (uint8_t)97, (uint8_t)186, (uint8_t)247, (uint8_t)197, (uint8_t)136, (uint8_t)249, (uint8_t)211, (uint8_t)190, (uint8_t)184, (uint8_t)166, (uint8_t)231, (uint8_t)135, (uint8_t)130, (uint8_t)175, (uint8_t)8, (uint8_t)145, (uint8_t)107, (uint8_t)0, (uint8_t)158};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)13, (uint8_t)212, (uint8_t)143, (uint8_t)223, (uint8_t)237, (uint8_t)142, (uint8_t)161, (uint8_t)250, (uint8_t)204, (uint8_t)63, (uint8_t)169, (uint8_t)183, (uint8_t)165, (uint8_t)188, (uint8_t)142, (uint8_t)122, (uint8_t)222, (uint8_t)87, (uint8_t)129, (uint8_t)49, (uint8_t)38, (uint8_t)52, (uint8_t)200, (uint8_t)75, (uint8_t)242, (uint8_t)115, (uint8_t)172, (uint8_t)109, (uint8_t)114, (uint8_t)121, (uint8_t)197, (uint8_t)9, (uint8_t)38, (uint8_t)147, (uint8_t)146, (uint8_t)104, (uint8_t)67, (uint8_t)209, (uint8_t)204, (uint8_t)242, (uint8_t)234, (uint8_t)242, (uint8_t)88, (uint8_t)170, (uint8_t)28, (uint8_t)146, (uint8_t)0, (uint8_t)187, (uint8_t)217, (uint8_t)249, (uint8_t)75, (uint8_t)115, (uint8_t)17, (uint8_t)59, (uint8_t)45, (uint8_t)54, (uint8_t)182, (uint8_t)136, (uint8_t)165, (uint8_t)118, (uint8_t)57, (uint8_t)94, (uint8_t)55, (uint8_t)142, (uint8_t)85, (uint8_t)222, (uint8_t)14, (uint8_t)219, (uint8_t)129, (uint8_t)157, (uint8_t)123, (uint8_t)148, (uint8_t)241, (uint8_t)125, (uint8_t)45, (uint8_t)139, (uint8_t)130, (uint8_t)123, (uint8_t)106, (uint8_t)215, (uint8_t)34, (uint8_t)132, (uint8_t)254, (uint8_t)120, (uint8_t)172, (uint8_t)119, (uint8_t)168, (uint8_t)46, (uint8_t)42, (uint8_t)63, (uint8_t)155, (uint8_t)231, (uint8_t)60, (uint8_t)29, (uint8_t)163, (uint8_t)89, (uint8_t)127, (uint8_t)89, (uint8_t)16, (uint8_t)119, (uint8_t)149, (uint8_t)169, (uint8_t)223, (uint8_t)26, (uint8_t)100, (uint8_t)31, (uint8_t)170, (uint8_t)19, (uint8_t)203, (uint8_t)161, (uint8_t)104, (uint8_t)62, (uint8_t)168, (uint8_t)35, (uint8_t)23, (uint8_t)158, (uint8_t)235, (uint8_t)52, (uint8_t)189, (uint8_t)188};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float) -2.5283293E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)22268, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)3635556367L, PH.base.pack) ;
        p143_press_diff_SET((float)3.267028E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_custom_state_SET((uint64_t)7731858259760842996L, PH.base.pack) ;
        p144_alt_SET((float) -6.6188743E37F, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        {
            float attitude_q[] =  {1.9020164E38F, 2.4460359E38F, -1.180927E37F, -1.2886351E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -1405692460, PH.base.pack) ;
        {
            float rates[] =  {-1.1132608E38F, 3.1018802E38F, -3.2896561E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)2136275444, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)3676090886080731472L, PH.base.pack) ;
        {
            float acc[] =  {5.1646727E37F, 1.1015106E38F, -4.677033E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-1.8181487E38F, 2.8767687E37F, 4.9051633E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-2.0172284E38F, -2.2732275E38F, 3.062219E36F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_acc_SET((float)5.998521E37F, PH.base.pack) ;
        p146_z_acc_SET((float) -2.3808233E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-1.0421911E38F, 2.4738706E38F, -1.2263813E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_x_pos_SET((float) -1.1833206E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.5940383E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -2.4375771E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -7.426149E37F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)3107479848229421350L, PH.base.pack) ;
        p146_pitch_rate_SET((float) -3.090332E38F, PH.base.pack) ;
        p146_y_pos_SET((float)5.5902423E37F, PH.base.pack) ;
        p146_roll_rate_SET((float)3.1179682E38F, PH.base.pack) ;
        p146_x_vel_SET((float)2.5006242E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)3.2743486E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {1.4235881E38F, 5.5510856E37F, -1.726105E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float) -1.344342E36F, PH.base.pack) ;
        p146_z_vel_SET((float)6.3229834E37F, PH.base.pack) ;
        {
            float q[] =  {-1.6049656E38F, 1.419711E38F, 6.444722E37F, -2.7566553E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1261267142, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)32424, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -1294115361, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -31869, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -84, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)28873, (uint16_t)62281, (uint16_t)56585, (uint16_t)10276, (uint16_t)19143, (uint16_t)17871, (uint16_t)31019, (uint16_t)6804, (uint16_t)22989, (uint16_t)29694};
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
            uint8_t flight_custom_version[] =  {(uint8_t)169, (uint8_t)222, (uint8_t)18, (uint8_t)81, (uint8_t)225, (uint8_t)241, (uint8_t)69, (uint8_t)89};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t os_custom_version[] =  {(uint8_t)125, (uint8_t)89, (uint8_t)107, (uint8_t)106, (uint8_t)94, (uint8_t)120, (uint8_t)35, (uint8_t)135};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)17, (uint8_t)76, (uint8_t)213, (uint8_t)21, (uint8_t)15, (uint8_t)101, (uint8_t)99, (uint8_t)236};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)8800815950528474068L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2269762600L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)187, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)179415996L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)248, (uint8_t)95, (uint8_t)226, (uint8_t)179, (uint8_t)79, (uint8_t)18, (uint8_t)3, (uint8_t)172, (uint8_t)221, (uint8_t)249, (uint8_t)65, (uint8_t)215, (uint8_t)101, (uint8_t)68, (uint8_t)103, (uint8_t)66, (uint8_t)69, (uint8_t)152};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)4195, PH.base.pack) ;
        p148_board_version_SET((uint32_t)797478935L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1112654780L, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY), PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p149_z_SET((float)4.516127E37F, &PH) ;
        p149_size_x_SET((float)9.424388E37F, PH.base.pack) ;
        p149_x_SET((float) -5.498283E37F, &PH) ;
        {
            float q[] =  {3.066418E37F, 2.094703E38F, 3.1274165E37F, 2.8510571E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_time_usec_SET((uint64_t)4729231930566739083L, PH.base.pack) ;
        p149_distance_SET((float)1.1006665E38F, PH.base.pack) ;
        p149_size_y_SET((float) -2.262753E38F, PH.base.pack) ;
        p149_angle_y_SET((float)2.2759952E37F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_angle_x_SET((float) -2.2936823E38F, PH.base.pack) ;
        p149_y_SET((float) -1.1763967E38F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)3, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CPU_LOAD_170(), &PH);
        p170_batVolt_SET((uint16_t)(uint16_t)17291, PH.base.pack) ;
        p170_ctrlLoad_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p170_sensLoad_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        c_CommunicationChannel_on_CPU_LOAD_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_BIAS_172(), &PH);
        p172_gzBias_SET((float) -1.9887842E38F, PH.base.pack) ;
        p172_axBias_SET((float) -4.1323194E36F, PH.base.pack) ;
        p172_azBias_SET((float) -1.6882394E37F, PH.base.pack) ;
        p172_gxBias_SET((float) -1.1179866E38F, PH.base.pack) ;
        p172_ayBias_SET((float) -1.3463437E38F, PH.base.pack) ;
        p172_gyBias_SET((float)6.9777624E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_BIAS_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIAGNOSTIC_173(), &PH);
        p173_diagFl1_SET((float) -1.1183116E38F, PH.base.pack) ;
        p173_diagSh2_SET((int16_t)(int16_t) -14014, PH.base.pack) ;
        p173_diagFl3_SET((float) -1.7033938E37F, PH.base.pack) ;
        p173_diagFl2_SET((float) -1.3661112E38F, PH.base.pack) ;
        p173_diagSh3_SET((int16_t)(int16_t)21160, PH.base.pack) ;
        p173_diagSh1_SET((int16_t)(int16_t)8001, PH.base.pack) ;
        c_CommunicationChannel_on_DIAGNOSTIC_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_NAVIGATION_176(), &PH);
        p176_u_m_SET((float)2.1413178E38F, PH.base.pack) ;
        p176_h_c_SET((uint16_t)(uint16_t)21155, PH.base.pack) ;
        p176_fromWP_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p176_phi_c_SET((float) -1.9709586E38F, PH.base.pack) ;
        p176_dist2Go_SET((float) -3.2713117E38F, PH.base.pack) ;
        p176_totalDist_SET((float) -2.7879488E38F, PH.base.pack) ;
        p176_toWP_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p176_ay_body_SET((float) -3.9920008E37F, PH.base.pack) ;
        p176_theta_c_SET((float) -2.1365563E38F, PH.base.pack) ;
        p176_psiDot_c_SET((float) -1.8198232E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_NAVIGATION_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA_LOG_177(), &PH);
        p177_fl_4_SET((float)2.0712196E38F, PH.base.pack) ;
        p177_fl_3_SET((float)1.0619669E38F, PH.base.pack) ;
        p177_fl_6_SET((float) -2.7682974E38F, PH.base.pack) ;
        p177_fl_2_SET((float) -5.089991E37F, PH.base.pack) ;
        p177_fl_1_SET((float)1.822784E38F, PH.base.pack) ;
        p177_fl_5_SET((float) -3.2218441E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_LOG_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_DATE_TIME_179(), &PH);
        p179_hour_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p179_year_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p179_clockStat_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p179_month_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p179_GppGl_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p179_visSat_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p179_sigUsedMask_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p179_min_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p179_sec_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p179_percentUsed_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p179_useSat_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p179_day_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_DATE_TIME_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MID_LVL_CMDS_180(), &PH);
        p180_hCommand_SET((float)3.034143E38F, PH.base.pack) ;
        p180_uCommand_SET((float) -3.2398561E38F, PH.base.pack) ;
        p180_target_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p180_rCommand_SET((float) -1.8757963E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MID_LVL_CMDS_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CTRL_SRFC_PT_181(), &PH);
        p181_bitfieldPt_SET((uint16_t)(uint16_t)41839, PH.base.pack) ;
        p181_target_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        c_CommunicationChannel_on_CTRL_SRFC_PT_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CAMERA_ORDER_184(), &PH);
        p184_tilt_SET((int8_t)(int8_t) -14, PH.base.pack) ;
        p184_pan_SET((int8_t)(int8_t) -119, PH.base.pack) ;
        p184_target_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p184_zoom_SET((int8_t)(int8_t)117, PH.base.pack) ;
        p184_moveHome_SET((int8_t)(int8_t) -6, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SURFACE_185(), &PH);
        p185_idSurface_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p185_bControl_SET((float) -2.8952645E37F, PH.base.pack) ;
        p185_target_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p185_mControl_SET((float) -2.9351894E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SURFACE_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_MOBILE_LOCATION_186(), &PH);
        p186_latitude_SET((float) -1.6528221E38F, PH.base.pack) ;
        p186_target_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p186_longitude_SET((float) -4.1761459E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CONFIGURATION_CAMERA_188(), &PH);
        p188_order_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p188_idOrder_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p188_target_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ISR_LOCATION_189(), &PH);
        p189_option2_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p189_target_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p189_height_SET((float)2.6341958E38F, PH.base.pack) ;
        p189_longitude_SET((float) -2.6601863E38F, PH.base.pack) ;
        p189_option1_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p189_option3_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p189_latitude_SET((float) -2.3901422E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ISR_LOCATION_189(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VOLT_SENSOR_191(), &PH);
        p191_reading2_SET((uint16_t)(uint16_t)9884, PH.base.pack) ;
        p191_voltage_SET((uint16_t)(uint16_t)9679, PH.base.pack) ;
        p191_r2Type_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        c_CommunicationChannel_on_VOLT_SENSOR_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PTZ_STATUS_192(), &PH);
        p192_pan_SET((int16_t)(int16_t) -864, PH.base.pack) ;
        p192_tilt_SET((int16_t)(int16_t)23241, PH.base.pack) ;
        p192_zoom_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        c_CommunicationChannel_on_PTZ_STATUS_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAV_STATUS_193(), &PH);
        p193_speed_SET((float) -2.2001652E38F, PH.base.pack) ;
        p193_latitude_SET((float) -2.2501577E38F, PH.base.pack) ;
        p193_longitude_SET((float) -2.7200977E38F, PH.base.pack) ;
        p193_altitude_SET((float)1.5352169E38F, PH.base.pack) ;
        p193_target_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p193_course_SET((float)2.4969291E38F, PH.base.pack) ;
        c_CommunicationChannel_on_UAV_STATUS_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUS_GPS_194(), &PH);
        p194_msgsType_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p194_csFails_SET((uint16_t)(uint16_t)2533, PH.base.pack) ;
        p194_gpsQuality_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p194_posStatus_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p194_magDir_SET((int8_t)(int8_t)81, PH.base.pack) ;
        p194_modeInd_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p194_magVar_SET((float)2.7204934E38F, PH.base.pack) ;
        c_CommunicationChannel_on_STATUS_GPS_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NOVATEL_DIAG_195(), &PH);
        p195_solStatus_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p195_csFails_SET((uint16_t)(uint16_t)42185, PH.base.pack) ;
        p195_velType_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p195_posType_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p195_receiverStatus_SET((uint32_t)3528696415L, PH.base.pack) ;
        p195_timeStatus_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p195_posSolAge_SET((float)1.4680518E38F, PH.base.pack) ;
        c_CommunicationChannel_on_NOVATEL_DIAG_195(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_DIAG_196(), &PH);
        p196_int1_SET((int16_t)(int16_t)23095, PH.base.pack) ;
        p196_char1_SET((int8_t)(int8_t) -68, PH.base.pack) ;
        p196_float2_SET((float) -1.0698434E38F, PH.base.pack) ;
        p196_float1_SET((float)1.6403435E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_DIAG_196(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BOOT_197(), &PH);
        p197_version_SET((uint32_t)1382537809L, PH.base.pack) ;
        c_CommunicationChannel_on_BOOT_197(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_mag_ratio_SET((float) -1.3948145E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)3.2512514E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -9.537695E37F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -1.6931808E38F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL), PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)3.3083626E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -6.821018E37F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.160803E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)1102387136783656079L, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)2.761047E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_wind_x_SET((float) -1.848325E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)1.3086165E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)5499659925661754800L, PH.base.pack) ;
        p231_wind_y_SET((float)1.7634478E38F, PH.base.pack) ;
        p231_wind_z_SET((float) -1.5790453E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)5.395813E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -9.070492E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)1.7917647E38F, PH.base.pack) ;
        p231_var_vert_SET((float)2.1098241E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_vd_SET((float) -3.3100371E38F, PH.base.pack) ;
        p232_vdop_SET((float) -1.2735507E38F, PH.base.pack) ;
        p232_vn_SET((float) -3.3015844E38F, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ), PH.base.pack) ;
        p232_vert_accuracy_SET((float) -2.5179407E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1379202724664413818L, PH.base.pack) ;
        p232_alt_SET((float)2.2659952E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)1.5531745E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)531651080L, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)34940, PH.base.pack) ;
        p232_ve_SET((float)5.8628424E37F, PH.base.pack) ;
        p232_lon_SET((int32_t)509311143, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -2.697582E37F, PH.base.pack) ;
        p232_hdop_SET((float) -1.4453898E38F, PH.base.pack) ;
        p232_lat_SET((int32_t) -299430801, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)244, (uint8_t)172, (uint8_t)135, (uint8_t)19, (uint8_t)122, (uint8_t)2, (uint8_t)183, (uint8_t)37, (uint8_t)116, (uint8_t)110, (uint8_t)200, (uint8_t)116, (uint8_t)116, (uint8_t)67, (uint8_t)219, (uint8_t)219, (uint8_t)28, (uint8_t)188, (uint8_t)31, (uint8_t)110, (uint8_t)208, (uint8_t)50, (uint8_t)86, (uint8_t)180, (uint8_t)57, (uint8_t)218, (uint8_t)60, (uint8_t)236, (uint8_t)21, (uint8_t)178, (uint8_t)199, (uint8_t)56, (uint8_t)214, (uint8_t)240, (uint8_t)1, (uint8_t)68, (uint8_t)229, (uint8_t)90, (uint8_t)183, (uint8_t)159, (uint8_t)7, (uint8_t)25, (uint8_t)173, (uint8_t)225, (uint8_t)218, (uint8_t)54, (uint8_t)101, (uint8_t)214, (uint8_t)224, (uint8_t)33, (uint8_t)159, (uint8_t)86, (uint8_t)31, (uint8_t)196, (uint8_t)169, (uint8_t)87, (uint8_t)11, (uint8_t)104, (uint8_t)219, (uint8_t)169, (uint8_t)223, (uint8_t)253, (uint8_t)161, (uint8_t)102, (uint8_t)55, (uint8_t)122, (uint8_t)172, (uint8_t)141, (uint8_t)241, (uint8_t)39, (uint8_t)242, (uint8_t)7, (uint8_t)161, (uint8_t)220, (uint8_t)251, (uint8_t)2, (uint8_t)145, (uint8_t)142, (uint8_t)99, (uint8_t)19, (uint8_t)56, (uint8_t)97, (uint8_t)116, (uint8_t)186, (uint8_t)136, (uint8_t)26, (uint8_t)17, (uint8_t)163, (uint8_t)91, (uint8_t)188, (uint8_t)3, (uint8_t)188, (uint8_t)242, (uint8_t)254, (uint8_t)126, (uint8_t)169, (uint8_t)202, (uint8_t)232, (uint8_t)73, (uint8_t)191, (uint8_t)66, (uint8_t)41, (uint8_t)43, (uint8_t)218, (uint8_t)33, (uint8_t)221, (uint8_t)90, (uint8_t)58, (uint8_t)18, (uint8_t)112, (uint8_t)98, (uint8_t)151, (uint8_t)166, (uint8_t)180, (uint8_t)214, (uint8_t)249, (uint8_t)161, (uint8_t)219, (uint8_t)149, (uint8_t)35, (uint8_t)49, (uint8_t)237, (uint8_t)3, (uint8_t)44, (uint8_t)211, (uint8_t)193, (uint8_t)23, (uint8_t)54, (uint8_t)111, (uint8_t)27, (uint8_t)75, (uint8_t)88, (uint8_t)98, (uint8_t)55, (uint8_t)223, (uint8_t)40, (uint8_t)187, (uint8_t)195, (uint8_t)175, (uint8_t)180, (uint8_t)139, (uint8_t)30, (uint8_t)84, (uint8_t)136, (uint8_t)64, (uint8_t)109, (uint8_t)43, (uint8_t)241, (uint8_t)227, (uint8_t)77, (uint8_t)109, (uint8_t)198, (uint8_t)237, (uint8_t)43, (uint8_t)185, (uint8_t)63, (uint8_t)65, (uint8_t)39, (uint8_t)34, (uint8_t)238, (uint8_t)147, (uint8_t)76, (uint8_t)166, (uint8_t)89, (uint8_t)42, (uint8_t)225, (uint8_t)179, (uint8_t)95, (uint8_t)194, (uint8_t)11, (uint8_t)241, (uint8_t)128, (uint8_t)145, (uint8_t)50, (uint8_t)235, (uint8_t)68, (uint8_t)159, (uint8_t)205, (uint8_t)88, (uint8_t)252};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -68, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -22054, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)57003, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)2331271178L, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)72, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)35885, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -10, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)21216, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -22702, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)24745, PH.base.pack) ;
        p234_latitude_SET((int32_t)443466467, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -26099, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1540720433, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)12, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float) -2.4024721E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)338702951L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)363923887L, PH.base.pack) ;
        p241_vibration_x_SET((float) -8.515018E37F, PH.base.pack) ;
        p241_vibration_z_SET((float)2.8215945E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)102150766099705419L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2723536242L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_x_SET((float)6.9779435E37F, PH.base.pack) ;
        p242_longitude_SET((int32_t)1899541490, PH.base.pack) ;
        p242_z_SET((float)1.3292455E38F, PH.base.pack) ;
        {
            float q[] =  {-2.241348E38F, -1.4646941E38F, -1.554121E38F, 2.0830424E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_x_SET((float)3.192588E36F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1529311154, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)2058490015811971295L, &PH) ;
        p242_y_SET((float) -2.196132E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -1231311054, PH.base.pack) ;
        p242_approach_z_SET((float)2.6822118E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -3.2049142E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_time_usec_SET((uint64_t)8412700632347070341L, &PH) ;
        p243_y_SET((float)1.7071341E38F, PH.base.pack) ;
        {
            float q[] =  {-1.6627198E38F, 4.6506455E37F, -4.6419905E37F, 2.2674176E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_x_SET((float) -1.9379621E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -246239054, PH.base.pack) ;
        p243_z_SET((float)8.915454E37F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p243_latitude_SET((int32_t) -716059091, PH.base.pack) ;
        p243_x_SET((float) -1.7285609E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)700910554, PH.base.pack) ;
        p243_approach_z_SET((float) -1.0257324E38F, PH.base.pack) ;
        p243_approach_y_SET((float)3.0097032E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1815403127, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)19708, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_hor_velocity_SET((uint16_t)(uint16_t)14554, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS |
                        e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN), PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -21430, PH.base.pack) ;
        p246_lon_SET((int32_t)1740151320, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3, PH.base.pack) ;
        p246_lat_SET((int32_t)5219191, PH.base.pack) ;
        {
            char16_t* callsign = u"qrxr";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_heading_SET((uint16_t)(uint16_t)20109, PH.base.pack) ;
        p246_altitude_SET((int32_t)1461243741, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)352631844L, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)52172, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -1.3410168E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)2501673281L, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)3.3540406E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -9.081996E37F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)44, (uint8_t)200, (uint8_t)95, (uint8_t)149, (uint8_t)170, (uint8_t)30, (uint8_t)172, (uint8_t)227, (uint8_t)19, (uint8_t)200, (uint8_t)69, (uint8_t)209, (uint8_t)205, (uint8_t)239, (uint8_t)25, (uint8_t)112, (uint8_t)64, (uint8_t)69, (uint8_t)220, (uint8_t)191, (uint8_t)185, (uint8_t)66, (uint8_t)58, (uint8_t)124, (uint8_t)66, (uint8_t)146, (uint8_t)106, (uint8_t)3, (uint8_t)173, (uint8_t)46, (uint8_t)75, (uint8_t)66, (uint8_t)50, (uint8_t)110, (uint8_t)69, (uint8_t)250, (uint8_t)66, (uint8_t)166, (uint8_t)211, (uint8_t)157, (uint8_t)60, (uint8_t)117, (uint8_t)102, (uint8_t)144, (uint8_t)55, (uint8_t)224, (uint8_t)209, (uint8_t)229, (uint8_t)15, (uint8_t)166, (uint8_t)209, (uint8_t)72, (uint8_t)46, (uint8_t)76, (uint8_t)164, (uint8_t)211, (uint8_t)60, (uint8_t)16, (uint8_t)7, (uint8_t)115, (uint8_t)186, (uint8_t)232, (uint8_t)146, (uint8_t)158, (uint8_t)117, (uint8_t)247, (uint8_t)18, (uint8_t)158, (uint8_t)140, (uint8_t)228, (uint8_t)204, (uint8_t)235, (uint8_t)151, (uint8_t)240, (uint8_t)218, (uint8_t)16, (uint8_t)103, (uint8_t)236, (uint8_t)119, (uint8_t)184, (uint8_t)137, (uint8_t)42, (uint8_t)146, (uint8_t)78, (uint8_t)80, (uint8_t)197, (uint8_t)30, (uint8_t)125, (uint8_t)224, (uint8_t)55, (uint8_t)167, (uint8_t)229, (uint8_t)188, (uint8_t)92, (uint8_t)33, (uint8_t)185, (uint8_t)19, (uint8_t)248, (uint8_t)61, (uint8_t)179, (uint8_t)186, (uint8_t)146, (uint8_t)78, (uint8_t)255, (uint8_t)152, (uint8_t)15, (uint8_t)215, (uint8_t)207, (uint8_t)199, (uint8_t)172, (uint8_t)69, (uint8_t)29, (uint8_t)198, (uint8_t)96, (uint8_t)62, (uint8_t)145, (uint8_t)87, (uint8_t)180, (uint8_t)94, (uint8_t)122, (uint8_t)3, (uint8_t)43, (uint8_t)84, (uint8_t)252, (uint8_t)134, (uint8_t)218, (uint8_t)188, (uint8_t)103, (uint8_t)205, (uint8_t)112, (uint8_t)219, (uint8_t)217, (uint8_t)10, (uint8_t)105, (uint8_t)162, (uint8_t)60, (uint8_t)138, (uint8_t)75, (uint8_t)171, (uint8_t)209, (uint8_t)9, (uint8_t)117, (uint8_t)155, (uint8_t)85, (uint8_t)142, (uint8_t)81, (uint8_t)38, (uint8_t)151, (uint8_t)90, (uint8_t)161, (uint8_t)85, (uint8_t)187, (uint8_t)241, (uint8_t)24, (uint8_t)245, (uint8_t)9, (uint8_t)52, (uint8_t)63, (uint8_t)94, (uint8_t)175, (uint8_t)90, (uint8_t)73, (uint8_t)77, (uint8_t)169, (uint8_t)6, (uint8_t)51, (uint8_t)212, (uint8_t)210, (uint8_t)75, (uint8_t)3, (uint8_t)29, (uint8_t)137, (uint8_t)66, (uint8_t)94, (uint8_t)125, (uint8_t)29, (uint8_t)76, (uint8_t)132, (uint8_t)3, (uint8_t)11, (uint8_t)163, (uint8_t)57, (uint8_t)151, (uint8_t)177, (uint8_t)60, (uint8_t)254, (uint8_t)148, (uint8_t)1, (uint8_t)7, (uint8_t)184, (uint8_t)95, (uint8_t)177, (uint8_t)240, (uint8_t)176, (uint8_t)68, (uint8_t)122, (uint8_t)249, (uint8_t)55, (uint8_t)145, (uint8_t)56, (uint8_t)132, (uint8_t)146, (uint8_t)140, (uint8_t)38, (uint8_t)43, (uint8_t)169, (uint8_t)125, (uint8_t)125, (uint8_t)216, (uint8_t)9, (uint8_t)68, (uint8_t)188, (uint8_t)91, (uint8_t)97, (uint8_t)35, (uint8_t)227, (uint8_t)96, (uint8_t)142, (uint8_t)74, (uint8_t)151, (uint8_t)72, (uint8_t)107, (uint8_t)6, (uint8_t)192, (uint8_t)72, (uint8_t)5, (uint8_t)201, (uint8_t)2, (uint8_t)31, (uint8_t)46, (uint8_t)223, (uint8_t)97, (uint8_t)233, (uint8_t)128, (uint8_t)178, (uint8_t)119, (uint8_t)138, (uint8_t)147, (uint8_t)241, (uint8_t)42, (uint8_t)72, (uint8_t)37, (uint8_t)26, (uint8_t)131, (uint8_t)126, (uint8_t)220, (uint8_t)254, (uint8_t)176, (uint8_t)26};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)22652, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)22052, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)17, (int8_t)111, (int8_t) -127, (int8_t)19, (int8_t) -81, (int8_t) -67, (int8_t) -53, (int8_t) -37, (int8_t)118, (int8_t)118, (int8_t)8, (int8_t)14, (int8_t) -16, (int8_t) -64, (int8_t) -44, (int8_t)7, (int8_t) -56, (int8_t)25, (int8_t) -26, (int8_t)89, (int8_t)43, (int8_t) -68, (int8_t)20, (int8_t) -109, (int8_t)44, (int8_t) -76, (int8_t)47, (int8_t) -79, (int8_t)51, (int8_t)59, (int8_t) -115, (int8_t) -114};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float) -2.734662E38F, PH.base.pack) ;
        p250_x_SET((float) -1.2869257E38F, PH.base.pack) ;
        {
            char16_t* name = u"voxxlO";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)6636738375043034924L, PH.base.pack) ;
        p250_z_SET((float)2.6624003E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float)2.9730526E38F, PH.base.pack) ;
        {
            char16_t* name = u"cqxmu";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)1451395895L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)1987941009L, PH.base.pack) ;
        p252_value_SET((int32_t)1836418142, PH.base.pack) ;
        {
            char16_t* name = u"p";
            p252_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ALERT, PH.base.pack) ;
        {
            char16_t* text = u"yqunhyhavikspzUcktdeBmzVcPkeqlYHujcgqtlqiczky";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float) -2.3185873E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)13700161L, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)7069931489992666373L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)81, (uint8_t)164, (uint8_t)31, (uint8_t)163, (uint8_t)239, (uint8_t)1, (uint8_t)204, (uint8_t)17, (uint8_t)104, (uint8_t)7, (uint8_t)94, (uint8_t)20, (uint8_t)201, (uint8_t)36, (uint8_t)36, (uint8_t)178, (uint8_t)156, (uint8_t)212, (uint8_t)46, (uint8_t)74, (uint8_t)92, (uint8_t)37, (uint8_t)8, (uint8_t)220, (uint8_t)28, (uint8_t)24, (uint8_t)194, (uint8_t)102, (uint8_t)194, (uint8_t)143, (uint8_t)107, (uint8_t)62};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)1517509949L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)248432746L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        {
            char16_t* tune = u"iSgokgdivoqizsvUwrbckaxdear";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_focal_length_SET((float) -3.3786545E38F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)58094, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)2397919213L, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES), PH.base.pack) ;
        p259_sensor_size_h_SET((float)2.1811795E37F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)57112, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)33424, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)98, (uint8_t)145, (uint8_t)178, (uint8_t)196, (uint8_t)157, (uint8_t)34, (uint8_t)59, (uint8_t)240, (uint8_t)181, (uint8_t)206, (uint8_t)42, (uint8_t)70, (uint8_t)58, (uint8_t)80, (uint8_t)206, (uint8_t)205, (uint8_t)27, (uint8_t)21, (uint8_t)20, (uint8_t)66, (uint8_t)54, (uint8_t)204, (uint8_t)138, (uint8_t)202, (uint8_t)100, (uint8_t)73, (uint8_t)178, (uint8_t)32, (uint8_t)168, (uint8_t)87, (uint8_t)116, (uint8_t)6};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        {
            uint8_t vendor_name[] =  {(uint8_t)28, (uint8_t)130, (uint8_t)16, (uint8_t)48, (uint8_t)224, (uint8_t)69, (uint8_t)242, (uint8_t)28, (uint8_t)84, (uint8_t)111, (uint8_t)79, (uint8_t)33, (uint8_t)28, (uint8_t)127, (uint8_t)144, (uint8_t)35, (uint8_t)51, (uint8_t)0, (uint8_t)226, (uint8_t)237, (uint8_t)117, (uint8_t)104, (uint8_t)98, (uint8_t)53, (uint8_t)134, (uint8_t)159, (uint8_t)82, (uint8_t)6, (uint8_t)160, (uint8_t)8, (uint8_t)165, (uint8_t)171};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)1.5449298E38F, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1984102429L, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"bjSirfjyKfcxljqyonqmtSsoezoxtbpzdyjhvzgrnsnkKHgtpOzpqtrpknlyfMywKbwueebcdYlh";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)361653200L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_count_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p261_used_capacity_SET((float)1.3585224E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2664488226L, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.3453559E37F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p261_read_speed_SET((float) -1.7810572E38F, PH.base.pack) ;
        p261_write_speed_SET((float)6.554171E37F, PH.base.pack) ;
        p261_total_capacity_SET((float)1.1795783E37F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)4260244672L, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2571320694L, PH.base.pack) ;
        p262_image_interval_SET((float) -2.1807178E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p262_available_capacity_SET((float)2.9173583E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_alt_SET((int32_t) -2070631102, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)6264861376725511357L, PH.base.pack) ;
        p263_lon_SET((int32_t) -939901184, PH.base.pack) ;
        p263_image_index_SET((int32_t) -181579124, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -872125824, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)884502443L, PH.base.pack) ;
        {
            float q[] =  {9.043213E36F, 9.618361E37F, 2.504039E38F, -5.3613477E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_lat_SET((int32_t) -1641479557, PH.base.pack) ;
        {
            char16_t* file_url = u"npTyvAtqjtrqchqypbweinzMdjTuyRFivfrbkzinvofGejwzrwpgqjhHurhOevcSPqjQsnxdjcocmopkkVlwtqLrcswxichnttaixqigjnBvmjgvyraEncpMwlcqprkdsjnsgyBhaixownjhbyygovXfjznfMicleprhr";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -80, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)393866942971931543L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)3250446628L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)8723619008563835978L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)8297060963220899421L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float) -2.5912871E38F, PH.base.pack) ;
        p265_roll_SET((float)1.9176168E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1968370552L, PH.base.pack) ;
        p265_yaw_SET((float) -1.1503992E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)50732, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)60, (uint8_t)20, (uint8_t)192, (uint8_t)120, (uint8_t)58, (uint8_t)38, (uint8_t)63, (uint8_t)74, (uint8_t)111, (uint8_t)42, (uint8_t)244, (uint8_t)96, (uint8_t)99, (uint8_t)130, (uint8_t)82, (uint8_t)52, (uint8_t)75, (uint8_t)129, (uint8_t)65, (uint8_t)200, (uint8_t)201, (uint8_t)24, (uint8_t)193, (uint8_t)94, (uint8_t)132, (uint8_t)230, (uint8_t)39, (uint8_t)175, (uint8_t)49, (uint8_t)142, (uint8_t)153, (uint8_t)203, (uint8_t)73, (uint8_t)8, (uint8_t)228, (uint8_t)204, (uint8_t)24, (uint8_t)182, (uint8_t)23, (uint8_t)67, (uint8_t)16, (uint8_t)158, (uint8_t)236, (uint8_t)22, (uint8_t)222, (uint8_t)115, (uint8_t)214, (uint8_t)73, (uint8_t)252, (uint8_t)180, (uint8_t)190, (uint8_t)232, (uint8_t)25, (uint8_t)166, (uint8_t)238, (uint8_t)189, (uint8_t)234, (uint8_t)58, (uint8_t)98, (uint8_t)45, (uint8_t)53, (uint8_t)175, (uint8_t)119, (uint8_t)207, (uint8_t)25, (uint8_t)191, (uint8_t)196, (uint8_t)75, (uint8_t)232, (uint8_t)249, (uint8_t)159, (uint8_t)75, (uint8_t)150, (uint8_t)104, (uint8_t)34, (uint8_t)246, (uint8_t)71, (uint8_t)92, (uint8_t)164, (uint8_t)54, (uint8_t)92, (uint8_t)71, (uint8_t)98, (uint8_t)61, (uint8_t)159, (uint8_t)130, (uint8_t)125, (uint8_t)192, (uint8_t)73, (uint8_t)97, (uint8_t)77, (uint8_t)136, (uint8_t)110, (uint8_t)185, (uint8_t)202, (uint8_t)108, (uint8_t)142, (uint8_t)51, (uint8_t)123, (uint8_t)156, (uint8_t)158, (uint8_t)142, (uint8_t)178, (uint8_t)55, (uint8_t)121, (uint8_t)228, (uint8_t)170, (uint8_t)248, (uint8_t)255, (uint8_t)198, (uint8_t)129, (uint8_t)31, (uint8_t)32, (uint8_t)43, (uint8_t)76, (uint8_t)108, (uint8_t)131, (uint8_t)142, (uint8_t)172, (uint8_t)206, (uint8_t)14, (uint8_t)174, (uint8_t)126, (uint8_t)58, (uint8_t)143, (uint8_t)61, (uint8_t)26, (uint8_t)102, (uint8_t)100, (uint8_t)207, (uint8_t)248, (uint8_t)190, (uint8_t)2, (uint8_t)111, (uint8_t)182, (uint8_t)82, (uint8_t)185, (uint8_t)205, (uint8_t)209, (uint8_t)75, (uint8_t)250, (uint8_t)237, (uint8_t)116, (uint8_t)75, (uint8_t)175, (uint8_t)19, (uint8_t)251, (uint8_t)58, (uint8_t)106, (uint8_t)148, (uint8_t)13, (uint8_t)211, (uint8_t)173, (uint8_t)3, (uint8_t)161, (uint8_t)229, (uint8_t)202, (uint8_t)183, (uint8_t)133, (uint8_t)147, (uint8_t)95, (uint8_t)225, (uint8_t)251, (uint8_t)240, (uint8_t)245, (uint8_t)161, (uint8_t)54, (uint8_t)1, (uint8_t)16, (uint8_t)4, (uint8_t)209, (uint8_t)222, (uint8_t)174, (uint8_t)31, (uint8_t)248, (uint8_t)48, (uint8_t)188, (uint8_t)59, (uint8_t)49, (uint8_t)113, (uint8_t)159, (uint8_t)27, (uint8_t)34, (uint8_t)26, (uint8_t)250, (uint8_t)28, (uint8_t)119, (uint8_t)157, (uint8_t)174, (uint8_t)247, (uint8_t)24, (uint8_t)34, (uint8_t)165, (uint8_t)44, (uint8_t)0, (uint8_t)18, (uint8_t)12, (uint8_t)181, (uint8_t)88, (uint8_t)61, (uint8_t)116, (uint8_t)81, (uint8_t)145, (uint8_t)30, (uint8_t)89, (uint8_t)205, (uint8_t)98, (uint8_t)15, (uint8_t)24, (uint8_t)82, (uint8_t)206, (uint8_t)172, (uint8_t)227, (uint8_t)66, (uint8_t)96, (uint8_t)77, (uint8_t)58, (uint8_t)162, (uint8_t)83, (uint8_t)213, (uint8_t)179, (uint8_t)170, (uint8_t)38, (uint8_t)155, (uint8_t)42, (uint8_t)60, (uint8_t)173, (uint8_t)133, (uint8_t)71, (uint8_t)173, (uint8_t)86, (uint8_t)18, (uint8_t)250, (uint8_t)143, (uint8_t)69, (uint8_t)119, (uint8_t)206, (uint8_t)198, (uint8_t)189, (uint8_t)156, (uint8_t)135, (uint8_t)7, (uint8_t)167, (uint8_t)58, (uint8_t)224, (uint8_t)86, (uint8_t)160, (uint8_t)75, (uint8_t)205};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)207, (uint8_t)141, (uint8_t)161, (uint8_t)64, (uint8_t)170, (uint8_t)138, (uint8_t)10, (uint8_t)120, (uint8_t)217, (uint8_t)91, (uint8_t)114, (uint8_t)102, (uint8_t)241, (uint8_t)58, (uint8_t)69, (uint8_t)194, (uint8_t)246, (uint8_t)77, (uint8_t)36, (uint8_t)186, (uint8_t)43, (uint8_t)4, (uint8_t)197, (uint8_t)147, (uint8_t)118, (uint8_t)189, (uint8_t)38, (uint8_t)52, (uint8_t)48, (uint8_t)126, (uint8_t)111, (uint8_t)76, (uint8_t)204, (uint8_t)96, (uint8_t)226, (uint8_t)159, (uint8_t)212, (uint8_t)100, (uint8_t)47, (uint8_t)140, (uint8_t)224, (uint8_t)23, (uint8_t)232, (uint8_t)70, (uint8_t)150, (uint8_t)6, (uint8_t)122, (uint8_t)213, (uint8_t)224, (uint8_t)53, (uint8_t)230, (uint8_t)74, (uint8_t)48, (uint8_t)245, (uint8_t)16, (uint8_t)26, (uint8_t)77, (uint8_t)4, (uint8_t)147, (uint8_t)67, (uint8_t)151, (uint8_t)197, (uint8_t)121, (uint8_t)117, (uint8_t)44, (uint8_t)127, (uint8_t)39, (uint8_t)200, (uint8_t)203, (uint8_t)237, (uint8_t)234, (uint8_t)186, (uint8_t)172, (uint8_t)239, (uint8_t)206, (uint8_t)186, (uint8_t)135, (uint8_t)190, (uint8_t)177, (uint8_t)135, (uint8_t)195, (uint8_t)245, (uint8_t)53, (uint8_t)250, (uint8_t)79, (uint8_t)221, (uint8_t)93, (uint8_t)148, (uint8_t)19, (uint8_t)156, (uint8_t)94, (uint8_t)104, (uint8_t)114, (uint8_t)233, (uint8_t)9, (uint8_t)212, (uint8_t)75, (uint8_t)245, (uint8_t)199, (uint8_t)255, (uint8_t)165, (uint8_t)240, (uint8_t)143, (uint8_t)155, (uint8_t)244, (uint8_t)153, (uint8_t)92, (uint8_t)102, (uint8_t)144, (uint8_t)135, (uint8_t)227, (uint8_t)72, (uint8_t)142, (uint8_t)43, (uint8_t)2, (uint8_t)225, (uint8_t)48, (uint8_t)66, (uint8_t)228, (uint8_t)105, (uint8_t)74, (uint8_t)189, (uint8_t)133, (uint8_t)35, (uint8_t)133, (uint8_t)213, (uint8_t)207, (uint8_t)176, (uint8_t)1, (uint8_t)47, (uint8_t)158, (uint8_t)144, (uint8_t)149, (uint8_t)44, (uint8_t)170, (uint8_t)75, (uint8_t)214, (uint8_t)232, (uint8_t)234, (uint8_t)40, (uint8_t)159, (uint8_t)21, (uint8_t)184, (uint8_t)68, (uint8_t)246, (uint8_t)64, (uint8_t)58, (uint8_t)126, (uint8_t)166, (uint8_t)76, (uint8_t)161, (uint8_t)205, (uint8_t)37, (uint8_t)62, (uint8_t)167, (uint8_t)30, (uint8_t)89, (uint8_t)116, (uint8_t)165, (uint8_t)251, (uint8_t)211, (uint8_t)150, (uint8_t)204, (uint8_t)149, (uint8_t)155, (uint8_t)66, (uint8_t)240, (uint8_t)42, (uint8_t)245, (uint8_t)25, (uint8_t)52, (uint8_t)243, (uint8_t)178, (uint8_t)183, (uint8_t)75, (uint8_t)63, (uint8_t)230, (uint8_t)89, (uint8_t)174, (uint8_t)5, (uint8_t)242, (uint8_t)50, (uint8_t)22, (uint8_t)78, (uint8_t)155, (uint8_t)190, (uint8_t)193, (uint8_t)89, (uint8_t)159, (uint8_t)135, (uint8_t)156, (uint8_t)146, (uint8_t)227, (uint8_t)136, (uint8_t)212, (uint8_t)163, (uint8_t)248, (uint8_t)5, (uint8_t)6, (uint8_t)124, (uint8_t)157, (uint8_t)13, (uint8_t)151, (uint8_t)247, (uint8_t)59, (uint8_t)84, (uint8_t)154, (uint8_t)176, (uint8_t)179, (uint8_t)164, (uint8_t)226, (uint8_t)207, (uint8_t)228, (uint8_t)71, (uint8_t)109, (uint8_t)77, (uint8_t)164, (uint8_t)60, (uint8_t)162, (uint8_t)69, (uint8_t)238, (uint8_t)29, (uint8_t)34, (uint8_t)9, (uint8_t)241, (uint8_t)129, (uint8_t)110, (uint8_t)194, (uint8_t)36, (uint8_t)47, (uint8_t)104, (uint8_t)66, (uint8_t)124, (uint8_t)61, (uint8_t)217, (uint8_t)202, (uint8_t)208, (uint8_t)16, (uint8_t)254, (uint8_t)211, (uint8_t)49, (uint8_t)42, (uint8_t)75, (uint8_t)254, (uint8_t)255, (uint8_t)55, (uint8_t)25, (uint8_t)195, (uint8_t)165};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_first_message_offset_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)16880, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)6480, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_status_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)1315, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)54097, PH.base.pack) ;
        {
            char16_t* uri = u"ndmvqiqbkwzvfudveShcqsgdufQltdrgpgyhetetontkdpwtptnxikzkBkewoudadruyzfzmoishnoqggjispmRbuaswkzUrmxayrqfQntqgxhqdpwnQfuqfbsd";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_rotation_SET((uint16_t)(uint16_t)1556, PH.base.pack) ;
        p269_framerate_SET((float) -2.8526972E38F, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1760251105L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p270_framerate_SET((float)3.6937987E37F, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)534572900L, PH.base.pack) ;
        {
            char16_t* uri = u"LWwtbOapescxrqxHyuydcdzikoifeukxjknorpkOxgvdLFnsfcoaoeptcpyGeokuurwAmyq";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_resolution_v_SET((uint16_t)(uint16_t)37532, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)18379, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)22842, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"fuqcphazqamGgggeeaeitiAovvhsv";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"czbomrilwzSfbKV";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)34863, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)26561, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)210, (uint8_t)141, (uint8_t)172, (uint8_t)219, (uint8_t)67, (uint8_t)12, (uint8_t)161, (uint8_t)147};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t library_version_hash[] =  {(uint8_t)179, (uint8_t)139, (uint8_t)183, (uint8_t)49, (uint8_t)246, (uint8_t)18, (uint8_t)115, (uint8_t)60};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_version_SET((uint16_t)(uint16_t)28061, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1217254271L, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)5333500663778041370L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)16592, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_major_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)745403811L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)2600777009L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        {
            char16_t* name = u"ljowpkuSqqsgqgbwujnmjcndxfffqfpMcTeammYgwFMskxxaJefljhV";
            p311_name_SET_(name, &PH) ;
        }
        p311_time_usec_SET((uint64_t)615327438749491127L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)203, (uint8_t)20, (uint8_t)2, (uint8_t)223, (uint8_t)93, (uint8_t)26, (uint8_t)62, (uint8_t)81, (uint8_t)138, (uint8_t)43, (uint8_t)143, (uint8_t)156, (uint8_t)120, (uint8_t)62, (uint8_t)147, (uint8_t)98};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        {
            char16_t* param_id = u"vuveuhzn";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_component_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)28639, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_value = u"nemaydqdjceVqRzwfalffcXgaxiqihiswfwabmptfsmtqnfouCydiuXsbjmVvcvwfkfytpD";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)36817, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"vweykdrgsS";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)12828, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        {
            char16_t* param_id = u"dvhzgtoeaowpl";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"negybpKaUsizrabggfnCyprwexrcmcjuvbbikzjbylosqqILuhfkvwztogSnrlfubzDjaerxAcxlcvyimuwhSonPza";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        {
            char16_t* param_value = u"RDijgcmtptuzMmjcemfyorfjyresfjmfdkpREckronxexbtTelvupdj";
            p324_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"gdmxycw";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)42708, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)36144, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)7682875618822851657L, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)35547, (uint16_t)16697, (uint16_t)17731, (uint16_t)48042, (uint16_t)64701, (uint16_t)4580, (uint16_t)55232, (uint16_t)65160, (uint16_t)49744, (uint16_t)59657, (uint16_t)32268, (uint16_t)25632, (uint16_t)17206, (uint16_t)62478, (uint16_t)24094, (uint16_t)45392, (uint16_t)54233, (uint16_t)31202, (uint16_t)44901, (uint16_t)21861, (uint16_t)29632, (uint16_t)30207, (uint16_t)50913, (uint16_t)72, (uint16_t)34364, (uint16_t)60588, (uint16_t)45897, (uint16_t)38773, (uint16_t)50260, (uint16_t)2699, (uint16_t)55723, (uint16_t)53582, (uint16_t)52249, (uint16_t)36371, (uint16_t)31192, (uint16_t)10840, (uint16_t)11093, (uint16_t)21960, (uint16_t)22371, (uint16_t)58154, (uint16_t)12591, (uint16_t)14220, (uint16_t)19339, (uint16_t)21994, (uint16_t)6727, (uint16_t)38994, (uint16_t)37678, (uint16_t)60140, (uint16_t)44239, (uint16_t)61270, (uint16_t)4398, (uint16_t)7985, (uint16_t)63764, (uint16_t)38638, (uint16_t)40058, (uint16_t)45021, (uint16_t)1288, (uint16_t)33708, (uint16_t)37447, (uint16_t)43715, (uint16_t)28880, (uint16_t)25526, (uint16_t)463, (uint16_t)54819, (uint16_t)31382, (uint16_t)50787, (uint16_t)47446, (uint16_t)39182, (uint16_t)12233, (uint16_t)15493, (uint16_t)11132, (uint16_t)4287};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


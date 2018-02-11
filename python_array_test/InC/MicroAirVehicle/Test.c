
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
    return  _en__G(get_bits(data, 40, 4));
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2423593780L);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED));
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CALIBRATING);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_TRICOPTER);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)70);
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL));
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)36522);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)15082);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN));
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)56378);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)12763);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -1848);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)53468);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)41960);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)51475);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)10801);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3723930155451809036L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2346676012L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)57156);
    assert(p3_afx_GET(pack) == (float) -1.4151404E38F);
    assert(p3_vz_GET(pack) == (float) -2.7409443E38F);
    assert(p3_x_GET(pack) == (float) -1.1879646E38F);
    assert(p3_yaw_GET(pack) == (float)2.1603748E38F);
    assert(p3_vx_GET(pack) == (float) -8.620627E37F);
    assert(p3_vy_GET(pack) == (float)2.8212832E38F);
    assert(p3_y_GET(pack) == (float)1.0188815E38F);
    assert(p3_afy_GET(pack) == (float) -2.633808E38F);
    assert(p3_afz_GET(pack) == (float)2.35496E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2528478284L);
    assert(p3_yaw_rate_GET(pack) == (float) -1.3372801E37F);
    assert(p3_z_GET(pack) == (float) -2.4526413E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)3198213617L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p4_time_usec_GET(pack) == (uint64_t)2810601885153838797L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p5_passkey_LEN(ph) == 24);
    {
        char16_t * exemplary = u"pjdbGtopwinRrxvkxteqqyzl";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)143);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)48);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 13);
    {
        char16_t * exemplary = u"vtwlsuuRtuwkj";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1510002246L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -27769);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p20_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"hnhcildy";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)95);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)17710);
    assert(p22_param_value_GET(pack) == (float)1.1768356E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)16579);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p22_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"ninsiofflh";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Dp";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float) -2.0916126E37F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_h_acc_TRY(ph) == (uint32_t)452352066L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p24_v_acc_TRY(ph) == (uint32_t)3699854308L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -1567502167);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)45671);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)3349082775L);
    assert(p24_time_usec_GET(pack) == (uint64_t)1901153044632798207L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)59847);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)45524);
    assert(p24_alt_GET(pack) == (int32_t) -1121075319);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)45660);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)4128800915L);
    assert(p24_lon_GET(pack) == (int32_t)1422886212);
    assert(p24_lat_GET(pack) == (int32_t) -1668169985);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)55, (uint8_t)247, (uint8_t)222, (uint8_t)110, (uint8_t)115, (uint8_t)147, (uint8_t)62, (uint8_t)98, (uint8_t)209, (uint8_t)238, (uint8_t)210, (uint8_t)221, (uint8_t)192, (uint8_t)251, (uint8_t)237, (uint8_t)194, (uint8_t)220, (uint8_t)159, (uint8_t)123, (uint8_t)245} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)49, (uint8_t)252, (uint8_t)71, (uint8_t)222, (uint8_t)10, (uint8_t)9, (uint8_t)86, (uint8_t)223, (uint8_t)123, (uint8_t)89, (uint8_t)74, (uint8_t)193, (uint8_t)136, (uint8_t)190, (uint8_t)172, (uint8_t)92, (uint8_t)73, (uint8_t)169, (uint8_t)236, (uint8_t)116} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)111, (uint8_t)33, (uint8_t)21, (uint8_t)186, (uint8_t)242, (uint8_t)121, (uint8_t)114, (uint8_t)3, (uint8_t)209, (uint8_t)20, (uint8_t)206, (uint8_t)102, (uint8_t)215, (uint8_t)164, (uint8_t)159, (uint8_t)134, (uint8_t)46, (uint8_t)65, (uint8_t)200, (uint8_t)87} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)16, (uint8_t)207, (uint8_t)24, (uint8_t)154, (uint8_t)67, (uint8_t)99, (uint8_t)20, (uint8_t)44, (uint8_t)18, (uint8_t)30, (uint8_t)237, (uint8_t)73, (uint8_t)25, (uint8_t)149, (uint8_t)213, (uint8_t)208, (uint8_t)151, (uint8_t)13, (uint8_t)67, (uint8_t)128} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)228, (uint8_t)114, (uint8_t)196, (uint8_t)199, (uint8_t)115, (uint8_t)134, (uint8_t)140, (uint8_t)151, (uint8_t)195, (uint8_t)97, (uint8_t)55, (uint8_t)39, (uint8_t)3, (uint8_t)77, (uint8_t)100, (uint8_t)45, (uint8_t)226, (uint8_t)192, (uint8_t)220} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)19);
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -7382);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -1332);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -9157);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)29582);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)20416);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2685151520L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)15292);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -19673);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -10698);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)24753);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -2191);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)23172);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -4007);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)32213);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -12810);
    assert(p27_time_usec_GET(pack) == (uint64_t)6197393346218843766L);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)3536);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)14097);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)2889);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -29894);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)26344);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -19782);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)3527);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)14211);
    assert(p28_time_usec_GET(pack) == (uint64_t)2461501344094065903L);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_diff_GET(pack) == (float) -1.3348324E38F);
    assert(p29_press_abs_GET(pack) == (float) -7.124683E37F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)22886);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1879251614L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float) -5.385155E36F);
    assert(p30_yawspeed_GET(pack) == (float)4.781216E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2505185329L);
    assert(p30_roll_GET(pack) == (float)3.3877881E38F);
    assert(p30_pitch_GET(pack) == (float) -6.8399825E37F);
    assert(p30_pitchspeed_GET(pack) == (float) -3.2025209E38F);
    assert(p30_yaw_GET(pack) == (float) -1.4475818E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float)2.1826538E38F);
    assert(p31_yawspeed_GET(pack) == (float) -3.2742486E38F);
    assert(p31_q2_GET(pack) == (float)1.9824122E38F);
    assert(p31_pitchspeed_GET(pack) == (float) -1.4232041E38F);
    assert(p31_q1_GET(pack) == (float)7.535904E37F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3223719818L);
    assert(p31_q4_GET(pack) == (float) -1.4382043E38F);
    assert(p31_rollspeed_GET(pack) == (float) -1.8005766E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)2955826632L);
    assert(p32_y_GET(pack) == (float) -3.1864793E38F);
    assert(p32_vy_GET(pack) == (float)3.398603E38F);
    assert(p32_vx_GET(pack) == (float) -6.751133E37F);
    assert(p32_x_GET(pack) == (float)2.4762709E38F);
    assert(p32_z_GET(pack) == (float) -2.4086879E38F);
    assert(p32_vz_GET(pack) == (float)1.9283343E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)22294);
    assert(p33_relative_alt_GET(pack) == (int32_t) -1485226810);
    assert(p33_alt_GET(pack) == (int32_t)1665373210);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -1509);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)8203);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)10582653L);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)10676);
    assert(p33_lat_GET(pack) == (int32_t) -123071964);
    assert(p33_lon_GET(pack) == (int32_t) -830532195);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -28363);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -29150);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -7561);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -3651);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -15697);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -29470);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1820205158L);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)19821);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -13873);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)56991);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)856);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1654019221L);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)44441);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)9874);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)55524);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)62532);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)21057);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)54446);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)50076);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)27517);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)64444);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)38152);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)22926);
    assert(p36_time_usec_GET(pack) == (uint32_t)4047470857L);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)12343);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)12858);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)52805);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)31034);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)48388);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)61418);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)20844);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)44614);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)6008);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)4425);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)27136);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)30824);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -13078);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -22423);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)27873);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)225);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p39_param1_GET(pack) == (float)2.871177E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p39_param4_GET(pack) == (float) -2.0028399E38F);
    assert(p39_param2_GET(pack) == (float) -3.2071294E38F);
    assert(p39_z_GET(pack) == (float)2.5264631E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p39_x_GET(pack) == (float) -2.5148338E38F);
    assert(p39_y_GET(pack) == (float) -3.12128E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)58363);
    assert(p39_param3_GET(pack) == (float)2.8806742E36F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)54891);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)47238);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)104);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)34082);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)199);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)28938);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)122);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)25401);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t)245359734);
    assert(p48_longitude_GET(pack) == (int32_t) -1520658198);
    assert(p48_time_usec_TRY(ph) == (uint64_t)5738072015603966005L);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p48_altitude_GET(pack) == (int32_t) -1664395322);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)8420493050355081020L);
    assert(p49_latitude_GET(pack) == (int32_t)3259329);
    assert(p49_longitude_GET(pack) == (int32_t)946025967);
    assert(p49_altitude_GET(pack) == (int32_t) -687916966);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"lcgsnescz";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_min_GET(pack) == (float) -3.3634314E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p50_param_value_max_GET(pack) == (float) -2.9798987E38F);
    assert(p50_scale_GET(pack) == (float) -2.302138E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)27024);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p50_param_value0_GET(pack) == (float)7.3993937E37F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)56096);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2y_GET(pack) == (float) -1.7098745E38F);
    assert(p54_p1y_GET(pack) == (float)2.7241757E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p54_p2z_GET(pack) == (float) -2.93023E38F);
    assert(p54_p1x_GET(pack) == (float) -2.0110962E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p54_p1z_GET(pack) == (float) -1.5259321E38F);
    assert(p54_p2x_GET(pack) == (float)7.3356714E37F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float)5.555786E36F);
    assert(p55_p1z_GET(pack) == (float)3.2840405E38F);
    assert(p55_p2x_GET(pack) == (float)9.624922E37F);
    assert(p55_p2y_GET(pack) == (float)3.3505965E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p55_p1x_GET(pack) == (float) -4.662704E37F);
    assert(p55_p2z_GET(pack) == (float) -5.9860393E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-6.1036905E37F, -9.226096E37F, -1.8048852E37F, -3.151878E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float) -2.2818405E37F);
    assert(p61_yawspeed_GET(pack) == (float)1.6946824E38F);
    {
        float exemplary[] =  {-2.3316161E38F, 2.7826292E38F, 2.0677182E38F, -5.048614E36F, 2.1495892E38F, 3.1818722E38F, 8.095292E37F, -3.6193433E37F, 1.8011133E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)1.020365E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)1671958004058356738L);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -26183);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)9848);
    assert(p62_nav_pitch_GET(pack) == (float)3.0324058E38F);
    assert(p62_aspd_error_GET(pack) == (float)1.7047986E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)30056);
    assert(p62_nav_roll_GET(pack) == (float)2.9787206E37F);
    assert(p62_xtrack_error_GET(pack) == (float)2.7043107E37F);
    assert(p62_alt_error_GET(pack) == (float)2.4276162E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vy_GET(pack) == (float)3.1245013E38F);
    assert(p63_lon_GET(pack) == (int32_t)1862417841);
    assert(p63_vx_GET(pack) == (float) -3.1641494E38F);
    assert(p63_relative_alt_GET(pack) == (int32_t)36525046);
    assert(p63_alt_GET(pack) == (int32_t)1873791501);
    {
        float exemplary[] =  {1.7105948E37F, -2.2748592E38F, 5.563844E37F, -3.7532903E37F, 7.2679647E37F, 8.20828E37F, 3.3512502E38F, -2.7349302E38F, -2.5527846E38F, -6.92839E37F, -4.244514E37F, -1.2660062E38F, -4.039854E37F, -3.3785702E38F, -1.2683408E38F, -2.7305099E38F, -1.9598304E37F, -9.059313E37F, -1.7369308E38F, 1.1610119E38F, -2.1601324E38F, 2.2650294E38F, 3.0616253E38F, 2.4297292E37F, -3.1003614E38F, 1.9600866E38F, -1.597541E38F, 1.6563358E38F, -2.37621E38F, -9.028436E37F, -2.1550437E38F, 3.3867892E38F, 1.4193644E37F, 8.4332104E37F, 1.9516126E38F, -3.0532038E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_time_usec_GET(pack) == (uint64_t)491036778005900441L);
    assert(p63_lat_GET(pack) == (int32_t)1292797653);
    assert(p63_vz_GET(pack) == (float)9.501174E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_z_GET(pack) == (float)2.0222984E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p64_time_usec_GET(pack) == (uint64_t)2183539936051366253L);
    assert(p64_y_GET(pack) == (float)1.228514E38F);
    assert(p64_ay_GET(pack) == (float) -1.8340728E38F);
    assert(p64_vx_GET(pack) == (float) -5.372609E37F);
    {
        float exemplary[] =  {8.900323E36F, -4.934569E37F, 1.4151232E38F, 1.8731655E38F, -1.41341E38F, 2.8232449E38F, -6.778165E36F, -8.0311896E36F, -3.4065075E37F, 7.826317E37F, 9.959434E37F, 9.760693E37F, 1.3285158E38F, -1.2884089E38F, 1.3205762E38F, -9.303471E37F, 6.111504E37F, -2.6802141E38F, 2.6554497E38F, -1.4407731E38F, -1.1644567E38F, 1.2995357E38F, 3.3338572E38F, -5.529965E37F, -2.6099843E37F, 2.9717277E38F, -3.0670752E38F, -5.008336E37F, -1.2418326E38F, 1.798309E38F, 2.6693858E38F, 4.053924E36F, -1.6236939E38F, 4.006538E37F, 1.931586E38F, -3.2853546E38F, -3.151547E38F, 1.3781696E38F, -2.9887543E37F, 3.2031066E38F, 3.0648514E38F, 1.3919331E38F, 1.4667922E37F, -2.7167633E38F, -1.0499182E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vz_GET(pack) == (float) -3.9642847E37F);
    assert(p64_ax_GET(pack) == (float) -1.0177339E38F);
    assert(p64_az_GET(pack) == (float)9.8351464E36F);
    assert(p64_vy_GET(pack) == (float)2.8219142E38F);
    assert(p64_x_GET(pack) == (float)8.902691E37F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)60956);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)54374);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)56877);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)36865);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)13461);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)22375);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)33590);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)36097);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)7246);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2823769843L);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)60670);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)37755);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)64206);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)5116);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)26669);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)22423);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)45400);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)42648);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)10110);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)1227);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)179);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)23701);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)39778);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)9435);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -23752);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -992);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)6392);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)233);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)51664);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)17669);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)43027);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)25869);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)63502);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)44571);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)17808);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)47107);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_LAST);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_param3_GET(pack) == (float)2.3127053E37F);
    assert(p73_x_GET(pack) == (int32_t) -1002998432);
    assert(p73_z_GET(pack) == (float)1.3005415E38F);
    assert(p73_param1_GET(pack) == (float)2.5216972E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)45606);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p73_param4_GET(pack) == (float)1.8484922E38F);
    assert(p73_y_GET(pack) == (int32_t) -772497769);
    assert(p73_param2_GET(pack) == (float) -1.3816249E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float)3.2728167E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)57842);
    assert(p74_alt_GET(pack) == (float) -1.3194194E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)11325);
    assert(p74_climb_GET(pack) == (float) -1.8727408E38F);
    assert(p74_airspeed_GET(pack) == (float) -2.4793459E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_x_GET(pack) == (int32_t) -336322319);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p75_y_GET(pack) == (int32_t) -606817494);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p75_param3_GET(pack) == (float)3.3130981E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p75_param2_GET(pack) == (float) -2.081503E38F);
    assert(p75_param1_GET(pack) == (float)2.2592458E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p75_param4_GET(pack) == (float) -3.1058326E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE);
    assert(p75_z_GET(pack) == (float) -1.6523164E37F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE);
    assert(p76_param5_GET(pack) == (float)1.0482015E38F);
    assert(p76_param3_GET(pack) == (float)1.9542734E38F);
    assert(p76_param4_GET(pack) == (float) -3.2566246E38F);
    assert(p76_param6_GET(pack) == (float)8.0808867E37F);
    assert(p76_param2_GET(pack) == (float) -1.372465E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p76_param7_GET(pack) == (float)5.0429154E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p76_param1_GET(pack) == (float) -9.862346E37F);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t)242936832);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)146);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)217);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)78);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_ACCEPTED);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FOLLOW);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_roll_GET(pack) == (float) -1.3332083E36F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1256731865L);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p81_thrust_GET(pack) == (float)1.2891983E37F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p81_yaw_GET(pack) == (float)3.2356855E38F);
    assert(p81_pitch_GET(pack) == (float)1.6550413E38F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_yaw_rate_GET(pack) == (float)1.2247258E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)782276471L);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)146);
    {
        float exemplary[] =  {-3.4377548E37F, 1.0982684E38F, -4.0808908E37F, 3.1715758E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_thrust_GET(pack) == (float)3.244486E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p82_body_pitch_rate_GET(pack) == (float)2.2063688E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)3.2894028E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float)1.3212564E38F);
    assert(p83_thrust_GET(pack) == (float) -1.9421669E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3435316170L);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p83_body_yaw_rate_GET(pack) == (float) -2.3814743E38F);
    {
        float exemplary[] =  {1.3271474E38F, -3.3206746E38F, 1.6248533E38F, 2.7499048E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_roll_rate_GET(pack) == (float)3.1337517E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afx_GET(pack) == (float) -3.4019769E38F);
    assert(p84_vz_GET(pack) == (float)5.091223E37F);
    assert(p84_yaw_rate_GET(pack) == (float) -5.961509E37F);
    assert(p84_z_GET(pack) == (float)3.1494845E37F);
    assert(p84_afz_GET(pack) == (float) -1.6181975E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p84_x_GET(pack) == (float)1.2235483E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p84_yaw_GET(pack) == (float) -3.0684298E38F);
    assert(p84_y_GET(pack) == (float) -1.3573952E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)51575);
    assert(p84_afy_GET(pack) == (float)1.3454248E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p84_vx_GET(pack) == (float) -2.5595265E38F);
    assert(p84_vy_GET(pack) == (float) -1.6085671E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3763944112L);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lat_int_GET(pack) == (int32_t) -1633200062);
    assert(p86_afx_GET(pack) == (float)2.808481E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)5347);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)4191360729L);
    assert(p86_vy_GET(pack) == (float) -9.080303E37F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p86_yaw_rate_GET(pack) == (float)2.7885547E38F);
    assert(p86_yaw_GET(pack) == (float) -5.786025E37F);
    assert(p86_afy_GET(pack) == (float) -2.6208331E38F);
    assert(p86_vx_GET(pack) == (float) -3.0042222E38F);
    assert(p86_vz_GET(pack) == (float) -2.8060955E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)716856956);
    assert(p86_afz_GET(pack) == (float) -2.0791354E38F);
    assert(p86_alt_GET(pack) == (float) -2.2521218E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lat_int_GET(pack) == (int32_t) -1256593703);
    assert(p87_vx_GET(pack) == (float)2.7500266E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.3384305E38F);
    assert(p87_vz_GET(pack) == (float)1.3264277E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)15037);
    assert(p87_afz_GET(pack) == (float) -4.6088054E37F);
    assert(p87_lon_int_GET(pack) == (int32_t)1334070125);
    assert(p87_vy_GET(pack) == (float)1.8361897E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)1381697407L);
    assert(p87_yaw_GET(pack) == (float) -2.4282827E38F);
    assert(p87_alt_GET(pack) == (float)2.4866887E38F);
    assert(p87_afy_GET(pack) == (float) -1.9642936E38F);
    assert(p87_afx_GET(pack) == (float) -1.6858977E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float) -2.1526117E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3725502360L);
    assert(p89_x_GET(pack) == (float)6.100752E37F);
    assert(p89_roll_GET(pack) == (float)3.0047607E38F);
    assert(p89_y_GET(pack) == (float) -1.755969E38F);
    assert(p89_pitch_GET(pack) == (float)2.7863826E38F);
    assert(p89_z_GET(pack) == (float) -3.042548E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -18542);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)2730);
    assert(p90_pitchspeed_GET(pack) == (float)2.7277356E38F);
    assert(p90_rollspeed_GET(pack) == (float)1.7064033E38F);
    assert(p90_lat_GET(pack) == (int32_t) -108942616);
    assert(p90_pitch_GET(pack) == (float)2.0475099E38F);
    assert(p90_yaw_GET(pack) == (float)2.098659E38F);
    assert(p90_lon_GET(pack) == (int32_t) -872317632);
    assert(p90_alt_GET(pack) == (int32_t)2117404459);
    assert(p90_time_usec_GET(pack) == (uint64_t)5037110464653088812L);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)581);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -11575);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)379);
    assert(p90_roll_GET(pack) == (float)9.75845E37F);
    assert(p90_yawspeed_GET(pack) == (float)2.1761888E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -19738);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux3_GET(pack) == (float) -3.5038157E37F);
    assert(p91_throttle_GET(pack) == (float)1.2278237E36F);
    assert(p91_pitch_elevator_GET(pack) == (float) -2.211536E38F);
    assert(p91_aux4_GET(pack) == (float)1.3518521E37F);
    assert(p91_time_usec_GET(pack) == (uint64_t)5150141168296370973L);
    assert(p91_aux1_GET(pack) == (float) -1.0824082E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p91_yaw_rudder_GET(pack) == (float)8.1900263E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p91_aux2_GET(pack) == (float)5.3108643E36F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.7300052E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)37866);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)36846);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)37938);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)27010);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)32613);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)34445);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)24465);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)53251);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)58311);
    assert(p92_time_usec_GET(pack) == (uint64_t)4774199946926482176L);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)2224);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)29752);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)56381);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.3418887E38F, -1.1743389E38F, -8.890766E37F, -2.6826523E36F, 2.7790467E38F, 3.2753256E38F, -2.8580919E38F, -4.9692147E37F, 3.3057312E38F, 3.0987997E38F, -1.1069859E37F, 1.2913704E38F, 2.3159413E38F, -1.5208455E38F, -1.4629913E38F, -3.0278713E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)7210501783100001950L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)6530728967065960754L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)17818);
    assert(p100_ground_distance_GET(pack) == (float)7.2697683E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)6369831151444093265L);
    assert(p100_flow_rate_y_TRY(ph) == (float) -8.574819E37F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.4268141E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float)7.4707513E37F);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.3167894E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -22147);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_yaw_GET(pack) == (float)2.7734422E38F);
    assert(p101_y_GET(pack) == (float)1.249295E38F);
    assert(p101_roll_GET(pack) == (float) -2.6083477E38F);
    assert(p101_x_GET(pack) == (float) -6.0385317E37F);
    assert(p101_z_GET(pack) == (float)9.93876E37F);
    assert(p101_pitch_GET(pack) == (float)1.9522136E38F);
    assert(p101_usec_GET(pack) == (uint64_t)5108322630986999685L);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_yaw_GET(pack) == (float) -3.6257022E36F);
    assert(p102_y_GET(pack) == (float) -9.148671E37F);
    assert(p102_z_GET(pack) == (float)2.4547164E37F);
    assert(p102_usec_GET(pack) == (uint64_t)2303744813218333173L);
    assert(p102_x_GET(pack) == (float)1.4322023E38F);
    assert(p102_pitch_GET(pack) == (float) -5.568722E37F);
    assert(p102_roll_GET(pack) == (float)3.3733517E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)1429102225769044302L);
    assert(p103_z_GET(pack) == (float) -2.4633603E38F);
    assert(p103_y_GET(pack) == (float)2.2230576E37F);
    assert(p103_x_GET(pack) == (float)2.9143571E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float)2.398004E38F);
    assert(p104_roll_GET(pack) == (float) -2.9553822E38F);
    assert(p104_pitch_GET(pack) == (float)1.9745297E38F);
    assert(p104_x_GET(pack) == (float)4.364791E37F);
    assert(p104_z_GET(pack) == (float)3.1161663E38F);
    assert(p104_y_GET(pack) == (float)6.915259E37F);
    assert(p104_usec_GET(pack) == (uint64_t)8642262507426670032L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_time_usec_GET(pack) == (uint64_t)8885696254686599720L);
    assert(p105_xmag_GET(pack) == (float)7.263052E37F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)32977);
    assert(p105_ygyro_GET(pack) == (float) -2.1482098E38F);
    assert(p105_zmag_GET(pack) == (float)1.1617395E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.2140743E38F);
    assert(p105_zacc_GET(pack) == (float)8.273549E37F);
    assert(p105_ymag_GET(pack) == (float)1.1474489E38F);
    assert(p105_yacc_GET(pack) == (float)3.2445598E37F);
    assert(p105_xgyro_GET(pack) == (float)3.1756004E38F);
    assert(p105_temperature_GET(pack) == (float)7.9227057E37F);
    assert(p105_abs_pressure_GET(pack) == (float)2.6015738E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -8.855815E37F);
    assert(p105_xacc_GET(pack) == (float)2.8243209E38F);
    assert(p105_zgyro_GET(pack) == (float)2.278825E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_usec_GET(pack) == (uint64_t)6005860237821084469L);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2523237366L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)19870);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3621392674L);
    assert(p106_integrated_xgyro_GET(pack) == (float) -2.9427292E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float) -5.1696465E37F);
    assert(p106_integrated_y_GET(pack) == (float) -1.5461865E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p106_integrated_x_GET(pack) == (float) -2.5072466E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -3.402776E38F);
    assert(p106_distance_GET(pack) == (float)2.998685E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zacc_GET(pack) == (float)2.9705319E38F);
    assert(p107_zmag_GET(pack) == (float)1.3147616E38F);
    assert(p107_ymag_GET(pack) == (float)2.9764844E38F);
    assert(p107_ygyro_GET(pack) == (float)3.3283584E38F);
    assert(p107_zgyro_GET(pack) == (float)3.3523732E38F);
    assert(p107_xgyro_GET(pack) == (float)3.307118E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -5.772684E37F);
    assert(p107_yacc_GET(pack) == (float)2.8137665E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)1766133564866033938L);
    assert(p107_xacc_GET(pack) == (float) -1.7955368E38F);
    assert(p107_xmag_GET(pack) == (float) -1.4836642E38F);
    assert(p107_pressure_alt_GET(pack) == (float)1.8410025E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -1.4827752E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)551810L);
    assert(p107_temperature_GET(pack) == (float)3.6846334E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_xgyro_GET(pack) == (float)2.60389E38F);
    assert(p108_alt_GET(pack) == (float) -2.2942235E38F);
    assert(p108_roll_GET(pack) == (float)1.0557409E38F);
    assert(p108_yacc_GET(pack) == (float) -9.3431604E36F);
    assert(p108_pitch_GET(pack) == (float)2.0856302E38F);
    assert(p108_xacc_GET(pack) == (float) -1.4077949E38F);
    assert(p108_ve_GET(pack) == (float) -1.5239533E38F);
    assert(p108_q4_GET(pack) == (float)1.4278976E38F);
    assert(p108_zacc_GET(pack) == (float)2.43498E38F);
    assert(p108_q3_GET(pack) == (float)2.544694E38F);
    assert(p108_q1_GET(pack) == (float)1.0102199E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.6550784E38F);
    assert(p108_yaw_GET(pack) == (float) -2.9591257E38F);
    assert(p108_q2_GET(pack) == (float)2.042374E38F);
    assert(p108_ygyro_GET(pack) == (float)2.4811195E37F);
    assert(p108_vn_GET(pack) == (float) -3.3543414E38F);
    assert(p108_lon_GET(pack) == (float) -3.0327877E38F);
    assert(p108_zgyro_GET(pack) == (float) -8.628874E37F);
    assert(p108_lat_GET(pack) == (float) -1.2751946E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)3.2970909E38F);
    assert(p108_vd_GET(pack) == (float) -3.9569317E36F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)6364);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)31164);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)126, (uint8_t)230, (uint8_t)196, (uint8_t)1, (uint8_t)255, (uint8_t)112, (uint8_t)93, (uint8_t)149, (uint8_t)104, (uint8_t)169, (uint8_t)186, (uint8_t)191, (uint8_t)152, (uint8_t)242, (uint8_t)81, (uint8_t)149, (uint8_t)84, (uint8_t)184, (uint8_t)194, (uint8_t)249, (uint8_t)27, (uint8_t)50, (uint8_t)140, (uint8_t)14, (uint8_t)190, (uint8_t)191, (uint8_t)153, (uint8_t)101, (uint8_t)248, (uint8_t)83, (uint8_t)155, (uint8_t)120, (uint8_t)109, (uint8_t)255, (uint8_t)249, (uint8_t)178, (uint8_t)57, (uint8_t)15, (uint8_t)10, (uint8_t)0, (uint8_t)94, (uint8_t)20, (uint8_t)44, (uint8_t)89, (uint8_t)20, (uint8_t)16, (uint8_t)222, (uint8_t)173, (uint8_t)130, (uint8_t)214, (uint8_t)119, (uint8_t)114, (uint8_t)0, (uint8_t)26, (uint8_t)76, (uint8_t)97, (uint8_t)65, (uint8_t)121, (uint8_t)124, (uint8_t)14, (uint8_t)13, (uint8_t)124, (uint8_t)179, (uint8_t)15, (uint8_t)0, (uint8_t)121, (uint8_t)85, (uint8_t)122, (uint8_t)43, (uint8_t)237, (uint8_t)219, (uint8_t)26, (uint8_t)136, (uint8_t)85, (uint8_t)115, (uint8_t)9, (uint8_t)168, (uint8_t)127, (uint8_t)47, (uint8_t)227, (uint8_t)47, (uint8_t)163, (uint8_t)144, (uint8_t)93, (uint8_t)241, (uint8_t)230, (uint8_t)40, (uint8_t)102, (uint8_t)199, (uint8_t)85, (uint8_t)92, (uint8_t)21, (uint8_t)115, (uint8_t)90, (uint8_t)58, (uint8_t)236, (uint8_t)200, (uint8_t)96, (uint8_t)255, (uint8_t)68, (uint8_t)173, (uint8_t)127, (uint8_t)172, (uint8_t)112, (uint8_t)0, (uint8_t)135, (uint8_t)126, (uint8_t)42, (uint8_t)40, (uint8_t)9, (uint8_t)76, (uint8_t)93, (uint8_t)9, (uint8_t)31, (uint8_t)208, (uint8_t)185, (uint8_t)34, (uint8_t)74, (uint8_t)103, (uint8_t)76, (uint8_t)145, (uint8_t)95, (uint8_t)111, (uint8_t)72, (uint8_t)233, (uint8_t)160, (uint8_t)93, (uint8_t)92, (uint8_t)119, (uint8_t)197, (uint8_t)255, (uint8_t)18, (uint8_t)89, (uint8_t)116, (uint8_t)142, (uint8_t)143, (uint8_t)7, (uint8_t)196, (uint8_t)194, (uint8_t)186, (uint8_t)191, (uint8_t)152, (uint8_t)121, (uint8_t)127, (uint8_t)151, (uint8_t)44, (uint8_t)91, (uint8_t)51, (uint8_t)15, (uint8_t)34, (uint8_t)130, (uint8_t)38, (uint8_t)10, (uint8_t)248, (uint8_t)179, (uint8_t)33, (uint8_t)38, (uint8_t)241, (uint8_t)24, (uint8_t)244, (uint8_t)131, (uint8_t)183, (uint8_t)223, (uint8_t)29, (uint8_t)203, (uint8_t)228, (uint8_t)228, (uint8_t)189, (uint8_t)117, (uint8_t)234, (uint8_t)97, (uint8_t)110, (uint8_t)141, (uint8_t)65, (uint8_t)121, (uint8_t)249, (uint8_t)110, (uint8_t)214, (uint8_t)18, (uint8_t)212, (uint8_t)23, (uint8_t)123, (uint8_t)61, (uint8_t)245, (uint8_t)71, (uint8_t)2, (uint8_t)253, (uint8_t)155, (uint8_t)76, (uint8_t)80, (uint8_t)85, (uint8_t)40, (uint8_t)140, (uint8_t)27, (uint8_t)44, (uint8_t)177, (uint8_t)70, (uint8_t)235, (uint8_t)83, (uint8_t)94, (uint8_t)235, (uint8_t)134, (uint8_t)92, (uint8_t)6, (uint8_t)68, (uint8_t)87, (uint8_t)226, (uint8_t)179, (uint8_t)140, (uint8_t)171, (uint8_t)242, (uint8_t)175, (uint8_t)178, (uint8_t)130, (uint8_t)13, (uint8_t)100, (uint8_t)142, (uint8_t)72, (uint8_t)23, (uint8_t)43, (uint8_t)39, (uint8_t)35, (uint8_t)12, (uint8_t)174, (uint8_t)227, (uint8_t)110, (uint8_t)186, (uint8_t)232, (uint8_t)85, (uint8_t)172, (uint8_t)119, (uint8_t)195, (uint8_t)124, (uint8_t)65, (uint8_t)92, (uint8_t)133, (uint8_t)188, (uint8_t)109, (uint8_t)171, (uint8_t)186, (uint8_t)74, (uint8_t)97, (uint8_t)188, (uint8_t)13, (uint8_t)65, (uint8_t)241, (uint8_t)147, (uint8_t)230, (uint8_t)65, (uint8_t)17, (uint8_t)1} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -3658619433269897817L);
    assert(p111_tc1_GET(pack) == (int64_t) -3253898904876412462L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)699184138294089148L);
    assert(p112_seq_GET(pack) == (uint32_t)4129047937L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lon_GET(pack) == (int32_t)1814575555);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -305);
    assert(p113_alt_GET(pack) == (int32_t)269405207);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p113_lat_GET(pack) == (int32_t)2028392830);
    assert(p113_time_usec_GET(pack) == (uint64_t)8624372883010290511L);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)50152);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)64346);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)30819);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)14140);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -24279);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)19403);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_xgyro_GET(pack) == (float)2.0297577E38F);
    assert(p114_integrated_x_GET(pack) == (float)1.6051558E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1426967680L);
    assert(p114_integrated_zgyro_GET(pack) == (float) -1.2972822E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)3302444927659862800L);
    assert(p114_integrated_y_GET(pack) == (float)2.3014257E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)817634869L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p114_distance_GET(pack) == (float)4.56764E37F);
    assert(p114_integrated_ygyro_GET(pack) == (float) -2.6264007E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -29003);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)18985);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -18974);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)13858);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)32313);
    assert(p115_time_usec_GET(pack) == (uint64_t)8860663836718199574L);
    assert(p115_lon_GET(pack) == (int32_t)2080233522);
    assert(p115_rollspeed_GET(pack) == (float)1.1060738E38F);
    assert(p115_pitchspeed_GET(pack) == (float) -6.5522446E37F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)10226);
    assert(p115_alt_GET(pack) == (int32_t) -108324409);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -1443);
    {
        float exemplary[] =  {-4.255923E37F, 3.3697552E38F, 1.2308578E38F, 2.7591655E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)9980);
    assert(p115_lat_GET(pack) == (int32_t) -1784560112);
    assert(p115_yawspeed_GET(pack) == (float) -3.3140768E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)33166);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)7063);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)14764);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)22305);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)20396);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)27245);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -22881);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -27543);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)17311);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)1068916683L);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)30977);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)43155);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)34194);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)16651);
    assert(p118_size_GET(pack) == (uint32_t)4069491901L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)19136);
    assert(p118_time_utc_GET(pack) == (uint32_t)1316573780L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)48661);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p119_ofs_GET(pack) == (uint32_t)4071583614L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p119_count_GET(pack) == (uint32_t)147677207L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)56077);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)10052);
    {
        uint8_t exemplary[] =  {(uint8_t)45, (uint8_t)135, (uint8_t)12, (uint8_t)67, (uint8_t)58, (uint8_t)49, (uint8_t)51, (uint8_t)162, (uint8_t)53, (uint8_t)193, (uint8_t)147, (uint8_t)63, (uint8_t)71, (uint8_t)65, (uint8_t)0, (uint8_t)106, (uint8_t)91, (uint8_t)152, (uint8_t)83, (uint8_t)254, (uint8_t)64, (uint8_t)227, (uint8_t)14, (uint8_t)127, (uint8_t)189, (uint8_t)19, (uint8_t)23, (uint8_t)149, (uint8_t)250, (uint8_t)223, (uint8_t)162, (uint8_t)220, (uint8_t)231, (uint8_t)97, (uint8_t)27, (uint8_t)100, (uint8_t)164, (uint8_t)140, (uint8_t)89, (uint8_t)224, (uint8_t)170, (uint8_t)105, (uint8_t)224, (uint8_t)68, (uint8_t)219, (uint8_t)215, (uint8_t)187, (uint8_t)131, (uint8_t)216, (uint8_t)228, (uint8_t)172, (uint8_t)33, (uint8_t)27, (uint8_t)100, (uint8_t)185, (uint8_t)82, (uint8_t)65, (uint8_t)50, (uint8_t)56, (uint8_t)39, (uint8_t)101, (uint8_t)127, (uint8_t)34, (uint8_t)41, (uint8_t)242, (uint8_t)232, (uint8_t)165, (uint8_t)252, (uint8_t)96, (uint8_t)255, (uint8_t)137, (uint8_t)249, (uint8_t)72, (uint8_t)225, (uint8_t)219, (uint8_t)226, (uint8_t)239, (uint8_t)16, (uint8_t)11, (uint8_t)180, (uint8_t)214, (uint8_t)65, (uint8_t)227, (uint8_t)229, (uint8_t)189, (uint8_t)219, (uint8_t)81, (uint8_t)124, (uint8_t)75, (uint8_t)162} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)3434369216L);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)87);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)227);
    {
        uint8_t exemplary[] =  {(uint8_t)55, (uint8_t)9, (uint8_t)47, (uint8_t)247, (uint8_t)59, (uint8_t)111, (uint8_t)127, (uint8_t)82, (uint8_t)43, (uint8_t)190, (uint8_t)6, (uint8_t)4, (uint8_t)207, (uint8_t)200, (uint8_t)13, (uint8_t)230, (uint8_t)66, (uint8_t)109, (uint8_t)29, (uint8_t)8, (uint8_t)65, (uint8_t)84, (uint8_t)123, (uint8_t)62, (uint8_t)6, (uint8_t)194, (uint8_t)68, (uint8_t)217, (uint8_t)39, (uint8_t)224, (uint8_t)87, (uint8_t)90, (uint8_t)83, (uint8_t)252, (uint8_t)211, (uint8_t)197, (uint8_t)162, (uint8_t)238, (uint8_t)51, (uint8_t)210, (uint8_t)62, (uint8_t)10, (uint8_t)97, (uint8_t)131, (uint8_t)59, (uint8_t)172, (uint8_t)42, (uint8_t)231, (uint8_t)180, (uint8_t)41, (uint8_t)201, (uint8_t)17, (uint8_t)140, (uint8_t)50, (uint8_t)14, (uint8_t)235, (uint8_t)100, (uint8_t)93, (uint8_t)167, (uint8_t)60, (uint8_t)12, (uint8_t)186, (uint8_t)134, (uint8_t)211, (uint8_t)109, (uint8_t)122, (uint8_t)4, (uint8_t)54, (uint8_t)223, (uint8_t)7, (uint8_t)178, (uint8_t)166, (uint8_t)0, (uint8_t)63, (uint8_t)216, (uint8_t)142, (uint8_t)227, (uint8_t)148, (uint8_t)48, (uint8_t)23, (uint8_t)163, (uint8_t)162, (uint8_t)149, (uint8_t)38, (uint8_t)188, (uint8_t)145, (uint8_t)2, (uint8_t)17, (uint8_t)42, (uint8_t)55, (uint8_t)172, (uint8_t)106, (uint8_t)166, (uint8_t)230, (uint8_t)117, (uint8_t)28, (uint8_t)80, (uint8_t)35, (uint8_t)223, (uint8_t)62, (uint8_t)80, (uint8_t)234, (uint8_t)5, (uint8_t)228, (uint8_t)49, (uint8_t)188, (uint8_t)42, (uint8_t)216, (uint8_t)79, (uint8_t)64} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)104);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_dgps_age_GET(pack) == (uint32_t)2497062888L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)47065);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p124_time_usec_GET(pack) == (uint64_t)642860949678860055L);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)27151);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX);
    assert(p124_lon_GET(pack) == (int32_t) -1502304725);
    assert(p124_alt_GET(pack) == (int32_t) -1329757083);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)50155);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)21674);
    assert(p124_lat_GET(pack) == (int32_t) -124650477);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)382);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)19562);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)19437);
    {
        uint8_t exemplary[] =  {(uint8_t)19, (uint8_t)219, (uint8_t)21, (uint8_t)128, (uint8_t)44, (uint8_t)80, (uint8_t)80, (uint8_t)247, (uint8_t)242, (uint8_t)130, (uint8_t)191, (uint8_t)91, (uint8_t)33, (uint8_t)168, (uint8_t)45, (uint8_t)169, (uint8_t)183, (uint8_t)239, (uint8_t)146, (uint8_t)37, (uint8_t)255, (uint8_t)155, (uint8_t)104, (uint8_t)9, (uint8_t)67, (uint8_t)238, (uint8_t)121, (uint8_t)118, (uint8_t)33, (uint8_t)68, (uint8_t)213, (uint8_t)23, (uint8_t)130, (uint8_t)33, (uint8_t)32, (uint8_t)114, (uint8_t)109, (uint8_t)69, (uint8_t)230, (uint8_t)112, (uint8_t)173, (uint8_t)167, (uint8_t)196, (uint8_t)154, (uint8_t)156, (uint8_t)155, (uint8_t)126, (uint8_t)103, (uint8_t)9, (uint8_t)219, (uint8_t)143, (uint8_t)127, (uint8_t)57, (uint8_t)61, (uint8_t)43, (uint8_t)235, (uint8_t)145, (uint8_t)190, (uint8_t)71, (uint8_t)221, (uint8_t)64, (uint8_t)192, (uint8_t)160, (uint8_t)204, (uint8_t)234, (uint8_t)195, (uint8_t)26, (uint8_t)223, (uint8_t)130, (uint8_t)118} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE));
    assert(p126_baudrate_GET(pack) == (uint32_t)3384153085L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1837940328);
    assert(p127_accuracy_GET(pack) == (uint32_t)946157739L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)694879360);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -719684696);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)2040511750L);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -617679716);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)40353);
    assert(p127_tow_GET(pack) == (uint32_t)2299043188L);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)535434086L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)8961);
    assert(p128_accuracy_GET(pack) == (uint32_t)949540057L);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)2009681827);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)727664551);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1449626114);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1948893964);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p128_tow_GET(pack) == (uint32_t)3408954245L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -8347);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1320672394L);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -31961);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)31616);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)21444);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -30569);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)11108);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -23677);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -5823);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)13993);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)37863);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p130_size_GET(pack) == (uint32_t)1151343869L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)35711);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)48596);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)25246);
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)246, (uint8_t)155, (uint8_t)165, (uint8_t)237, (uint8_t)218, (uint8_t)117, (uint8_t)35, (uint8_t)163, (uint8_t)25, (uint8_t)119, (uint8_t)100, (uint8_t)166, (uint8_t)30, (uint8_t)154, (uint8_t)150, (uint8_t)190, (uint8_t)204, (uint8_t)17, (uint8_t)193, (uint8_t)109, (uint8_t)237, (uint8_t)31, (uint8_t)133, (uint8_t)252, (uint8_t)85, (uint8_t)239, (uint8_t)178, (uint8_t)103, (uint8_t)84, (uint8_t)174, (uint8_t)126, (uint8_t)249, (uint8_t)26, (uint8_t)189, (uint8_t)186, (uint8_t)26, (uint8_t)197, (uint8_t)118, (uint8_t)119, (uint8_t)166, (uint8_t)209, (uint8_t)193, (uint8_t)113, (uint8_t)61, (uint8_t)137, (uint8_t)8, (uint8_t)145, (uint8_t)198, (uint8_t)249, (uint8_t)42, (uint8_t)156, (uint8_t)73, (uint8_t)150, (uint8_t)60, (uint8_t)236, (uint8_t)2, (uint8_t)108, (uint8_t)58, (uint8_t)234, (uint8_t)231, (uint8_t)14, (uint8_t)51, (uint8_t)61, (uint8_t)107, (uint8_t)192, (uint8_t)37, (uint8_t)233, (uint8_t)225, (uint8_t)90, (uint8_t)104, (uint8_t)112, (uint8_t)213, (uint8_t)71, (uint8_t)231, (uint8_t)27, (uint8_t)236, (uint8_t)44, (uint8_t)228, (uint8_t)65, (uint8_t)32, (uint8_t)152, (uint8_t)154, (uint8_t)89, (uint8_t)81, (uint8_t)23, (uint8_t)22, (uint8_t)162, (uint8_t)235, (uint8_t)89, (uint8_t)81, (uint8_t)119, (uint8_t)129, (uint8_t)78, (uint8_t)190, (uint8_t)217, (uint8_t)125, (uint8_t)142, (uint8_t)230, (uint8_t)56, (uint8_t)232, (uint8_t)95, (uint8_t)211, (uint8_t)177, (uint8_t)150, (uint8_t)141, (uint8_t)162, (uint8_t)192, (uint8_t)233, (uint8_t)189, (uint8_t)1, (uint8_t)220, (uint8_t)245, (uint8_t)240, (uint8_t)120, (uint8_t)29, (uint8_t)140, (uint8_t)66, (uint8_t)202, (uint8_t)88, (uint8_t)80, (uint8_t)234, (uint8_t)34, (uint8_t)196, (uint8_t)17, (uint8_t)48, (uint8_t)180, (uint8_t)12, (uint8_t)217, (uint8_t)225, (uint8_t)39, (uint8_t)147, (uint8_t)186, (uint8_t)148, (uint8_t)125, (uint8_t)70, (uint8_t)61, (uint8_t)161, (uint8_t)162, (uint8_t)251, (uint8_t)191, (uint8_t)243, (uint8_t)96, (uint8_t)162, (uint8_t)169, (uint8_t)47, (uint8_t)94, (uint8_t)157, (uint8_t)148, (uint8_t)217, (uint8_t)243, (uint8_t)39, (uint8_t)27, (uint8_t)71, (uint8_t)245, (uint8_t)113, (uint8_t)21, (uint8_t)131, (uint8_t)155, (uint8_t)80, (uint8_t)75, (uint8_t)83, (uint8_t)213, (uint8_t)145, (uint8_t)195, (uint8_t)179, (uint8_t)47, (uint8_t)151, (uint8_t)109, (uint8_t)20, (uint8_t)188, (uint8_t)211, (uint8_t)158, (uint8_t)229, (uint8_t)85, (uint8_t)144, (uint8_t)183, (uint8_t)94, (uint8_t)43, (uint8_t)159, (uint8_t)201, (uint8_t)9, (uint8_t)146, (uint8_t)157, (uint8_t)43, (uint8_t)168, (uint8_t)203, (uint8_t)3, (uint8_t)79, (uint8_t)78, (uint8_t)146, (uint8_t)168, (uint8_t)251, (uint8_t)48, (uint8_t)185, (uint8_t)52, (uint8_t)200, (uint8_t)83, (uint8_t)210, (uint8_t)28, (uint8_t)12, (uint8_t)123, (uint8_t)53, (uint8_t)115, (uint8_t)224, (uint8_t)221, (uint8_t)201, (uint8_t)188, (uint8_t)215, (uint8_t)58, (uint8_t)48, (uint8_t)137, (uint8_t)38, (uint8_t)48, (uint8_t)6, (uint8_t)61, (uint8_t)70, (uint8_t)96, (uint8_t)202, (uint8_t)255, (uint8_t)109, (uint8_t)168, (uint8_t)242, (uint8_t)82, (uint8_t)27, (uint8_t)216, (uint8_t)145, (uint8_t)139, (uint8_t)198, (uint8_t)29, (uint8_t)175, (uint8_t)168, (uint8_t)91, (uint8_t)95, (uint8_t)145, (uint8_t)125, (uint8_t)54, (uint8_t)251, (uint8_t)41, (uint8_t)157, (uint8_t)84, (uint8_t)231, (uint8_t)68, (uint8_t)215, (uint8_t)10, (uint8_t)16, (uint8_t)37, (uint8_t)89, (uint8_t)206, (uint8_t)121, (uint8_t)173, (uint8_t)100, (uint8_t)105} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)34954);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)130);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1747155244L);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)23218);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)2130709184);
    assert(p133_mask_GET(pack) == (uint64_t)8580945693956203757L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)12669);
    assert(p133_lon_GET(pack) == (int32_t)279601308);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lat_GET(pack) == (int32_t)968116785);
    {
        int16_t exemplary[] =  {(int16_t) -21249, (int16_t)26980, (int16_t) -1566, (int16_t)20273, (int16_t)4585, (int16_t) -13536, (int16_t) -31043, (int16_t)2205, (int16_t) -7519, (int16_t) -22443, (int16_t)32571, (int16_t) -29749, (int16_t)32131, (int16_t) -1588, (int16_t)15989, (int16_t) -27791} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)22838);
    assert(p134_lon_GET(pack) == (int32_t)1080098559);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)249);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -1149736305);
    assert(p135_lat_GET(pack) == (int32_t) -1961865049);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float)7.4192497E36F);
    assert(p136_lon_GET(pack) == (int32_t) -1738178040);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)53979);
    assert(p136_current_height_GET(pack) == (float) -2.1234468E38F);
    assert(p136_lat_GET(pack) == (int32_t) -1732168533);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)46340);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)8797);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -11);
    assert(p137_press_diff_GET(pack) == (float) -2.2941567E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3825485953L);
    assert(p137_press_abs_GET(pack) == (float)1.950535E38F);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_z_GET(pack) == (float)1.5758249E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)7362590263729988431L);
    assert(p138_x_GET(pack) == (float) -1.8016398E38F);
    assert(p138_y_GET(pack) == (float)2.997715E37F);
    {
        float exemplary[] =  {4.336294E37F, 2.1302092E38F, 1.2186463E38F, -2.4219775E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)4800704983101110186L);
    {
        float exemplary[] =  {6.658204E37F, 2.4667138E38F, -2.677045E38F, -7.922227E37F, 2.994173E38F, -2.6249302E38F, 2.1902264E38F, 8.051566E36F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)7179024428114352859L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)90);
    {
        float exemplary[] =  {1.88495E38F, -2.6594551E38F, 1.0275828E38F, 8.041979E36F, 2.6825734E38F, 2.2985327E38F, -3.382826E38F, -2.7432764E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_monotonic_GET(pack) == (float)3.2881285E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.5052808E38F);
    assert(p141_altitude_local_GET(pack) == (float)3.3215503E37F);
    assert(p141_altitude_amsl_GET(pack) == (float) -8.119583E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)529910638981686912L);
    assert(p141_altitude_terrain_GET(pack) == (float)1.9944774E38F);
    assert(p141_altitude_relative_GET(pack) == (float)1.136537E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)9, (uint8_t)156, (uint8_t)190, (uint8_t)93, (uint8_t)152, (uint8_t)224, (uint8_t)85, (uint8_t)11, (uint8_t)131, (uint8_t)157, (uint8_t)120, (uint8_t)98, (uint8_t)183, (uint8_t)98, (uint8_t)159, (uint8_t)65, (uint8_t)101, (uint8_t)40, (uint8_t)162, (uint8_t)84, (uint8_t)212, (uint8_t)29, (uint8_t)37, (uint8_t)68, (uint8_t)80, (uint8_t)149, (uint8_t)15, (uint8_t)174, (uint8_t)230, (uint8_t)154, (uint8_t)149, (uint8_t)30, (uint8_t)86, (uint8_t)29, (uint8_t)251, (uint8_t)102, (uint8_t)62, (uint8_t)150, (uint8_t)162, (uint8_t)165, (uint8_t)140, (uint8_t)216, (uint8_t)188, (uint8_t)226, (uint8_t)130, (uint8_t)105, (uint8_t)134, (uint8_t)209, (uint8_t)165, (uint8_t)114, (uint8_t)116, (uint8_t)17, (uint8_t)51, (uint8_t)220, (uint8_t)167, (uint8_t)176, (uint8_t)151, (uint8_t)116, (uint8_t)181, (uint8_t)227, (uint8_t)158, (uint8_t)109, (uint8_t)198, (uint8_t)6, (uint8_t)70, (uint8_t)152, (uint8_t)64, (uint8_t)191, (uint8_t)38, (uint8_t)226, (uint8_t)96, (uint8_t)42, (uint8_t)224, (uint8_t)216, (uint8_t)208, (uint8_t)209, (uint8_t)224, (uint8_t)156, (uint8_t)36, (uint8_t)230, (uint8_t)195, (uint8_t)186, (uint8_t)166, (uint8_t)101, (uint8_t)129, (uint8_t)255, (uint8_t)64, (uint8_t)13, (uint8_t)73, (uint8_t)104, (uint8_t)206, (uint8_t)166, (uint8_t)164, (uint8_t)83, (uint8_t)87, (uint8_t)200, (uint8_t)226, (uint8_t)70, (uint8_t)140, (uint8_t)65, (uint8_t)208, (uint8_t)205, (uint8_t)156, (uint8_t)214, (uint8_t)121, (uint8_t)204, (uint8_t)219, (uint8_t)205, (uint8_t)175, (uint8_t)41, (uint8_t)252, (uint8_t)191, (uint8_t)107, (uint8_t)8, (uint8_t)133, (uint8_t)114, (uint8_t)30, (uint8_t)136, (uint8_t)17} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)160);
    {
        uint8_t exemplary[] =  {(uint8_t)64, (uint8_t)50, (uint8_t)162, (uint8_t)160, (uint8_t)209, (uint8_t)19, (uint8_t)51, (uint8_t)135, (uint8_t)88, (uint8_t)175, (uint8_t)237, (uint8_t)144, (uint8_t)255, (uint8_t)176, (uint8_t)111, (uint8_t)241, (uint8_t)185, (uint8_t)146, (uint8_t)152, (uint8_t)160, (uint8_t)52, (uint8_t)43, (uint8_t)168, (uint8_t)213, (uint8_t)79, (uint8_t)86, (uint8_t)164, (uint8_t)7, (uint8_t)220, (uint8_t)242, (uint8_t)30, (uint8_t)9, (uint8_t)15, (uint8_t)121, (uint8_t)36, (uint8_t)0, (uint8_t)128, (uint8_t)196, (uint8_t)216, (uint8_t)205, (uint8_t)253, (uint8_t)170, (uint8_t)45, (uint8_t)11, (uint8_t)18, (uint8_t)91, (uint8_t)5, (uint8_t)31, (uint8_t)131, (uint8_t)62, (uint8_t)145, (uint8_t)141, (uint8_t)11, (uint8_t)149, (uint8_t)216, (uint8_t)241, (uint8_t)62, (uint8_t)22, (uint8_t)96, (uint8_t)16, (uint8_t)79, (uint8_t)138, (uint8_t)8, (uint8_t)204, (uint8_t)174, (uint8_t)236, (uint8_t)230, (uint8_t)232, (uint8_t)92, (uint8_t)161, (uint8_t)38, (uint8_t)116, (uint8_t)61, (uint8_t)63, (uint8_t)41, (uint8_t)96, (uint8_t)136, (uint8_t)205, (uint8_t)152, (uint8_t)179, (uint8_t)181, (uint8_t)21, (uint8_t)19, (uint8_t)57, (uint8_t)84, (uint8_t)74, (uint8_t)157, (uint8_t)56, (uint8_t)75, (uint8_t)136, (uint8_t)192, (uint8_t)244, (uint8_t)217, (uint8_t)241, (uint8_t)233, (uint8_t)254, (uint8_t)73, (uint8_t)93, (uint8_t)55, (uint8_t)9, (uint8_t)211, (uint8_t)10, (uint8_t)235, (uint8_t)249, (uint8_t)232, (uint8_t)159, (uint8_t)52, (uint8_t)82, (uint8_t)102, (uint8_t)232, (uint8_t)243, (uint8_t)30, (uint8_t)61, (uint8_t)228, (uint8_t)70, (uint8_t)62, (uint8_t)120, (uint8_t)33, (uint8_t)28, (uint8_t)176} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)169);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)324482988L);
    assert(p143_press_diff_GET(pack) == (float) -1.637617E36F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)30048);
    assert(p143_press_abs_GET(pack) == (float)1.7418178E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.5953186E38F, 1.5390273E38F, -1.2524508E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -1266844792);
    {
        float exemplary[] =  {8.957169E37F, 1.1031359E38F, 3.2809542E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)2165679574356749825L);
    assert(p144_custom_state_GET(pack) == (uint64_t)4950983836715804360L);
    assert(p144_lon_GET(pack) == (int32_t)1449170487);
    {
        float exemplary[] =  {-3.363802E38F, 3.2238965E38F, -2.4109483E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {8.1137244E37F, 2.3725779E38F, -7.194086E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)4.993015E37F);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)152);
    {
        float exemplary[] =  {-5.771193E37F, -1.9231081E38F, 3.2567337E38F, -2.616168E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-7.556189E37F, 1.0221413E38F, -2.7653171E38F, -3.0230193E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_roll_rate_GET(pack) == (float)2.4483897E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -2.4975996E38F);
    assert(p146_x_acc_GET(pack) == (float)2.2267078E38F);
    assert(p146_x_vel_GET(pack) == (float)1.4182401E38F);
    {
        float exemplary[] =  {9.584607E37F, 3.069771E38F, -3.2085752E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_pitch_rate_GET(pack) == (float)2.499509E37F);
    assert(p146_x_pos_GET(pack) == (float)3.3645862E36F);
    assert(p146_z_vel_GET(pack) == (float) -1.9671226E38F);
    assert(p146_y_acc_GET(pack) == (float)1.0567986E38F);
    assert(p146_airspeed_GET(pack) == (float)8.596693E36F);
    assert(p146_y_pos_GET(pack) == (float)1.062275E38F);
    assert(p146_z_pos_GET(pack) == (float) -9.432074E37F);
    {
        float exemplary[] =  {-2.3055365E37F, 1.703949E38F, -9.595443E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float) -2.2626255E38F);
    assert(p146_y_vel_GET(pack) == (float)9.965751E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)7600725570938640001L);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_energy_consumed_GET(pack) == (int32_t)1549919477);
    assert(p147_current_consumed_GET(pack) == (int32_t)1950748840);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -102);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    {
        uint16_t exemplary[] =  {(uint16_t)42894, (uint16_t)50044, (uint16_t)29283, (uint16_t)17930, (uint16_t)12515, (uint16_t)14343, (uint16_t)55174, (uint16_t)14309, (uint16_t)44738, (uint16_t)8892} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -24231);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -23223);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)92, (uint8_t)1, (uint8_t)81, (uint8_t)99, (uint8_t)206, (uint8_t)67, (uint8_t)189, (uint8_t)57} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT));
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1174098223L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)54237);
    {
        uint8_t exemplary[] =  {(uint8_t)104, (uint8_t)108, (uint8_t)160, (uint8_t)216, (uint8_t)150, (uint8_t)37, (uint8_t)94, (uint8_t)39} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1181809887L);
    assert(p148_board_version_GET(pack) == (uint32_t)1467254107L);
    {
        uint8_t exemplary[] =  {(uint8_t)169, (uint8_t)7, (uint8_t)94, (uint8_t)77, (uint8_t)239, (uint8_t)57, (uint8_t)229, (uint8_t)57} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)21158);
    assert(p148_uid_GET(pack) == (uint64_t)3583184188896619140L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3751005983L);
    {
        uint8_t exemplary[] =  {(uint8_t)247, (uint8_t)30, (uint8_t)114, (uint8_t)104, (uint8_t)35, (uint8_t)70, (uint8_t)77, (uint8_t)86, (uint8_t)159, (uint8_t)148, (uint8_t)146, (uint8_t)216, (uint8_t)226, (uint8_t)80, (uint8_t)51, (uint8_t)85, (uint8_t)143, (uint8_t)211} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_size_y_GET(pack) == (float)5.7778105E37F);
    {
        float exemplary[] =  {2.5424396E38F, 3.0043088E38F, 3.9621497E37F, -1.2407471E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_angle_y_GET(pack) == (float) -1.6189391E38F);
    assert(p149_distance_GET(pack) == (float) -2.3898528E38F);
    assert(p149_x_TRY(ph) == (float)1.7527618E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p149_angle_x_GET(pack) == (float)2.5695783E38F);
    assert(p149_size_x_GET(pack) == (float) -1.6792512E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)231);
    assert(p149_time_usec_GET(pack) == (uint64_t)5317533209621957290L);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON);
    assert(p149_y_TRY(ph) == (float) -1.753927E38F);
    assert(p149_z_TRY(ph) == (float)1.0720301E38F);
};


void c_CommunicationChannel_on_ARRAY_TEST_0_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_v1_GET(pack) == (uint8_t)(uint8_t)248);
    {
        uint32_t exemplary[] =  {2557929228L, 1541642112L, 2690608397L, 2596955902L} ;
        uint32_t*  sample = p150_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)107, (uint8_t)203, (uint8_t)146, (uint8_t)253} ;
        uint8_t*  sample = p150_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)45416, (uint16_t)32761, (uint16_t)31205, (uint16_t)38254} ;
        uint16_t*  sample = p150_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int8_t exemplary[] =  {(int8_t)63, (int8_t) -70, (int8_t) -80, (int8_t)102} ;
        int8_t*  sample = p150_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_1_151(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {957022200L, 2284539905L, 4258903440L, 4212684547L} ;
        uint32_t*  sample = p151_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_3_153(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {3482472394L, 1753280730L, 4194140847L, 610893673L} ;
        uint32_t*  sample = p153_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p153_v_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_CommunicationChannel_on_ARRAY_TEST_4_154(Bounds_Inside * ph, Pack * pack)
{
    {
        uint32_t exemplary[] =  {3717773926L, 2911425666L, 3231432080L, 3680802255L} ;
        uint32_t*  sample = p154_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p154_v_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_CommunicationChannel_on_ARRAY_TEST_5_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_c2_LEN(ph) == 3);
    {
        char16_t * exemplary = u"Dgv";
        char16_t * sample = p155_c2_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p155_c1_LEN(ph) == 2);
    {
        char16_t * exemplary = u"mp";
        char16_t * sample = p155_c1_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_6_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_ar_c_LEN(ph) == 1);
    {
        char16_t * exemplary = u"n";
        char16_t * sample = p156_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_v3_GET(pack) == (uint32_t)564803386L);
    assert(p156_v1_GET(pack) == (uint8_t)(uint8_t)137);
    {
        uint32_t exemplary[] =  {3640450294L, 1558377688L} ;
        uint32_t*  sample = p156_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)22132, (uint16_t)25664} ;
        uint16_t*  sample = p156_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        double exemplary[] =  {1.2583799612781677E308, 1.298016257020958E308} ;
        double*  sample = p156_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)4} ;
        uint8_t*  sample = p156_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t)15790, (int16_t)23797} ;
        int16_t*  sample = p156_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.3929298E38F, -1.1445463E38F} ;
        float*  sample = p156_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {1582561337, -1947838858} ;
        int32_t*  sample = p156_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p156_v2_GET(pack) == (uint16_t)(uint16_t)29223);
    {
        int8_t exemplary[] =  {(int8_t) -35, (int8_t)17} ;
        int8_t*  sample = p156_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_7_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_ar_c_LEN(ph) == 16);
    {
        char16_t * exemplary = u"yrbtrkerlueiaboF";
        char16_t * sample = p157_ar_c_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int32_t exemplary[] =  {-1466887520, 326322049} ;
        int32_t*  sample = p157_ar_i32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.501923E38F, 3.9260075E37F} ;
        float*  sample = p157_ar_f_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int8_t exemplary[] =  {(int8_t) -100, (int8_t)3} ;
        int8_t*  sample = p157_ar_i8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)158, (uint8_t)95} ;
        uint8_t*  sample = p157_ar_u8_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        double exemplary[] =  {-1.6167808777353305E308, 1.430167869280193E307} ;
        double*  sample = p157_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)36324, (uint16_t)1823} ;
        uint16_t*  sample = p157_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {3621943668L, 2178516584L} ;
        uint32_t*  sample = p157_ar_u32_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        int16_t exemplary[] =  {(int16_t)31758, (int16_t) -19413} ;
        int16_t*  sample = p157_ar_i16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ARRAY_TEST_8_158(Bounds_Inside * ph, Pack * pack)
{
    {
        double exemplary[] =  {-1.42666016685961E308, 4.885377199235113E307} ;
        double*  sample = p158_ar_d_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint16_t exemplary[] =  {(uint16_t)17786, (uint16_t)8819} ;
        uint16_t*  sample = p158_ar_u16_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p158_v3_GET(pack) == (uint32_t)1589646566L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -2.6433677E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.665664E37F);
    assert(p230_hagl_ratio_GET(pack) == (float) -2.6097273E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.2226164E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)1.3260667E38F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS));
    assert(p230_time_usec_GET(pack) == (uint64_t)5508060286975452109L);
    assert(p230_tas_ratio_GET(pack) == (float) -3.1235797E38F);
    assert(p230_mag_ratio_GET(pack) == (float)1.4933814E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -3.1252033E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_x_GET(pack) == (float)2.0469113E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)3.1566484E38F);
    assert(p231_var_horiz_GET(pack) == (float)2.314555E38F);
    assert(p231_wind_alt_GET(pack) == (float)1.6802953E36F);
    assert(p231_wind_y_GET(pack) == (float)1.4998136E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -3.0454711E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)4237176375741762941L);
    assert(p231_wind_z_GET(pack) == (float)3.1608736E38F);
    assert(p231_var_vert_GET(pack) == (float) -1.2695339E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_ve_GET(pack) == (float) -8.942892E37F);
    assert(p232_hdop_GET(pack) == (float)2.9827368E38F);
    assert(p232_vd_GET(pack) == (float) -2.6584266E38F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)49358);
    assert(p232_vert_accuracy_GET(pack) == (float) -9.645408E37F);
    assert(p232_lon_GET(pack) == (int32_t) -231683496);
    assert(p232_speed_accuracy_GET(pack) == (float) -1.374141E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p232_vdop_GET(pack) == (float)2.5886877E38F);
    assert(p232_alt_GET(pack) == (float) -1.5644803E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2409201263L);
    assert(p232_lat_GET(pack) == (int32_t) -506346963);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p232_vn_GET(pack) == (float)3.6820666E37F);
    assert(p232_horiz_accuracy_GET(pack) == (float)2.9597086E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)2879729629079381597L);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)34);
    {
        uint8_t exemplary[] =  {(uint8_t)242, (uint8_t)19, (uint8_t)162, (uint8_t)166, (uint8_t)129, (uint8_t)78, (uint8_t)127, (uint8_t)40, (uint8_t)47, (uint8_t)131, (uint8_t)247, (uint8_t)94, (uint8_t)193, (uint8_t)127, (uint8_t)57, (uint8_t)84, (uint8_t)95, (uint8_t)239, (uint8_t)108, (uint8_t)219, (uint8_t)233, (uint8_t)175, (uint8_t)219, (uint8_t)244, (uint8_t)23, (uint8_t)3, (uint8_t)16, (uint8_t)213, (uint8_t)211, (uint8_t)170, (uint8_t)151, (uint8_t)210, (uint8_t)82, (uint8_t)240, (uint8_t)220, (uint8_t)255, (uint8_t)17, (uint8_t)26, (uint8_t)41, (uint8_t)199, (uint8_t)71, (uint8_t)32, (uint8_t)149, (uint8_t)111, (uint8_t)173, (uint8_t)74, (uint8_t)135, (uint8_t)152, (uint8_t)210, (uint8_t)135, (uint8_t)32, (uint8_t)15, (uint8_t)147, (uint8_t)118, (uint8_t)254, (uint8_t)29, (uint8_t)169, (uint8_t)209, (uint8_t)250, (uint8_t)35, (uint8_t)194, (uint8_t)229, (uint8_t)99, (uint8_t)18, (uint8_t)252, (uint8_t)213, (uint8_t)17, (uint8_t)226, (uint8_t)132, (uint8_t)20, (uint8_t)5, (uint8_t)246, (uint8_t)254, (uint8_t)220, (uint8_t)164, (uint8_t)45, (uint8_t)135, (uint8_t)101, (uint8_t)177, (uint8_t)109, (uint8_t)15, (uint8_t)12, (uint8_t)157, (uint8_t)216, (uint8_t)241, (uint8_t)210, (uint8_t)11, (uint8_t)13, (uint8_t)207, (uint8_t)183, (uint8_t)20, (uint8_t)85, (uint8_t)48, (uint8_t)91, (uint8_t)79, (uint8_t)65, (uint8_t)146, (uint8_t)206, (uint8_t)253, (uint8_t)84, (uint8_t)84, (uint8_t)77, (uint8_t)40, (uint8_t)152, (uint8_t)208, (uint8_t)18, (uint8_t)148, (uint8_t)22, (uint8_t)96, (uint8_t)4, (uint8_t)73, (uint8_t)43, (uint8_t)230, (uint8_t)36, (uint8_t)174, (uint8_t)112, (uint8_t)24, (uint8_t)3, (uint8_t)150, (uint8_t)244, (uint8_t)52, (uint8_t)163, (uint8_t)70, (uint8_t)16, (uint8_t)8, (uint8_t)22, (uint8_t)24, (uint8_t)88, (uint8_t)127, (uint8_t)243, (uint8_t)186, (uint8_t)93, (uint8_t)213, (uint8_t)21, (uint8_t)67, (uint8_t)27, (uint8_t)182, (uint8_t)164, (uint8_t)200, (uint8_t)100, (uint8_t)83, (uint8_t)28, (uint8_t)86, (uint8_t)67, (uint8_t)136, (uint8_t)87, (uint8_t)170, (uint8_t)103, (uint8_t)69, (uint8_t)122, (uint8_t)107, (uint8_t)187, (uint8_t)98, (uint8_t)122, (uint8_t)105, (uint8_t)92, (uint8_t)15, (uint8_t)101, (uint8_t)203, (uint8_t)171, (uint8_t)117, (uint8_t)143, (uint8_t)143, (uint8_t)8, (uint8_t)150, (uint8_t)138, (uint8_t)109, (uint8_t)112, (uint8_t)168, (uint8_t)42, (uint8_t)104, (uint8_t)58, (uint8_t)195, (uint8_t)43, (uint8_t)21, (uint8_t)139, (uint8_t)107, (uint8_t)113, (uint8_t)95, (uint8_t)254} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -20324);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -3635);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED));
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)4831);
    assert(p234_longitude_GET(pack) == (int32_t)1318275495);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)35);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -23);
    assert(p234_latitude_GET(pack) == (int32_t)960973065);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)23698);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)45);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)39731);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -22577);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -20893);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)13);
    assert(p234_custom_mode_GET(pack) == (uint32_t)355444115L);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_y_GET(pack) == (float)1.5704676E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2792677979L);
    assert(p241_vibration_z_GET(pack) == (float) -1.3241156E38F);
    assert(p241_vibration_x_GET(pack) == (float) -1.0868268E37F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)2192051921L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)1401918637L);
    assert(p241_time_usec_GET(pack) == (uint64_t)6462386996895577439L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_z_GET(pack) == (float) -1.149653E38F);
    assert(p242_latitude_GET(pack) == (int32_t)1841036408);
    assert(p242_altitude_GET(pack) == (int32_t) -594570178);
    assert(p242_y_GET(pack) == (float)9.190323E37F);
    assert(p242_longitude_GET(pack) == (int32_t)1255143641);
    {
        float exemplary[] =  {-7.9437185E36F, 2.6790015E38F, -2.600767E38F, 2.4565882E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_x_GET(pack) == (float)7.4853957E37F);
    assert(p242_approach_y_GET(pack) == (float)2.5776476E38F);
    assert(p242_approach_x_GET(pack) == (float) -1.4133221E38F);
    assert(p242_approach_z_GET(pack) == (float)1.6606257E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)4582738122282280312L);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_time_usec_TRY(ph) == (uint64_t)6954490397893475402L);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p243_y_GET(pack) == (float)3.0427342E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.2381238E38F);
    assert(p243_z_GET(pack) == (float)3.1565983E38F);
    assert(p243_approach_y_GET(pack) == (float) -1.1562075E37F);
    assert(p243_altitude_GET(pack) == (int32_t)53032462);
    assert(p243_latitude_GET(pack) == (int32_t) -322633197);
    assert(p243_longitude_GET(pack) == (int32_t) -282051097);
    assert(p243_x_GET(pack) == (float)7.143673E37F);
    {
        float exemplary[] =  {1.8780075E38F, 2.5285873E38F, -3.073129E38F, 1.7413351E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_x_GET(pack) == (float)1.3486973E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)34840);
    assert(p244_interval_us_GET(pack) == (int32_t)1109074266);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)43240);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)47883);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)53243);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)1039754255L);
    assert(p246_altitude_GET(pack) == (int32_t)1174113838);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"odacm";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lon_GET(pack) == (int32_t)1157344715);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -17320);
    assert(p246_lat_GET(pack) == (int32_t)1716184006);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED));
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_altitude_minimum_delta_GET(pack) == (float)2.3378915E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.892997E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.698219E38F);
    assert(p247_threat_level_GET(pack) == (e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW));
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_id_GET(pack) == (uint32_t)3891010012L);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)27798);
    {
        uint8_t exemplary[] =  {(uint8_t)85, (uint8_t)98, (uint8_t)78, (uint8_t)195, (uint8_t)71, (uint8_t)15, (uint8_t)219, (uint8_t)230, (uint8_t)149, (uint8_t)116, (uint8_t)195, (uint8_t)64, (uint8_t)195, (uint8_t)72, (uint8_t)184, (uint8_t)179, (uint8_t)85, (uint8_t)249, (uint8_t)118, (uint8_t)164, (uint8_t)252, (uint8_t)160, (uint8_t)131, (uint8_t)223, (uint8_t)143, (uint8_t)72, (uint8_t)183, (uint8_t)75, (uint8_t)45, (uint8_t)223, (uint8_t)192, (uint8_t)87, (uint8_t)140, (uint8_t)103, (uint8_t)228, (uint8_t)240, (uint8_t)62, (uint8_t)206, (uint8_t)72, (uint8_t)60, (uint8_t)109, (uint8_t)13, (uint8_t)197, (uint8_t)38, (uint8_t)112, (uint8_t)143, (uint8_t)141, (uint8_t)101, (uint8_t)10, (uint8_t)178, (uint8_t)15, (uint8_t)162, (uint8_t)185, (uint8_t)137, (uint8_t)202, (uint8_t)125, (uint8_t)159, (uint8_t)23, (uint8_t)216, (uint8_t)77, (uint8_t)241, (uint8_t)190, (uint8_t)26, (uint8_t)2, (uint8_t)75, (uint8_t)122, (uint8_t)31, (uint8_t)104, (uint8_t)100, (uint8_t)76, (uint8_t)213, (uint8_t)136, (uint8_t)253, (uint8_t)36, (uint8_t)243, (uint8_t)157, (uint8_t)236, (uint8_t)43, (uint8_t)218, (uint8_t)81, (uint8_t)85, (uint8_t)155, (uint8_t)25, (uint8_t)155, (uint8_t)3, (uint8_t)107, (uint8_t)175, (uint8_t)19, (uint8_t)251, (uint8_t)67, (uint8_t)205, (uint8_t)252, (uint8_t)112, (uint8_t)29, (uint8_t)190, (uint8_t)182, (uint8_t)11, (uint8_t)83, (uint8_t)110, (uint8_t)144, (uint8_t)52, (uint8_t)253, (uint8_t)215, (uint8_t)50, (uint8_t)148, (uint8_t)13, (uint8_t)169, (uint8_t)12, (uint8_t)132, (uint8_t)194, (uint8_t)237, (uint8_t)235, (uint8_t)42, (uint8_t)190, (uint8_t)197, (uint8_t)10, (uint8_t)92, (uint8_t)109, (uint8_t)151, (uint8_t)12, (uint8_t)33, (uint8_t)133, (uint8_t)72, (uint8_t)167, (uint8_t)146, (uint8_t)12, (uint8_t)96, (uint8_t)137, (uint8_t)31, (uint8_t)168, (uint8_t)254, (uint8_t)91, (uint8_t)82, (uint8_t)77, (uint8_t)98, (uint8_t)197, (uint8_t)67, (uint8_t)126, (uint8_t)179, (uint8_t)77, (uint8_t)13, (uint8_t)120, (uint8_t)199, (uint8_t)195, (uint8_t)121, (uint8_t)50, (uint8_t)69, (uint8_t)79, (uint8_t)11, (uint8_t)112, (uint8_t)110, (uint8_t)216, (uint8_t)165, (uint8_t)248, (uint8_t)23, (uint8_t)219, (uint8_t)140, (uint8_t)88, (uint8_t)192, (uint8_t)14, (uint8_t)85, (uint8_t)197, (uint8_t)17, (uint8_t)205, (uint8_t)238, (uint8_t)188, (uint8_t)192, (uint8_t)99, (uint8_t)231, (uint8_t)158, (uint8_t)200, (uint8_t)104, (uint8_t)97, (uint8_t)125, (uint8_t)186, (uint8_t)99, (uint8_t)186, (uint8_t)145, (uint8_t)219, (uint8_t)27, (uint8_t)72, (uint8_t)51, (uint8_t)33, (uint8_t)188, (uint8_t)2, (uint8_t)244, (uint8_t)134, (uint8_t)186, (uint8_t)139, (uint8_t)158, (uint8_t)58, (uint8_t)65, (uint8_t)130, (uint8_t)52, (uint8_t)141, (uint8_t)236, (uint8_t)12, (uint8_t)227, (uint8_t)166, (uint8_t)34, (uint8_t)45, (uint8_t)240, (uint8_t)138, (uint8_t)201, (uint8_t)218, (uint8_t)147, (uint8_t)203, (uint8_t)118, (uint8_t)209, (uint8_t)87, (uint8_t)79, (uint8_t)121, (uint8_t)119, (uint8_t)72, (uint8_t)127, (uint8_t)83, (uint8_t)208, (uint8_t)157, (uint8_t)180, (uint8_t)22, (uint8_t)129, (uint8_t)107, (uint8_t)218, (uint8_t)12, (uint8_t)237, (uint8_t)170, (uint8_t)241, (uint8_t)6, (uint8_t)20, (uint8_t)239, (uint8_t)196, (uint8_t)77, (uint8_t)80, (uint8_t)199, (uint8_t)71, (uint8_t)95, (uint8_t)233, (uint8_t)155, (uint8_t)220, (uint8_t)129, (uint8_t)15, (uint8_t)163, (uint8_t)127, (uint8_t)110, (uint8_t)44, (uint8_t)77, (uint8_t)70, (uint8_t)126, (uint8_t)146} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)61589);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)127);
    {
        int8_t exemplary[] =  {(int8_t)113, (int8_t)122, (int8_t) -2, (int8_t)8, (int8_t) -115, (int8_t) -114, (int8_t) -28, (int8_t)12, (int8_t) -99, (int8_t) -127, (int8_t) -27, (int8_t) -92, (int8_t)59, (int8_t) -36, (int8_t)92, (int8_t) -24, (int8_t) -34, (int8_t)59, (int8_t)92, (int8_t) -105, (int8_t)118, (int8_t) -18, (int8_t)0, (int8_t) -126, (int8_t)7, (int8_t)104, (int8_t) -99, (int8_t) -57, (int8_t) -13, (int8_t) -15, (int8_t)40, (int8_t) -12} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float) -1.9085707E38F);
    assert(p250_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"nd";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_x_GET(pack) == (float) -1.4127459E38F);
    assert(p250_z_GET(pack) == (float) -7.546532E36F);
    assert(p250_time_usec_GET(pack) == (uint64_t)3282488120697872514L);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2854253318L);
    assert(p251_value_GET(pack) == (float)5.5621735E37F);
    assert(p251_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"oglicu";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"t";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)2871683760L);
    assert(p252_value_GET(pack) == (int32_t) -705439287);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 11);
    {
        char16_t * exemplary = u"polukxsadxv";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)3.4438005E37F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)322082211L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)70, (uint8_t)184, (uint8_t)194, (uint8_t)100, (uint8_t)124, (uint8_t)86, (uint8_t)4, (uint8_t)181, (uint8_t)20, (uint8_t)88, (uint8_t)21, (uint8_t)202, (uint8_t)62, (uint8_t)115, (uint8_t)229, (uint8_t)247, (uint8_t)9, (uint8_t)235, (uint8_t)46, (uint8_t)15, (uint8_t)62, (uint8_t)122, (uint8_t)253, (uint8_t)39, (uint8_t)229, (uint8_t)15, (uint8_t)80, (uint8_t)79, (uint8_t)57, (uint8_t)232, (uint8_t)45, (uint8_t)53} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)7560929290111555814L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)142);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)429217244L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1589180048L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p258_tune_LEN(ph) == 27);
    {
        char16_t * exemplary = u"tjucyymxjfaulapdaoqcjktsybk";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 54);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_sensor_size_v_GET(pack) == (float) -2.9153854E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)204, (uint8_t)172, (uint8_t)63, (uint8_t)180, (uint8_t)229, (uint8_t)119, (uint8_t)249, (uint8_t)128, (uint8_t)37, (uint8_t)2, (uint8_t)139, (uint8_t)77, (uint8_t)135, (uint8_t)152, (uint8_t)11, (uint8_t)209, (uint8_t)191, (uint8_t)169, (uint8_t)156, (uint8_t)141, (uint8_t)70, (uint8_t)69, (uint8_t)47, (uint8_t)215, (uint8_t)217, (uint8_t)150, (uint8_t)81, (uint8_t)198, (uint8_t)231, (uint8_t)221, (uint8_t)109, (uint8_t)210} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float)1.1062894E38F);
    assert(p259_cam_definition_uri_LEN(ph) == 81);
    {
        char16_t * exemplary = u"qfpeebumbksZqsPrxvdnifkhmatobkevwydwarcavxaxdfvtcrteqrbUsleaTymjkzpjhnyduyByodnbt";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 162);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)29587);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1182861284L);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)35687);
    assert(p259_sensor_size_h_GET(pack) == (float) -7.8216775E37F);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)27);
    {
        uint8_t exemplary[] =  {(uint8_t)24, (uint8_t)217, (uint8_t)116, (uint8_t)13, (uint8_t)111, (uint8_t)227, (uint8_t)117, (uint8_t)81, (uint8_t)131, (uint8_t)214, (uint8_t)122, (uint8_t)113, (uint8_t)145, (uint8_t)6, (uint8_t)206, (uint8_t)111, (uint8_t)239, (uint8_t)49, (uint8_t)186, (uint8_t)120, (uint8_t)221, (uint8_t)20, (uint8_t)144, (uint8_t)22, (uint8_t)222, (uint8_t)222, (uint8_t)255, (uint8_t)82, (uint8_t)124, (uint8_t)62, (uint8_t)14, (uint8_t)11} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)2735639571L);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)37572);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == (e_CAMERA_MODE_CAMERA_MODE_IMAGE));
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)1149194277L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p261_used_capacity_GET(pack) == (float)6.0241033E37F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)1786781158L);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p261_read_speed_GET(pack) == (float)1.2982745E38F);
    assert(p261_available_capacity_GET(pack) == (float)3.179876E38F);
    assert(p261_total_capacity_GET(pack) == (float) -2.631183E38F);
    assert(p261_write_speed_GET(pack) == (float)2.634955E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3939513950L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1009750268L);
    assert(p262_available_capacity_GET(pack) == (float) -2.6755825E38F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p262_image_interval_GET(pack) == (float)1.2733833E38F);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_image_index_GET(pack) == (int32_t)2021227512);
    {
        float exemplary[] =  {-9.680622E37F, -1.0260593E38F, 1.7954011E38F, 1.0507582E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)71);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1698069613L);
    assert(p263_lat_GET(pack) == (int32_t)1223528164);
    assert(p263_time_utc_GET(pack) == (uint64_t)6354560211876425049L);
    assert(p263_file_url_LEN(ph) == 84);
    {
        char16_t * exemplary = u"tmietjvszicbvyywsbgnljlskimpxnzkmyotpgtwupyWhQxozwnlqdxtdwjenwpkcatgkotajvnigqkyolad";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 168);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_lon_GET(pack) == (int32_t)38337045);
    assert(p263_alt_GET(pack) == (int32_t)1002221480);
    assert(p263_relative_alt_GET(pack) == (int32_t) -896008055);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)5272936744596671832L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)3604298498276461216L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3376908904L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)7321033113673573919L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float)3.1429431E38F);
    assert(p265_yaw_GET(pack) == (float) -3.3693192E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)724555417L);
    assert(p265_pitch_GET(pack) == (float) -2.2857483E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)227);
    {
        uint8_t exemplary[] =  {(uint8_t)184, (uint8_t)233, (uint8_t)57, (uint8_t)103, (uint8_t)138, (uint8_t)1, (uint8_t)151, (uint8_t)149, (uint8_t)50, (uint8_t)47, (uint8_t)139, (uint8_t)89, (uint8_t)58, (uint8_t)207, (uint8_t)46, (uint8_t)153, (uint8_t)63, (uint8_t)175, (uint8_t)151, (uint8_t)161, (uint8_t)93, (uint8_t)111, (uint8_t)7, (uint8_t)133, (uint8_t)202, (uint8_t)79, (uint8_t)99, (uint8_t)178, (uint8_t)47, (uint8_t)50, (uint8_t)32, (uint8_t)76, (uint8_t)198, (uint8_t)13, (uint8_t)199, (uint8_t)227, (uint8_t)87, (uint8_t)87, (uint8_t)82, (uint8_t)217, (uint8_t)142, (uint8_t)252, (uint8_t)246, (uint8_t)250, (uint8_t)201, (uint8_t)185, (uint8_t)110, (uint8_t)207, (uint8_t)89, (uint8_t)45, (uint8_t)16, (uint8_t)35, (uint8_t)59, (uint8_t)38, (uint8_t)99, (uint8_t)20, (uint8_t)25, (uint8_t)134, (uint8_t)218, (uint8_t)70, (uint8_t)124, (uint8_t)1, (uint8_t)166, (uint8_t)28, (uint8_t)12, (uint8_t)208, (uint8_t)48, (uint8_t)1, (uint8_t)197, (uint8_t)236, (uint8_t)203, (uint8_t)186, (uint8_t)139, (uint8_t)87, (uint8_t)40, (uint8_t)180, (uint8_t)191, (uint8_t)186, (uint8_t)153, (uint8_t)79, (uint8_t)197, (uint8_t)58, (uint8_t)220, (uint8_t)188, (uint8_t)103, (uint8_t)92, (uint8_t)81, (uint8_t)232, (uint8_t)184, (uint8_t)213, (uint8_t)151, (uint8_t)76, (uint8_t)38, (uint8_t)229, (uint8_t)87, (uint8_t)224, (uint8_t)23, (uint8_t)211, (uint8_t)161, (uint8_t)197, (uint8_t)156, (uint8_t)139, (uint8_t)240, (uint8_t)246, (uint8_t)115, (uint8_t)140, (uint8_t)173, (uint8_t)189, (uint8_t)93, (uint8_t)32, (uint8_t)34, (uint8_t)212, (uint8_t)186, (uint8_t)218, (uint8_t)224, (uint8_t)40, (uint8_t)51, (uint8_t)232, (uint8_t)236, (uint8_t)46, (uint8_t)138, (uint8_t)101, (uint8_t)146, (uint8_t)174, (uint8_t)150, (uint8_t)219, (uint8_t)180, (uint8_t)216, (uint8_t)67, (uint8_t)41, (uint8_t)192, (uint8_t)101, (uint8_t)74, (uint8_t)206, (uint8_t)84, (uint8_t)193, (uint8_t)142, (uint8_t)251, (uint8_t)120, (uint8_t)138, (uint8_t)81, (uint8_t)130, (uint8_t)177, (uint8_t)62, (uint8_t)220, (uint8_t)213, (uint8_t)215, (uint8_t)185, (uint8_t)16, (uint8_t)230, (uint8_t)200, (uint8_t)178, (uint8_t)38, (uint8_t)246, (uint8_t)200, (uint8_t)57, (uint8_t)75, (uint8_t)79, (uint8_t)117, (uint8_t)71, (uint8_t)227, (uint8_t)32, (uint8_t)123, (uint8_t)156, (uint8_t)94, (uint8_t)128, (uint8_t)194, (uint8_t)51, (uint8_t)181, (uint8_t)176, (uint8_t)142, (uint8_t)225, (uint8_t)227, (uint8_t)97, (uint8_t)110, (uint8_t)165, (uint8_t)151, (uint8_t)205, (uint8_t)103, (uint8_t)128, (uint8_t)218, (uint8_t)149, (uint8_t)222, (uint8_t)29, (uint8_t)42, (uint8_t)241, (uint8_t)174, (uint8_t)230, (uint8_t)153, (uint8_t)156, (uint8_t)164, (uint8_t)57, (uint8_t)227, (uint8_t)211, (uint8_t)120, (uint8_t)254, (uint8_t)174, (uint8_t)217, (uint8_t)66, (uint8_t)50, (uint8_t)111, (uint8_t)158, (uint8_t)242, (uint8_t)153, (uint8_t)114, (uint8_t)100, (uint8_t)216, (uint8_t)123, (uint8_t)88, (uint8_t)173, (uint8_t)195, (uint8_t)201, (uint8_t)86, (uint8_t)72, (uint8_t)131, (uint8_t)120, (uint8_t)221, (uint8_t)238, (uint8_t)200, (uint8_t)121, (uint8_t)239, (uint8_t)95, (uint8_t)23, (uint8_t)57, (uint8_t)222, (uint8_t)7, (uint8_t)220, (uint8_t)64, (uint8_t)63, (uint8_t)204, (uint8_t)131, (uint8_t)183, (uint8_t)101, (uint8_t)135, (uint8_t)196, (uint8_t)202, (uint8_t)127, (uint8_t)232, (uint8_t)68, (uint8_t)95, (uint8_t)182, (uint8_t)94, (uint8_t)195, (uint8_t)58, (uint8_t)40, (uint8_t)125, (uint8_t)228, (uint8_t)135, (uint8_t)220} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)18057);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)85);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)2573);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    {
        uint8_t exemplary[] =  {(uint8_t)224, (uint8_t)119, (uint8_t)11, (uint8_t)165, (uint8_t)241, (uint8_t)83, (uint8_t)182, (uint8_t)167, (uint8_t)250, (uint8_t)7, (uint8_t)84, (uint8_t)161, (uint8_t)128, (uint8_t)216, (uint8_t)155, (uint8_t)189, (uint8_t)10, (uint8_t)253, (uint8_t)105, (uint8_t)63, (uint8_t)214, (uint8_t)224, (uint8_t)147, (uint8_t)84, (uint8_t)202, (uint8_t)218, (uint8_t)53, (uint8_t)85, (uint8_t)48, (uint8_t)106, (uint8_t)30, (uint8_t)198, (uint8_t)99, (uint8_t)194, (uint8_t)149, (uint8_t)239, (uint8_t)208, (uint8_t)220, (uint8_t)246, (uint8_t)254, (uint8_t)215, (uint8_t)251, (uint8_t)2, (uint8_t)127, (uint8_t)37, (uint8_t)11, (uint8_t)56, (uint8_t)13, (uint8_t)101, (uint8_t)72, (uint8_t)14, (uint8_t)226, (uint8_t)224, (uint8_t)81, (uint8_t)2, (uint8_t)166, (uint8_t)98, (uint8_t)42, (uint8_t)187, (uint8_t)10, (uint8_t)23, (uint8_t)72, (uint8_t)21, (uint8_t)28, (uint8_t)68, (uint8_t)28, (uint8_t)225, (uint8_t)7, (uint8_t)152, (uint8_t)84, (uint8_t)93, (uint8_t)107, (uint8_t)227, (uint8_t)248, (uint8_t)121, (uint8_t)117, (uint8_t)241, (uint8_t)145, (uint8_t)118, (uint8_t)201, (uint8_t)48, (uint8_t)211, (uint8_t)117, (uint8_t)220, (uint8_t)135, (uint8_t)124, (uint8_t)140, (uint8_t)110, (uint8_t)122, (uint8_t)101, (uint8_t)237, (uint8_t)44, (uint8_t)180, (uint8_t)78, (uint8_t)55, (uint8_t)4, (uint8_t)199, (uint8_t)13, (uint8_t)218, (uint8_t)170, (uint8_t)169, (uint8_t)240, (uint8_t)66, (uint8_t)205, (uint8_t)204, (uint8_t)116, (uint8_t)146, (uint8_t)31, (uint8_t)167, (uint8_t)3, (uint8_t)232, (uint8_t)121, (uint8_t)223, (uint8_t)108, (uint8_t)1, (uint8_t)137, (uint8_t)184, (uint8_t)26, (uint8_t)245, (uint8_t)52, (uint8_t)86, (uint8_t)147, (uint8_t)132, (uint8_t)228, (uint8_t)94, (uint8_t)218, (uint8_t)176, (uint8_t)235, (uint8_t)137, (uint8_t)54, (uint8_t)154, (uint8_t)212, (uint8_t)178, (uint8_t)163, (uint8_t)137, (uint8_t)102, (uint8_t)179, (uint8_t)104, (uint8_t)167, (uint8_t)54, (uint8_t)237, (uint8_t)74, (uint8_t)67, (uint8_t)174, (uint8_t)145, (uint8_t)173, (uint8_t)249, (uint8_t)79, (uint8_t)158, (uint8_t)228, (uint8_t)53, (uint8_t)97, (uint8_t)55, (uint8_t)229, (uint8_t)110, (uint8_t)64, (uint8_t)35, (uint8_t)199, (uint8_t)183, (uint8_t)54, (uint8_t)225, (uint8_t)115, (uint8_t)115, (uint8_t)254, (uint8_t)98, (uint8_t)122, (uint8_t)165, (uint8_t)66, (uint8_t)37, (uint8_t)217, (uint8_t)157, (uint8_t)180, (uint8_t)248, (uint8_t)149, (uint8_t)26, (uint8_t)54, (uint8_t)146, (uint8_t)21, (uint8_t)209, (uint8_t)101, (uint8_t)66, (uint8_t)47, (uint8_t)186, (uint8_t)187, (uint8_t)88, (uint8_t)59, (uint8_t)179, (uint8_t)28, (uint8_t)229, (uint8_t)117, (uint8_t)222, (uint8_t)100, (uint8_t)49, (uint8_t)89, (uint8_t)94, (uint8_t)37, (uint8_t)151, (uint8_t)14, (uint8_t)51, (uint8_t)38, (uint8_t)83, (uint8_t)154, (uint8_t)71, (uint8_t)138, (uint8_t)245, (uint8_t)26, (uint8_t)53, (uint8_t)209, (uint8_t)45, (uint8_t)241, (uint8_t)74, (uint8_t)204, (uint8_t)5, (uint8_t)211, (uint8_t)239, (uint8_t)120, (uint8_t)53, (uint8_t)31, (uint8_t)104, (uint8_t)166, (uint8_t)114, (uint8_t)231, (uint8_t)194, (uint8_t)2, (uint8_t)0, (uint8_t)143, (uint8_t)186, (uint8_t)158, (uint8_t)194, (uint8_t)230, (uint8_t)220, (uint8_t)254, (uint8_t)16, (uint8_t)46, (uint8_t)60, (uint8_t)83, (uint8_t)8, (uint8_t)227, (uint8_t)253, (uint8_t)145, (uint8_t)123, (uint8_t)238, (uint8_t)187, (uint8_t)166, (uint8_t)104, (uint8_t)68, (uint8_t)94, (uint8_t)145, (uint8_t)196} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)55826);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)226);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)4994);
    assert(p269_framerate_GET(pack) == (float)1.0813347E38F);
    assert(p269_uri_LEN(ph) == 69);
    {
        char16_t * exemplary = u"iwtlpyiqsdoflRicvtotxpofhikmpubpfkegpepzbtCfhpgubhnliaytjnMbkcnncdrJq";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 138);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p269_bitrate_GET(pack) == (uint32_t)3112803489L);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)60307);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)34337);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p270_bitrate_GET(pack) == (uint32_t)2775779314L);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p270_uri_LEN(ph) == 222);
    {
        char16_t * exemplary = u"sfwixrrZebQoSIvxcoqEeenyigqiixhthNbhdztpepBLkoigwfogleosqdbrifgitnfdiQzlaxliwmxWkndifhtxymernVhqbikpudtyggsribwwsraodhwblDspzmoCohmeknquaFkubjhucvqadoupbvUihdkHyXduAgngyosZgSdaiqretyytbYhgmhbkcrakruyyjinuahwBdijxamkvvpevjs";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 444);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)18637);
    assert(p270_framerate_GET(pack) == (float) -2.5235058E38F);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)50706);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)41664);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 22);
    {
        char16_t * exemplary = u"szcgzXNtrchdjrlnejqbqa";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 32);
    {
        char16_t * exemplary = u"jcogjxiCttlxnrfdkiYyhsnoafntidrk";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)122, (uint8_t)232, (uint8_t)248, (uint8_t)83, (uint8_t)101, (uint8_t)233, (uint8_t)0} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)21003);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)35253);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)55416);
    {
        uint8_t exemplary[] =  {(uint8_t)98, (uint8_t)186, (uint8_t)177, (uint8_t)248, (uint8_t)141, (uint8_t)190, (uint8_t)127, (uint8_t)249} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1829974794L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p310_time_usec_GET(pack) == (uint64_t)4790528632306991987L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)58257);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)255, (uint8_t)100, (uint8_t)136, (uint8_t)89, (uint8_t)245, (uint8_t)195, (uint8_t)119, (uint8_t)41, (uint8_t)50, (uint8_t)226, (uint8_t)154, (uint8_t)18, (uint8_t)129, (uint8_t)238, (uint8_t)78, (uint8_t)225} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)4130534608L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p311_name_LEN(ph) == 76);
    {
        char16_t * exemplary = u"fjFFFoxhreublhhCgviKwaifazicehlrrEevocnctqdughvfmkEvjfDohIijanbclcZhvkwzytlP";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 152);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)2976849736391526273L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)1079477178L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"is";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -2707);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)56);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)45008);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p322_param_value_LEN(ph) == 78);
    {
        char16_t * exemplary = u"yvzdelvncvoybqvaxyqMswfykqgPNwzhSHxgezUbiiUaphnynfzrbrabludfdqxotfwgvnuRefpsfl";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 156);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)54931);
    assert(p322_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"erjewpbypevjdNp";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p323_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"i";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p323_param_value_LEN(ph) == 30);
    {
        char16_t * exemplary = u"qkylnyxzylzuQJxptqkvygjcoizjJc";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED);
    assert(p324_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"qpvoLkQevpbrdq";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64);
    assert(p324_param_value_LEN(ph) == 121);
    {
        char16_t * exemplary = u"yyckhceagszuqXkyfhjuaxwcoeJcrbqnbyXnxfentbaghBktipvpftysiigvqhxsRdotyfrqnmmiThigspcrcelxLgktipwJyiwdeksdcvPvbUNuhtubwszir";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 242);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)52614);
    assert(p330_time_usec_GET(pack) == (uint64_t)5045172981299597924L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
    {
        uint16_t exemplary[] =  {(uint16_t)40688, (uint16_t)8771, (uint16_t)27197, (uint16_t)29368, (uint16_t)10304, (uint16_t)2747, (uint16_t)33826, (uint16_t)38649, (uint16_t)13239, (uint16_t)58392, (uint16_t)58742, (uint16_t)21004, (uint16_t)51278, (uint16_t)44517, (uint16_t)60677, (uint16_t)6153, (uint16_t)9095, (uint16_t)45364, (uint16_t)20957, (uint16_t)22814, (uint16_t)21335, (uint16_t)17102, (uint16_t)36962, (uint16_t)27918, (uint16_t)26440, (uint16_t)26678, (uint16_t)36417, (uint16_t)26738, (uint16_t)56066, (uint16_t)57148, (uint16_t)16029, (uint16_t)36844, (uint16_t)3664, (uint16_t)35953, (uint16_t)60420, (uint16_t)17093, (uint16_t)52179, (uint16_t)44681, (uint16_t)55402, (uint16_t)35930, (uint16_t)5750, (uint16_t)57283, (uint16_t)28035, (uint16_t)13275, (uint16_t)23120, (uint16_t)22745, (uint16_t)18863, (uint16_t)62778, (uint16_t)52659, (uint16_t)3871, (uint16_t)3996, (uint16_t)38198, (uint16_t)34833, (uint16_t)1214, (uint16_t)23753, (uint16_t)7457, (uint16_t)8114, (uint16_t)5829, (uint16_t)59042, (uint16_t)21006, (uint16_t)56260, (uint16_t)10638, (uint16_t)52725, (uint16_t)29414, (uint16_t)42113, (uint16_t)65123, (uint16_t)11936, (uint16_t)63549, (uint16_t)19640, (uint16_t)21190, (uint16_t)40550, (uint16_t)61212} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)38686);
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
        p0_custom_mode_SET((uint32_t)2423593780L, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SMARTAP, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_TRICOPTER, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN), PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)70, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)41960, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)15082, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -1848, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)56378, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)12763, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)36522, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)10801, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)51475, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL), PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)53468, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2346676012L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3723930155451809036L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float) -2.7409443E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2528478284L, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.3372801E37F, PH.base.pack) ;
        p3_y_SET((float)1.0188815E38F, PH.base.pack) ;
        p3_afx_SET((float) -1.4151404E38F, PH.base.pack) ;
        p3_vy_SET((float)2.8212832E38F, PH.base.pack) ;
        p3_afz_SET((float)2.35496E38F, PH.base.pack) ;
        p3_afy_SET((float) -2.633808E38F, PH.base.pack) ;
        p3_vx_SET((float) -8.620627E37F, PH.base.pack) ;
        p3_z_SET((float) -2.4526413E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)57156, PH.base.pack) ;
        p3_x_SET((float) -1.1879646E38F, PH.base.pack) ;
        p3_yaw_SET((float)2.1603748E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p4_seq_SET((uint32_t)3198213617L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)2810601885153838797L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        {
            char16_t* passkey = u"pjdbGtopwinRrxvkxteqqyzl";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"vtwlsuuRtuwkj";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)1510002246L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -27769, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        {
            char16_t* param_id = u"hnhcildy";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_index_SET((uint16_t)(uint16_t)17710, PH.base.pack) ;
        p22_param_value_SET((float)1.1768356E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"ninsiofflh";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_count_SET((uint16_t)(uint16_t)16579, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_value_SET((float) -2.0916126E37F, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT8, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        {
            char16_t* param_id = u"Dp";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_epv_SET((uint16_t)(uint16_t)59847, PH.base.pack) ;
        p24_lon_SET((int32_t)1422886212, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)45524, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)3699854308L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)45671, PH.base.pack) ;
        p24_lat_SET((int32_t) -1668169985, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p24_alt_SET((int32_t) -1121075319, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -1567502167, &PH) ;
        p24_h_acc_SET((uint32_t)452352066L, &PH) ;
        p24_time_usec_SET((uint64_t)1901153044632798207L, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)4128800915L, &PH) ;
        p24_hdg_acc_SET((uint32_t)3349082775L, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)45660, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)49, (uint8_t)252, (uint8_t)71, (uint8_t)222, (uint8_t)10, (uint8_t)9, (uint8_t)86, (uint8_t)223, (uint8_t)123, (uint8_t)89, (uint8_t)74, (uint8_t)193, (uint8_t)136, (uint8_t)190, (uint8_t)172, (uint8_t)92, (uint8_t)73, (uint8_t)169, (uint8_t)236, (uint8_t)116};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        {
            uint8_t satellite_elevation[] =  {(uint8_t)142, (uint8_t)228, (uint8_t)114, (uint8_t)196, (uint8_t)199, (uint8_t)115, (uint8_t)134, (uint8_t)140, (uint8_t)151, (uint8_t)195, (uint8_t)97, (uint8_t)55, (uint8_t)39, (uint8_t)3, (uint8_t)77, (uint8_t)100, (uint8_t)45, (uint8_t)226, (uint8_t)192, (uint8_t)220};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)16, (uint8_t)207, (uint8_t)24, (uint8_t)154, (uint8_t)67, (uint8_t)99, (uint8_t)20, (uint8_t)44, (uint8_t)18, (uint8_t)30, (uint8_t)237, (uint8_t)73, (uint8_t)25, (uint8_t)149, (uint8_t)213, (uint8_t)208, (uint8_t)151, (uint8_t)13, (uint8_t)67, (uint8_t)128};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)111, (uint8_t)33, (uint8_t)21, (uint8_t)186, (uint8_t)242, (uint8_t)121, (uint8_t)114, (uint8_t)3, (uint8_t)209, (uint8_t)20, (uint8_t)206, (uint8_t)102, (uint8_t)215, (uint8_t)164, (uint8_t)159, (uint8_t)134, (uint8_t)46, (uint8_t)65, (uint8_t)200, (uint8_t)87};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)55, (uint8_t)247, (uint8_t)222, (uint8_t)110, (uint8_t)115, (uint8_t)147, (uint8_t)62, (uint8_t)98, (uint8_t)209, (uint8_t)238, (uint8_t)210, (uint8_t)221, (uint8_t)192, (uint8_t)251, (uint8_t)237, (uint8_t)194, (uint8_t)220, (uint8_t)159, (uint8_t)123, (uint8_t)245};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_ygyro_SET((int16_t)(int16_t)29582, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -1332, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)15292, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -7382, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)24753, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)20416, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -9157, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -19673, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -10698, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)2685151520L, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_zacc_SET((int16_t)(int16_t)32213, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -4007, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)23172, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -12810, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)3536, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)6197393346218843766L, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -29894, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -2191, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)14097, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)2889, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)2461501344094065903L, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)3527, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -19782, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)26344, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)14211, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_time_boot_ms_SET((uint32_t)1879251614L, PH.base.pack) ;
        p29_press_abs_SET((float) -7.124683E37F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)22886, PH.base.pack) ;
        p29_press_diff_SET((float) -1.3348324E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yawspeed_SET((float)4.781216E37F, PH.base.pack) ;
        p30_pitch_SET((float) -6.8399825E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2505185329L, PH.base.pack) ;
        p30_rollspeed_SET((float) -5.385155E36F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -3.2025209E38F, PH.base.pack) ;
        p30_yaw_SET((float) -1.4475818E38F, PH.base.pack) ;
        p30_roll_SET((float)3.3877881E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float) -1.4232041E38F, PH.base.pack) ;
        p31_q3_SET((float)2.1826538E38F, PH.base.pack) ;
        p31_q4_SET((float) -1.4382043E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -3.2742486E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -1.8005766E38F, PH.base.pack) ;
        p31_q1_SET((float)7.535904E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)3223719818L, PH.base.pack) ;
        p31_q2_SET((float)1.9824122E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_z_SET((float) -2.4086879E38F, PH.base.pack) ;
        p32_x_SET((float)2.4762709E38F, PH.base.pack) ;
        p32_vy_SET((float)3.398603E38F, PH.base.pack) ;
        p32_vz_SET((float)1.9283343E38F, PH.base.pack) ;
        p32_vx_SET((float) -6.751133E37F, PH.base.pack) ;
        p32_y_SET((float) -3.1864793E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)2955826632L, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t) -1485226810, PH.base.pack) ;
        p33_lon_SET((int32_t) -830532195, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)10582653L, PH.base.pack) ;
        p33_lat_SET((int32_t) -123071964, PH.base.pack) ;
        p33_alt_SET((int32_t)1665373210, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -1509, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)10676, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)8203, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)22294, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan7_scaled_SET((int16_t)(int16_t) -7561, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -29470, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -3651, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)19821, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -15697, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -13873, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1820205158L, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -29150, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -28363, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan1_raw_SET((uint16_t)(uint16_t)54446, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)56991, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)856, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)55524, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)9874, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)62532, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)21057, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1654019221L, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)44441, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo8_raw_SET((uint16_t)(uint16_t)27136, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)20844, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)38152, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)4425, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)6008, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)50076, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)12343, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)61418, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)64444, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)22926, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)48388, &PH) ;
        p36_time_usec_SET((uint32_t)4047470857L, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)27517, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)31034, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)12858, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)44614, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)52805, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -13078, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)30824, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -22423, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)27873, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_x_SET((float) -2.5148338E38F, PH.base.pack) ;
        p39_param1_SET((float)2.871177E38F, PH.base.pack) ;
        p39_y_SET((float) -3.12128E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p39_z_SET((float)2.5264631E38F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_ARM_AUTHORIZATION_REQUEST, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)58363, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p39_param2_SET((float) -3.2071294E38F, PH.base.pack) ;
        p39_param3_SET((float)2.8806742E36F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p39_param4_SET((float) -2.0028399E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)54891, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)47238, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)34082, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)28938, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)25401, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM3, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -1664395322, PH.base.pack) ;
        p48_latitude_SET((int32_t)245359734, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p48_longitude_SET((int32_t) -1520658198, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)5738072015603966005L, &PH) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_altitude_SET((int32_t) -687916966, PH.base.pack) ;
        p49_longitude_SET((int32_t)946025967, PH.base.pack) ;
        p49_latitude_SET((int32_t)3259329, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)8420493050355081020L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_index_SET((int16_t)(int16_t)27024, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.9798987E38F, PH.base.pack) ;
        p50_param_value_min_SET((float) -3.3634314E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        {
            char16_t* param_id = u"lcgsnescz";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p50_param_value0_SET((float)7.3993937E37F, PH.base.pack) ;
        p50_scale_SET((float) -2.302138E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)56096, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p2y_SET((float) -1.7098745E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p54_p1y_SET((float)2.7241757E38F, PH.base.pack) ;
        p54_p1x_SET((float) -2.0110962E37F, PH.base.pack) ;
        p54_p2x_SET((float)7.3356714E37F, PH.base.pack) ;
        p54_p1z_SET((float) -1.5259321E38F, PH.base.pack) ;
        p54_p2z_SET((float) -2.93023E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1y_SET((float)5.555786E36F, PH.base.pack) ;
        p55_p2y_SET((float)3.3505965E37F, PH.base.pack) ;
        p55_p1x_SET((float) -4.662704E37F, PH.base.pack) ;
        p55_p1z_SET((float)3.2840405E38F, PH.base.pack) ;
        p55_p2z_SET((float) -5.9860393E37F, PH.base.pack) ;
        p55_p2x_SET((float)9.624922E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        {
            float q[] =  {-6.1036905E37F, -9.226096E37F, -1.8048852E37F, -3.151878E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)1.020365E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)1671958004058356738L, PH.base.pack) ;
        p61_yawspeed_SET((float)1.6946824E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.2818405E37F, PH.base.pack) ;
        {
            float covariance[] =  {-2.3316161E38F, 2.7826292E38F, 2.0677182E38F, -5.048614E36F, 2.1495892E38F, 3.1818722E38F, 8.095292E37F, -3.6193433E37F, 1.8011133E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_roll_SET((float)2.9787206E37F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)9848, PH.base.pack) ;
        p62_nav_pitch_SET((float)3.0324058E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -26183, PH.base.pack) ;
        p62_aspd_error_SET((float)1.7047986E38F, PH.base.pack) ;
        p62_alt_error_SET((float)2.4276162E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)30056, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.7043107E37F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_time_usec_SET((uint64_t)491036778005900441L, PH.base.pack) ;
        p63_lat_SET((int32_t)1292797653, PH.base.pack) ;
        {
            float covariance[] =  {1.7105948E37F, -2.2748592E38F, 5.563844E37F, -3.7532903E37F, 7.2679647E37F, 8.20828E37F, 3.3512502E38F, -2.7349302E38F, -2.5527846E38F, -6.92839E37F, -4.244514E37F, -1.2660062E38F, -4.039854E37F, -3.3785702E38F, -1.2683408E38F, -2.7305099E38F, -1.9598304E37F, -9.059313E37F, -1.7369308E38F, 1.1610119E38F, -2.1601324E38F, 2.2650294E38F, 3.0616253E38F, 2.4297292E37F, -3.1003614E38F, 1.9600866E38F, -1.597541E38F, 1.6563358E38F, -2.37621E38F, -9.028436E37F, -2.1550437E38F, 3.3867892E38F, 1.4193644E37F, 8.4332104E37F, 1.9516126E38F, -3.0532038E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p63_vz_SET((float)9.501174E37F, PH.base.pack) ;
        p63_alt_SET((int32_t)1873791501, PH.base.pack) ;
        p63_vx_SET((float) -3.1641494E38F, PH.base.pack) ;
        p63_vy_SET((float)3.1245013E38F, PH.base.pack) ;
        p63_lon_SET((int32_t)1862417841, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)36525046, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_y_SET((float)1.228514E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)2183539936051366253L, PH.base.pack) ;
        p64_vz_SET((float) -3.9642847E37F, PH.base.pack) ;
        p64_ay_SET((float) -1.8340728E38F, PH.base.pack) ;
        p64_z_SET((float)2.0222984E38F, PH.base.pack) ;
        p64_az_SET((float)9.8351464E36F, PH.base.pack) ;
        p64_vx_SET((float) -5.372609E37F, PH.base.pack) ;
        {
            float covariance[] =  {8.900323E36F, -4.934569E37F, 1.4151232E38F, 1.8731655E38F, -1.41341E38F, 2.8232449E38F, -6.778165E36F, -8.0311896E36F, -3.4065075E37F, 7.826317E37F, 9.959434E37F, 9.760693E37F, 1.3285158E38F, -1.2884089E38F, 1.3205762E38F, -9.303471E37F, 6.111504E37F, -2.6802141E38F, 2.6554497E38F, -1.4407731E38F, -1.1644567E38F, 1.2995357E38F, 3.3338572E38F, -5.529965E37F, -2.6099843E37F, 2.9717277E38F, -3.0670752E38F, -5.008336E37F, -1.2418326E38F, 1.798309E38F, 2.6693858E38F, 4.053924E36F, -1.6236939E38F, 4.006538E37F, 1.931586E38F, -3.2853546E38F, -3.151547E38F, 1.3781696E38F, -2.9887543E37F, 3.2031066E38F, 3.0648514E38F, 1.3919331E38F, 1.4667922E37F, -2.7167633E38F, -1.0499182E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_ax_SET((float) -1.0177339E38F, PH.base.pack) ;
        p64_x_SET((float)8.902691E37F, PH.base.pack) ;
        p64_vy_SET((float)2.8219142E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan18_raw_SET((uint16_t)(uint16_t)13461, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)5116, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)60956, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)36097, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2823769843L, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)54374, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)22423, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)33590, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)42648, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)37755, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)36865, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)7246, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)10110, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)64206, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)26669, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)56877, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)60670, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)22375, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)45400, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)1227, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)23701, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t)6392, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)9435, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)39778, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -992, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -23752, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan5_raw_SET((uint16_t)(uint16_t)47107, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)63502, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)44571, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)43027, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)25869, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)51664, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)17669, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)17808, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param4_SET((float)1.8484922E38F, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_x_SET((int32_t) -1002998432, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p73_param3_SET((float)2.3127053E37F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)45606, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_LAST, PH.base.pack) ;
        p73_param2_SET((float) -1.3816249E38F, PH.base.pack) ;
        p73_y_SET((int32_t) -772497769, PH.base.pack) ;
        p73_param1_SET((float)2.5216972E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p73_z_SET((float)1.3005415E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_climb_SET((float) -1.8727408E38F, PH.base.pack) ;
        p74_airspeed_SET((float) -2.4793459E38F, PH.base.pack) ;
        p74_alt_SET((float) -1.3194194E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)3.2728167E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)11325, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)57842, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p75_y_SET((int32_t) -606817494, PH.base.pack) ;
        p75_param3_SET((float)3.3130981E38F, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p75_x_SET((int32_t) -336322319, PH.base.pack) ;
        p75_param2_SET((float) -2.081503E38F, PH.base.pack) ;
        p75_param4_SET((float) -3.1058326E38F, PH.base.pack) ;
        p75_param1_SET((float)2.2592458E38F, PH.base.pack) ;
        p75_z_SET((float) -1.6523164E37F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float)8.0808867E37F, PH.base.pack) ;
        p76_param5_SET((float)1.0482015E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p76_param4_SET((float) -3.2566246E38F, PH.base.pack) ;
        p76_param2_SET((float) -1.372465E38F, PH.base.pack) ;
        p76_param3_SET((float)1.9542734E38F, PH.base.pack) ;
        p76_param1_SET((float) -9.862346E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p76_param7_SET((float)5.0429154E37F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_STORAGE, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_ACCEPTED, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_FOLLOW, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)78, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)146, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)217, &PH) ;
        p77_result_param2_SET((int32_t)242936832, &PH) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float)1.6550413E38F, PH.base.pack) ;
        p81_thrust_SET((float)1.2891983E37F, PH.base.pack) ;
        p81_yaw_SET((float)3.2356855E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p81_roll_SET((float) -1.3332083E36F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1256731865L, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        {
            float q[] =  {-3.4377548E37F, 1.0982684E38F, -4.0808908E37F, 3.1715758E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_thrust_SET((float)3.244486E38F, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)1.2247258E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)782276471L, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p82_body_pitch_rate_SET((float)2.2063688E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p82_body_roll_rate_SET((float)3.2894028E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float)3.1337517E38F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.3212564E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p83_thrust_SET((float) -1.9421669E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -2.3814743E38F, PH.base.pack) ;
        {
            float q[] =  {1.3271474E38F, -3.3206746E38F, 1.6248533E38F, 2.7499048E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_time_boot_ms_SET((uint32_t)3435316170L, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_yaw_rate_SET((float) -5.961509E37F, PH.base.pack) ;
        p84_afy_SET((float)1.3454248E38F, PH.base.pack) ;
        p84_z_SET((float)3.1494845E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p84_afz_SET((float) -1.6181975E38F, PH.base.pack) ;
        p84_vz_SET((float)5.091223E37F, PH.base.pack) ;
        p84_vx_SET((float) -2.5595265E38F, PH.base.pack) ;
        p84_x_SET((float)1.2235483E37F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p84_afx_SET((float) -3.4019769E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3763944112L, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p84_yaw_SET((float) -3.0684298E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)51575, PH.base.pack) ;
        p84_vy_SET((float) -1.6085671E37F, PH.base.pack) ;
        p84_y_SET((float) -1.3573952E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p86_afz_SET((float) -2.0791354E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.7885547E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)716856956, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)4191360729L, PH.base.pack) ;
        p86_vx_SET((float) -3.0042222E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)5347, PH.base.pack) ;
        p86_alt_SET((float) -2.2521218E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1633200062, PH.base.pack) ;
        p86_yaw_SET((float) -5.786025E37F, PH.base.pack) ;
        p86_vz_SET((float) -2.8060955E38F, PH.base.pack) ;
        p86_vy_SET((float) -9.080303E37F, PH.base.pack) ;
        p86_afy_SET((float) -2.6208331E38F, PH.base.pack) ;
        p86_afx_SET((float)2.808481E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afy_SET((float) -1.9642936E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1256593703, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.3384305E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)15037, PH.base.pack) ;
        p87_vy_SET((float)1.8361897E38F, PH.base.pack) ;
        p87_afz_SET((float) -4.6088054E37F, PH.base.pack) ;
        p87_afx_SET((float) -1.6858977E38F, PH.base.pack) ;
        p87_vz_SET((float)1.3264277E38F, PH.base.pack) ;
        p87_alt_SET((float)2.4866887E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)1381697407L, PH.base.pack) ;
        p87_vx_SET((float)2.7500266E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)1334070125, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p87_yaw_SET((float) -2.4282827E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_time_boot_ms_SET((uint32_t)3725502360L, PH.base.pack) ;
        p89_roll_SET((float)3.0047607E38F, PH.base.pack) ;
        p89_x_SET((float)6.100752E37F, PH.base.pack) ;
        p89_z_SET((float) -3.042548E38F, PH.base.pack) ;
        p89_yaw_SET((float) -2.1526117E37F, PH.base.pack) ;
        p89_y_SET((float) -1.755969E38F, PH.base.pack) ;
        p89_pitch_SET((float)2.7863826E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_yawspeed_SET((float)2.1761888E38F, PH.base.pack) ;
        p90_rollspeed_SET((float)1.7064033E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -18542, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)581, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)379, PH.base.pack) ;
        p90_pitchspeed_SET((float)2.7277356E38F, PH.base.pack) ;
        p90_lat_SET((int32_t) -108942616, PH.base.pack) ;
        p90_yaw_SET((float)2.098659E38F, PH.base.pack) ;
        p90_lon_SET((int32_t) -872317632, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)5037110464653088812L, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)2730, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -11575, PH.base.pack) ;
        p90_alt_SET((int32_t)2117404459, PH.base.pack) ;
        p90_pitch_SET((float)2.0475099E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -19738, PH.base.pack) ;
        p90_roll_SET((float)9.75845E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux4_SET((float)1.3518521E37F, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -2.211536E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)5150141168296370973L, PH.base.pack) ;
        p91_yaw_rudder_SET((float)8.1900263E37F, PH.base.pack) ;
        p91_aux3_SET((float) -3.5038157E37F, PH.base.pack) ;
        p91_aux2_SET((float)5.3108643E36F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.7300052E38F, PH.base.pack) ;
        p91_throttle_SET((float)1.2278237E36F, PH.base.pack) ;
        p91_aux1_SET((float) -1.0824082E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan1_raw_SET((uint16_t)(uint16_t)56381, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)2224, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)53251, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)4774199946926482176L, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)36846, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)37938, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)27010, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)29752, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)37866, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)24465, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)58311, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)34445, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)32613, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {1.3418887E38F, -1.1743389E38F, -8.890766E37F, -2.6826523E36F, 2.7790467E38F, 3.2753256E38F, -2.8580919E38F, -4.9692147E37F, 3.3057312E38F, 3.0987997E38F, -1.1069859E37F, 1.2913704E38F, 2.3159413E38F, -1.5208455E38F, -1.4629913E38F, -3.0278713E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_flags_SET((uint64_t)7210501783100001950L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)6530728967065960754L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_x_SET((int16_t)(int16_t) -22147, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float)7.4707513E37F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -8.574819E37F, &PH) ;
        p100_ground_distance_SET((float)7.2697683E37F, PH.base.pack) ;
        p100_flow_rate_x_SET((float)1.3167894E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)17818, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)6369831151444093265L, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.4268141E38F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_z_SET((float)9.93876E37F, PH.base.pack) ;
        p101_yaw_SET((float)2.7734422E38F, PH.base.pack) ;
        p101_y_SET((float)1.249295E38F, PH.base.pack) ;
        p101_roll_SET((float) -2.6083477E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)5108322630986999685L, PH.base.pack) ;
        p101_x_SET((float) -6.0385317E37F, PH.base.pack) ;
        p101_pitch_SET((float)1.9522136E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float)1.4322023E38F, PH.base.pack) ;
        p102_z_SET((float)2.4547164E37F, PH.base.pack) ;
        p102_y_SET((float) -9.148671E37F, PH.base.pack) ;
        p102_pitch_SET((float) -5.568722E37F, PH.base.pack) ;
        p102_roll_SET((float)3.3733517E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)2303744813218333173L, PH.base.pack) ;
        p102_yaw_SET((float) -3.6257022E36F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -2.4633603E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)1429102225769044302L, PH.base.pack) ;
        p103_y_SET((float)2.2230576E37F, PH.base.pack) ;
        p103_x_SET((float)2.9143571E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_y_SET((float)6.915259E37F, PH.base.pack) ;
        p104_z_SET((float)3.1161663E38F, PH.base.pack) ;
        p104_pitch_SET((float)1.9745297E38F, PH.base.pack) ;
        p104_x_SET((float)4.364791E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)8642262507426670032L, PH.base.pack) ;
        p104_yaw_SET((float)2.398004E38F, PH.base.pack) ;
        p104_roll_SET((float) -2.9553822E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_zgyro_SET((float)2.278825E37F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8885696254686599720L, PH.base.pack) ;
        p105_yacc_SET((float)3.2445598E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float)2.6015738E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)32977, PH.base.pack) ;
        p105_pressure_alt_SET((float) -8.855815E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.2140743E38F, PH.base.pack) ;
        p105_xacc_SET((float)2.8243209E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.1474489E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -2.1482098E38F, PH.base.pack) ;
        p105_zmag_SET((float)1.1617395E38F, PH.base.pack) ;
        p105_temperature_SET((float)7.9227057E37F, PH.base.pack) ;
        p105_xmag_SET((float)7.263052E37F, PH.base.pack) ;
        p105_zacc_SET((float)8.273549E37F, PH.base.pack) ;
        p105_xgyro_SET((float)3.1756004E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_xgyro_SET((float) -2.9427292E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -3.402776E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -5.1696465E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6005860237821084469L, PH.base.pack) ;
        p106_distance_SET((float)2.998685E38F, PH.base.pack) ;
        p106_integrated_y_SET((float) -1.5461865E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)19870, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2523237366L, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.5072466E38F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3621392674L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_diff_pressure_SET((float) -5.772684E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)1766133564866033938L, PH.base.pack) ;
        p107_ygyro_SET((float)3.3283584E38F, PH.base.pack) ;
        p107_zmag_SET((float)1.3147616E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -1.4827752E38F, PH.base.pack) ;
        p107_xgyro_SET((float)3.307118E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)1.8410025E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)551810L, PH.base.pack) ;
        p107_xacc_SET((float) -1.7955368E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.9764844E38F, PH.base.pack) ;
        p107_zgyro_SET((float)3.3523732E38F, PH.base.pack) ;
        p107_zacc_SET((float)2.9705319E38F, PH.base.pack) ;
        p107_temperature_SET((float)3.6846334E37F, PH.base.pack) ;
        p107_xmag_SET((float) -1.4836642E38F, PH.base.pack) ;
        p107_yacc_SET((float)2.8137665E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_alt_SET((float) -2.2942235E38F, PH.base.pack) ;
        p108_q4_SET((float)1.4278976E38F, PH.base.pack) ;
        p108_roll_SET((float)1.0557409E38F, PH.base.pack) ;
        p108_q2_SET((float)2.042374E38F, PH.base.pack) ;
        p108_vd_SET((float) -3.9569317E36F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.6550784E38F, PH.base.pack) ;
        p108_vn_SET((float) -3.3543414E38F, PH.base.pack) ;
        p108_yacc_SET((float) -9.3431604E36F, PH.base.pack) ;
        p108_q1_SET((float)1.0102199E38F, PH.base.pack) ;
        p108_yaw_SET((float) -2.9591257E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.0856302E38F, PH.base.pack) ;
        p108_xgyro_SET((float)2.60389E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -8.628874E37F, PH.base.pack) ;
        p108_ve_SET((float) -1.5239533E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)3.2970909E38F, PH.base.pack) ;
        p108_lat_SET((float) -1.2751946E38F, PH.base.pack) ;
        p108_ygyro_SET((float)2.4811195E37F, PH.base.pack) ;
        p108_zacc_SET((float)2.43498E38F, PH.base.pack) ;
        p108_xacc_SET((float) -1.4077949E38F, PH.base.pack) ;
        p108_q3_SET((float)2.544694E38F, PH.base.pack) ;
        p108_lon_SET((float) -3.0327877E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_fixed__SET((uint16_t)(uint16_t)6364, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)31164, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)126, (uint8_t)230, (uint8_t)196, (uint8_t)1, (uint8_t)255, (uint8_t)112, (uint8_t)93, (uint8_t)149, (uint8_t)104, (uint8_t)169, (uint8_t)186, (uint8_t)191, (uint8_t)152, (uint8_t)242, (uint8_t)81, (uint8_t)149, (uint8_t)84, (uint8_t)184, (uint8_t)194, (uint8_t)249, (uint8_t)27, (uint8_t)50, (uint8_t)140, (uint8_t)14, (uint8_t)190, (uint8_t)191, (uint8_t)153, (uint8_t)101, (uint8_t)248, (uint8_t)83, (uint8_t)155, (uint8_t)120, (uint8_t)109, (uint8_t)255, (uint8_t)249, (uint8_t)178, (uint8_t)57, (uint8_t)15, (uint8_t)10, (uint8_t)0, (uint8_t)94, (uint8_t)20, (uint8_t)44, (uint8_t)89, (uint8_t)20, (uint8_t)16, (uint8_t)222, (uint8_t)173, (uint8_t)130, (uint8_t)214, (uint8_t)119, (uint8_t)114, (uint8_t)0, (uint8_t)26, (uint8_t)76, (uint8_t)97, (uint8_t)65, (uint8_t)121, (uint8_t)124, (uint8_t)14, (uint8_t)13, (uint8_t)124, (uint8_t)179, (uint8_t)15, (uint8_t)0, (uint8_t)121, (uint8_t)85, (uint8_t)122, (uint8_t)43, (uint8_t)237, (uint8_t)219, (uint8_t)26, (uint8_t)136, (uint8_t)85, (uint8_t)115, (uint8_t)9, (uint8_t)168, (uint8_t)127, (uint8_t)47, (uint8_t)227, (uint8_t)47, (uint8_t)163, (uint8_t)144, (uint8_t)93, (uint8_t)241, (uint8_t)230, (uint8_t)40, (uint8_t)102, (uint8_t)199, (uint8_t)85, (uint8_t)92, (uint8_t)21, (uint8_t)115, (uint8_t)90, (uint8_t)58, (uint8_t)236, (uint8_t)200, (uint8_t)96, (uint8_t)255, (uint8_t)68, (uint8_t)173, (uint8_t)127, (uint8_t)172, (uint8_t)112, (uint8_t)0, (uint8_t)135, (uint8_t)126, (uint8_t)42, (uint8_t)40, (uint8_t)9, (uint8_t)76, (uint8_t)93, (uint8_t)9, (uint8_t)31, (uint8_t)208, (uint8_t)185, (uint8_t)34, (uint8_t)74, (uint8_t)103, (uint8_t)76, (uint8_t)145, (uint8_t)95, (uint8_t)111, (uint8_t)72, (uint8_t)233, (uint8_t)160, (uint8_t)93, (uint8_t)92, (uint8_t)119, (uint8_t)197, (uint8_t)255, (uint8_t)18, (uint8_t)89, (uint8_t)116, (uint8_t)142, (uint8_t)143, (uint8_t)7, (uint8_t)196, (uint8_t)194, (uint8_t)186, (uint8_t)191, (uint8_t)152, (uint8_t)121, (uint8_t)127, (uint8_t)151, (uint8_t)44, (uint8_t)91, (uint8_t)51, (uint8_t)15, (uint8_t)34, (uint8_t)130, (uint8_t)38, (uint8_t)10, (uint8_t)248, (uint8_t)179, (uint8_t)33, (uint8_t)38, (uint8_t)241, (uint8_t)24, (uint8_t)244, (uint8_t)131, (uint8_t)183, (uint8_t)223, (uint8_t)29, (uint8_t)203, (uint8_t)228, (uint8_t)228, (uint8_t)189, (uint8_t)117, (uint8_t)234, (uint8_t)97, (uint8_t)110, (uint8_t)141, (uint8_t)65, (uint8_t)121, (uint8_t)249, (uint8_t)110, (uint8_t)214, (uint8_t)18, (uint8_t)212, (uint8_t)23, (uint8_t)123, (uint8_t)61, (uint8_t)245, (uint8_t)71, (uint8_t)2, (uint8_t)253, (uint8_t)155, (uint8_t)76, (uint8_t)80, (uint8_t)85, (uint8_t)40, (uint8_t)140, (uint8_t)27, (uint8_t)44, (uint8_t)177, (uint8_t)70, (uint8_t)235, (uint8_t)83, (uint8_t)94, (uint8_t)235, (uint8_t)134, (uint8_t)92, (uint8_t)6, (uint8_t)68, (uint8_t)87, (uint8_t)226, (uint8_t)179, (uint8_t)140, (uint8_t)171, (uint8_t)242, (uint8_t)175, (uint8_t)178, (uint8_t)130, (uint8_t)13, (uint8_t)100, (uint8_t)142, (uint8_t)72, (uint8_t)23, (uint8_t)43, (uint8_t)39, (uint8_t)35, (uint8_t)12, (uint8_t)174, (uint8_t)227, (uint8_t)110, (uint8_t)186, (uint8_t)232, (uint8_t)85, (uint8_t)172, (uint8_t)119, (uint8_t)195, (uint8_t)124, (uint8_t)65, (uint8_t)92, (uint8_t)133, (uint8_t)188, (uint8_t)109, (uint8_t)171, (uint8_t)186, (uint8_t)74, (uint8_t)97, (uint8_t)188, (uint8_t)13, (uint8_t)65, (uint8_t)241, (uint8_t)147, (uint8_t)230, (uint8_t)65, (uint8_t)17, (uint8_t)1};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t) -3253898904876412462L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -3658619433269897817L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)4129047937L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)699184138294089148L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_satellites_visible_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p113_alt_SET((int32_t)269405207, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -305, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)64346, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)8624372883010290511L, PH.base.pack) ;
        p113_lon_SET((int32_t)1814575555, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -24279, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)14140, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)30819, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)19403, PH.base.pack) ;
        p113_lat_SET((int32_t)2028392830, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)50152, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_distance_SET((float)4.56764E37F, PH.base.pack) ;
        p114_integrated_y_SET((float)2.3014257E37F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -29003, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)3302444927659862800L, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -1.2972822E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1426967680L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)817634869L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -2.6264007E38F, PH.base.pack) ;
        p114_integrated_x_SET((float)1.6051558E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)2.0297577E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        {
            float attitude_quaternion[] =  {-4.255923E37F, 3.3697552E38F, 1.2308578E38F, 2.7591655E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_alt_SET((int32_t) -108324409, PH.base.pack) ;
        p115_pitchspeed_SET((float) -6.5522446E37F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)13858, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)10226, PH.base.pack) ;
        p115_rollspeed_SET((float)1.1060738E38F, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)8860663836718199574L, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)33166, PH.base.pack) ;
        p115_lat_SET((int32_t) -1784560112, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -1443, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)9980, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)32313, PH.base.pack) ;
        p115_lon_SET((int32_t)2080233522, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)18985, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -18974, PH.base.pack) ;
        p115_yawspeed_SET((float) -3.3140768E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t)22305, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -22881, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)30977, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)7063, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)1068916683L, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -27543, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)20396, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)14764, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)27245, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)17311, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)43155, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)34194, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_time_utc_SET((uint32_t)1316573780L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)19136, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)48661, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)16651, PH.base.pack) ;
        p118_size_SET((uint32_t)4069491901L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)147677207L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)56077, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p119_ofs_SET((uint32_t)4071583614L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)3434369216L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)10052, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)45, (uint8_t)135, (uint8_t)12, (uint8_t)67, (uint8_t)58, (uint8_t)49, (uint8_t)51, (uint8_t)162, (uint8_t)53, (uint8_t)193, (uint8_t)147, (uint8_t)63, (uint8_t)71, (uint8_t)65, (uint8_t)0, (uint8_t)106, (uint8_t)91, (uint8_t)152, (uint8_t)83, (uint8_t)254, (uint8_t)64, (uint8_t)227, (uint8_t)14, (uint8_t)127, (uint8_t)189, (uint8_t)19, (uint8_t)23, (uint8_t)149, (uint8_t)250, (uint8_t)223, (uint8_t)162, (uint8_t)220, (uint8_t)231, (uint8_t)97, (uint8_t)27, (uint8_t)100, (uint8_t)164, (uint8_t)140, (uint8_t)89, (uint8_t)224, (uint8_t)170, (uint8_t)105, (uint8_t)224, (uint8_t)68, (uint8_t)219, (uint8_t)215, (uint8_t)187, (uint8_t)131, (uint8_t)216, (uint8_t)228, (uint8_t)172, (uint8_t)33, (uint8_t)27, (uint8_t)100, (uint8_t)185, (uint8_t)82, (uint8_t)65, (uint8_t)50, (uint8_t)56, (uint8_t)39, (uint8_t)101, (uint8_t)127, (uint8_t)34, (uint8_t)41, (uint8_t)242, (uint8_t)232, (uint8_t)165, (uint8_t)252, (uint8_t)96, (uint8_t)255, (uint8_t)137, (uint8_t)249, (uint8_t)72, (uint8_t)225, (uint8_t)219, (uint8_t)226, (uint8_t)239, (uint8_t)16, (uint8_t)11, (uint8_t)180, (uint8_t)214, (uint8_t)65, (uint8_t)227, (uint8_t)229, (uint8_t)189, (uint8_t)219, (uint8_t)81, (uint8_t)124, (uint8_t)75, (uint8_t)162};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)55, (uint8_t)9, (uint8_t)47, (uint8_t)247, (uint8_t)59, (uint8_t)111, (uint8_t)127, (uint8_t)82, (uint8_t)43, (uint8_t)190, (uint8_t)6, (uint8_t)4, (uint8_t)207, (uint8_t)200, (uint8_t)13, (uint8_t)230, (uint8_t)66, (uint8_t)109, (uint8_t)29, (uint8_t)8, (uint8_t)65, (uint8_t)84, (uint8_t)123, (uint8_t)62, (uint8_t)6, (uint8_t)194, (uint8_t)68, (uint8_t)217, (uint8_t)39, (uint8_t)224, (uint8_t)87, (uint8_t)90, (uint8_t)83, (uint8_t)252, (uint8_t)211, (uint8_t)197, (uint8_t)162, (uint8_t)238, (uint8_t)51, (uint8_t)210, (uint8_t)62, (uint8_t)10, (uint8_t)97, (uint8_t)131, (uint8_t)59, (uint8_t)172, (uint8_t)42, (uint8_t)231, (uint8_t)180, (uint8_t)41, (uint8_t)201, (uint8_t)17, (uint8_t)140, (uint8_t)50, (uint8_t)14, (uint8_t)235, (uint8_t)100, (uint8_t)93, (uint8_t)167, (uint8_t)60, (uint8_t)12, (uint8_t)186, (uint8_t)134, (uint8_t)211, (uint8_t)109, (uint8_t)122, (uint8_t)4, (uint8_t)54, (uint8_t)223, (uint8_t)7, (uint8_t)178, (uint8_t)166, (uint8_t)0, (uint8_t)63, (uint8_t)216, (uint8_t)142, (uint8_t)227, (uint8_t)148, (uint8_t)48, (uint8_t)23, (uint8_t)163, (uint8_t)162, (uint8_t)149, (uint8_t)38, (uint8_t)188, (uint8_t)145, (uint8_t)2, (uint8_t)17, (uint8_t)42, (uint8_t)55, (uint8_t)172, (uint8_t)106, (uint8_t)166, (uint8_t)230, (uint8_t)117, (uint8_t)28, (uint8_t)80, (uint8_t)35, (uint8_t)223, (uint8_t)62, (uint8_t)80, (uint8_t)234, (uint8_t)5, (uint8_t)228, (uint8_t)49, (uint8_t)188, (uint8_t)42, (uint8_t)216, (uint8_t)79, (uint8_t)64};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_satellites_visible_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)50155, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)2497062888L, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)21674, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)27151, PH.base.pack) ;
        p124_lon_SET((int32_t) -1502304725, PH.base.pack) ;
        p124_alt_SET((int32_t) -1329757083, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)47065, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)642860949678860055L, PH.base.pack) ;
        p124_lat_SET((int32_t) -124650477, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID), PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)19562, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)382, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)19437, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE), PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)19, (uint8_t)219, (uint8_t)21, (uint8_t)128, (uint8_t)44, (uint8_t)80, (uint8_t)80, (uint8_t)247, (uint8_t)242, (uint8_t)130, (uint8_t)191, (uint8_t)91, (uint8_t)33, (uint8_t)168, (uint8_t)45, (uint8_t)169, (uint8_t)183, (uint8_t)239, (uint8_t)146, (uint8_t)37, (uint8_t)255, (uint8_t)155, (uint8_t)104, (uint8_t)9, (uint8_t)67, (uint8_t)238, (uint8_t)121, (uint8_t)118, (uint8_t)33, (uint8_t)68, (uint8_t)213, (uint8_t)23, (uint8_t)130, (uint8_t)33, (uint8_t)32, (uint8_t)114, (uint8_t)109, (uint8_t)69, (uint8_t)230, (uint8_t)112, (uint8_t)173, (uint8_t)167, (uint8_t)196, (uint8_t)154, (uint8_t)156, (uint8_t)155, (uint8_t)126, (uint8_t)103, (uint8_t)9, (uint8_t)219, (uint8_t)143, (uint8_t)127, (uint8_t)57, (uint8_t)61, (uint8_t)43, (uint8_t)235, (uint8_t)145, (uint8_t)190, (uint8_t)71, (uint8_t)221, (uint8_t)64, (uint8_t)192, (uint8_t)160, (uint8_t)204, (uint8_t)234, (uint8_t)195, (uint8_t)26, (uint8_t)223, (uint8_t)130, (uint8_t)118};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_baudrate_SET((uint32_t)3384153085L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)694879360, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)40353, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -719684696, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -617679716, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p127_tow_SET((uint32_t)2299043188L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)946157739L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1837940328, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)2040511750L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1449626114, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)535434086L, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)8961, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)949540057L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)2009681827, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1948893964, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)727664551, PH.base.pack) ;
        p128_tow_SET((uint32_t)3408954245L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_xmag_SET((int16_t)(int16_t) -8347, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -31961, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)13993, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -23677, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -5823, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -30569, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1320672394L, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)21444, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)11108, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)31616, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_payload_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)35711, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)37863, PH.base.pack) ;
        p130_size_SET((uint32_t)1151343869L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)48596, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)25246, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)184, (uint8_t)246, (uint8_t)155, (uint8_t)165, (uint8_t)237, (uint8_t)218, (uint8_t)117, (uint8_t)35, (uint8_t)163, (uint8_t)25, (uint8_t)119, (uint8_t)100, (uint8_t)166, (uint8_t)30, (uint8_t)154, (uint8_t)150, (uint8_t)190, (uint8_t)204, (uint8_t)17, (uint8_t)193, (uint8_t)109, (uint8_t)237, (uint8_t)31, (uint8_t)133, (uint8_t)252, (uint8_t)85, (uint8_t)239, (uint8_t)178, (uint8_t)103, (uint8_t)84, (uint8_t)174, (uint8_t)126, (uint8_t)249, (uint8_t)26, (uint8_t)189, (uint8_t)186, (uint8_t)26, (uint8_t)197, (uint8_t)118, (uint8_t)119, (uint8_t)166, (uint8_t)209, (uint8_t)193, (uint8_t)113, (uint8_t)61, (uint8_t)137, (uint8_t)8, (uint8_t)145, (uint8_t)198, (uint8_t)249, (uint8_t)42, (uint8_t)156, (uint8_t)73, (uint8_t)150, (uint8_t)60, (uint8_t)236, (uint8_t)2, (uint8_t)108, (uint8_t)58, (uint8_t)234, (uint8_t)231, (uint8_t)14, (uint8_t)51, (uint8_t)61, (uint8_t)107, (uint8_t)192, (uint8_t)37, (uint8_t)233, (uint8_t)225, (uint8_t)90, (uint8_t)104, (uint8_t)112, (uint8_t)213, (uint8_t)71, (uint8_t)231, (uint8_t)27, (uint8_t)236, (uint8_t)44, (uint8_t)228, (uint8_t)65, (uint8_t)32, (uint8_t)152, (uint8_t)154, (uint8_t)89, (uint8_t)81, (uint8_t)23, (uint8_t)22, (uint8_t)162, (uint8_t)235, (uint8_t)89, (uint8_t)81, (uint8_t)119, (uint8_t)129, (uint8_t)78, (uint8_t)190, (uint8_t)217, (uint8_t)125, (uint8_t)142, (uint8_t)230, (uint8_t)56, (uint8_t)232, (uint8_t)95, (uint8_t)211, (uint8_t)177, (uint8_t)150, (uint8_t)141, (uint8_t)162, (uint8_t)192, (uint8_t)233, (uint8_t)189, (uint8_t)1, (uint8_t)220, (uint8_t)245, (uint8_t)240, (uint8_t)120, (uint8_t)29, (uint8_t)140, (uint8_t)66, (uint8_t)202, (uint8_t)88, (uint8_t)80, (uint8_t)234, (uint8_t)34, (uint8_t)196, (uint8_t)17, (uint8_t)48, (uint8_t)180, (uint8_t)12, (uint8_t)217, (uint8_t)225, (uint8_t)39, (uint8_t)147, (uint8_t)186, (uint8_t)148, (uint8_t)125, (uint8_t)70, (uint8_t)61, (uint8_t)161, (uint8_t)162, (uint8_t)251, (uint8_t)191, (uint8_t)243, (uint8_t)96, (uint8_t)162, (uint8_t)169, (uint8_t)47, (uint8_t)94, (uint8_t)157, (uint8_t)148, (uint8_t)217, (uint8_t)243, (uint8_t)39, (uint8_t)27, (uint8_t)71, (uint8_t)245, (uint8_t)113, (uint8_t)21, (uint8_t)131, (uint8_t)155, (uint8_t)80, (uint8_t)75, (uint8_t)83, (uint8_t)213, (uint8_t)145, (uint8_t)195, (uint8_t)179, (uint8_t)47, (uint8_t)151, (uint8_t)109, (uint8_t)20, (uint8_t)188, (uint8_t)211, (uint8_t)158, (uint8_t)229, (uint8_t)85, (uint8_t)144, (uint8_t)183, (uint8_t)94, (uint8_t)43, (uint8_t)159, (uint8_t)201, (uint8_t)9, (uint8_t)146, (uint8_t)157, (uint8_t)43, (uint8_t)168, (uint8_t)203, (uint8_t)3, (uint8_t)79, (uint8_t)78, (uint8_t)146, (uint8_t)168, (uint8_t)251, (uint8_t)48, (uint8_t)185, (uint8_t)52, (uint8_t)200, (uint8_t)83, (uint8_t)210, (uint8_t)28, (uint8_t)12, (uint8_t)123, (uint8_t)53, (uint8_t)115, (uint8_t)224, (uint8_t)221, (uint8_t)201, (uint8_t)188, (uint8_t)215, (uint8_t)58, (uint8_t)48, (uint8_t)137, (uint8_t)38, (uint8_t)48, (uint8_t)6, (uint8_t)61, (uint8_t)70, (uint8_t)96, (uint8_t)202, (uint8_t)255, (uint8_t)109, (uint8_t)168, (uint8_t)242, (uint8_t)82, (uint8_t)27, (uint8_t)216, (uint8_t)145, (uint8_t)139, (uint8_t)198, (uint8_t)29, (uint8_t)175, (uint8_t)168, (uint8_t)91, (uint8_t)95, (uint8_t)145, (uint8_t)125, (uint8_t)54, (uint8_t)251, (uint8_t)41, (uint8_t)157, (uint8_t)84, (uint8_t)231, (uint8_t)68, (uint8_t)215, (uint8_t)10, (uint8_t)16, (uint8_t)37, (uint8_t)89, (uint8_t)206, (uint8_t)121, (uint8_t)173, (uint8_t)100, (uint8_t)105};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_min_distance_SET((uint16_t)(uint16_t)34954, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1747155244L, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)23218, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)130, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)8580945693956203757L, PH.base.pack) ;
        p133_lon_SET((int32_t)279601308, PH.base.pack) ;
        p133_lat_SET((int32_t)2130709184, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)12669, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)22838, PH.base.pack) ;
        p134_lon_SET((int32_t)1080098559, PH.base.pack) ;
        p134_lat_SET((int32_t)968116785, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -21249, (int16_t)26980, (int16_t) -1566, (int16_t)20273, (int16_t)4585, (int16_t) -13536, (int16_t) -31043, (int16_t)2205, (int16_t) -7519, (int16_t) -22443, (int16_t)32571, (int16_t) -29749, (int16_t)32131, (int16_t) -1588, (int16_t)15989, (int16_t) -27791};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -1149736305, PH.base.pack) ;
        p135_lat_SET((int32_t) -1961865049, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)53979, PH.base.pack) ;
        p136_terrain_height_SET((float)7.4192497E36F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)46340, PH.base.pack) ;
        p136_lat_SET((int32_t) -1732168533, PH.base.pack) ;
        p136_current_height_SET((float) -2.1234468E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)8797, PH.base.pack) ;
        p136_lon_SET((int32_t) -1738178040, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)3825485953L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -11, PH.base.pack) ;
        p137_press_abs_SET((float)1.950535E38F, PH.base.pack) ;
        p137_press_diff_SET((float) -2.2941567E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_y_SET((float)2.997715E37F, PH.base.pack) ;
        {
            float q[] =  {4.336294E37F, 2.1302092E38F, 1.2186463E38F, -2.4219775E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -1.8016398E38F, PH.base.pack) ;
        p138_z_SET((float)1.5758249E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)7362590263729988431L, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        {
            float controls[] =  {6.658204E37F, 2.4667138E38F, -2.677045E38F, -7.922227E37F, 2.994173E38F, -2.6249302E38F, 2.1902264E38F, 8.051566E36F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_time_usec_SET((uint64_t)4800704983101110186L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {1.88495E38F, -2.6594551E38F, 1.0275828E38F, 8.041979E36F, 2.6825734E38F, 2.2985327E38F, -3.382826E38F, -2.7432764E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)7179024428114352859L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_bottom_clearance_SET((float) -1.5052808E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)529910638981686912L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)3.2881285E38F, PH.base.pack) ;
        p141_altitude_local_SET((float)3.3215503E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)1.9944774E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -8.119583E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float)1.136537E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)64, (uint8_t)50, (uint8_t)162, (uint8_t)160, (uint8_t)209, (uint8_t)19, (uint8_t)51, (uint8_t)135, (uint8_t)88, (uint8_t)175, (uint8_t)237, (uint8_t)144, (uint8_t)255, (uint8_t)176, (uint8_t)111, (uint8_t)241, (uint8_t)185, (uint8_t)146, (uint8_t)152, (uint8_t)160, (uint8_t)52, (uint8_t)43, (uint8_t)168, (uint8_t)213, (uint8_t)79, (uint8_t)86, (uint8_t)164, (uint8_t)7, (uint8_t)220, (uint8_t)242, (uint8_t)30, (uint8_t)9, (uint8_t)15, (uint8_t)121, (uint8_t)36, (uint8_t)0, (uint8_t)128, (uint8_t)196, (uint8_t)216, (uint8_t)205, (uint8_t)253, (uint8_t)170, (uint8_t)45, (uint8_t)11, (uint8_t)18, (uint8_t)91, (uint8_t)5, (uint8_t)31, (uint8_t)131, (uint8_t)62, (uint8_t)145, (uint8_t)141, (uint8_t)11, (uint8_t)149, (uint8_t)216, (uint8_t)241, (uint8_t)62, (uint8_t)22, (uint8_t)96, (uint8_t)16, (uint8_t)79, (uint8_t)138, (uint8_t)8, (uint8_t)204, (uint8_t)174, (uint8_t)236, (uint8_t)230, (uint8_t)232, (uint8_t)92, (uint8_t)161, (uint8_t)38, (uint8_t)116, (uint8_t)61, (uint8_t)63, (uint8_t)41, (uint8_t)96, (uint8_t)136, (uint8_t)205, (uint8_t)152, (uint8_t)179, (uint8_t)181, (uint8_t)21, (uint8_t)19, (uint8_t)57, (uint8_t)84, (uint8_t)74, (uint8_t)157, (uint8_t)56, (uint8_t)75, (uint8_t)136, (uint8_t)192, (uint8_t)244, (uint8_t)217, (uint8_t)241, (uint8_t)233, (uint8_t)254, (uint8_t)73, (uint8_t)93, (uint8_t)55, (uint8_t)9, (uint8_t)211, (uint8_t)10, (uint8_t)235, (uint8_t)249, (uint8_t)232, (uint8_t)159, (uint8_t)52, (uint8_t)82, (uint8_t)102, (uint8_t)232, (uint8_t)243, (uint8_t)30, (uint8_t)61, (uint8_t)228, (uint8_t)70, (uint8_t)62, (uint8_t)120, (uint8_t)33, (uint8_t)28, (uint8_t)176};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)174, (uint8_t)9, (uint8_t)156, (uint8_t)190, (uint8_t)93, (uint8_t)152, (uint8_t)224, (uint8_t)85, (uint8_t)11, (uint8_t)131, (uint8_t)157, (uint8_t)120, (uint8_t)98, (uint8_t)183, (uint8_t)98, (uint8_t)159, (uint8_t)65, (uint8_t)101, (uint8_t)40, (uint8_t)162, (uint8_t)84, (uint8_t)212, (uint8_t)29, (uint8_t)37, (uint8_t)68, (uint8_t)80, (uint8_t)149, (uint8_t)15, (uint8_t)174, (uint8_t)230, (uint8_t)154, (uint8_t)149, (uint8_t)30, (uint8_t)86, (uint8_t)29, (uint8_t)251, (uint8_t)102, (uint8_t)62, (uint8_t)150, (uint8_t)162, (uint8_t)165, (uint8_t)140, (uint8_t)216, (uint8_t)188, (uint8_t)226, (uint8_t)130, (uint8_t)105, (uint8_t)134, (uint8_t)209, (uint8_t)165, (uint8_t)114, (uint8_t)116, (uint8_t)17, (uint8_t)51, (uint8_t)220, (uint8_t)167, (uint8_t)176, (uint8_t)151, (uint8_t)116, (uint8_t)181, (uint8_t)227, (uint8_t)158, (uint8_t)109, (uint8_t)198, (uint8_t)6, (uint8_t)70, (uint8_t)152, (uint8_t)64, (uint8_t)191, (uint8_t)38, (uint8_t)226, (uint8_t)96, (uint8_t)42, (uint8_t)224, (uint8_t)216, (uint8_t)208, (uint8_t)209, (uint8_t)224, (uint8_t)156, (uint8_t)36, (uint8_t)230, (uint8_t)195, (uint8_t)186, (uint8_t)166, (uint8_t)101, (uint8_t)129, (uint8_t)255, (uint8_t)64, (uint8_t)13, (uint8_t)73, (uint8_t)104, (uint8_t)206, (uint8_t)166, (uint8_t)164, (uint8_t)83, (uint8_t)87, (uint8_t)200, (uint8_t)226, (uint8_t)70, (uint8_t)140, (uint8_t)65, (uint8_t)208, (uint8_t)205, (uint8_t)156, (uint8_t)214, (uint8_t)121, (uint8_t)204, (uint8_t)219, (uint8_t)205, (uint8_t)175, (uint8_t)41, (uint8_t)252, (uint8_t)191, (uint8_t)107, (uint8_t)8, (uint8_t)133, (uint8_t)114, (uint8_t)30, (uint8_t)136, (uint8_t)17};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float)1.7418178E38F, PH.base.pack) ;
        p143_press_diff_SET((float) -1.637617E36F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)30048, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)324482988L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        p144_lat_SET((int32_t) -1266844792, PH.base.pack) ;
        {
            float position_cov[] =  {-3.363802E38F, 3.2238965E38F, -2.4109483E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {8.957169E37F, 1.1031359E38F, 3.2809542E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {1.5953186E38F, 1.5390273E38F, -1.2524508E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)4.993015E37F, PH.base.pack) ;
        p144_lon_SET((int32_t)1449170487, PH.base.pack) ;
        {
            float attitude_q[] =  {-5.771193E37F, -1.9231081E38F, 3.2567337E38F, -2.616168E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {8.1137244E37F, 2.3725779E38F, -7.194086E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)2165679574356749825L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)4950983836715804360L, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_time_usec_SET((uint64_t)7600725570938640001L, PH.base.pack) ;
        {
            float pos_variance[] =  {9.584607E37F, 3.069771E38F, -3.2085752E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_pitch_rate_SET((float)2.499509E37F, PH.base.pack) ;
        p146_y_pos_SET((float)1.062275E38F, PH.base.pack) ;
        p146_roll_rate_SET((float)2.4483897E38F, PH.base.pack) ;
        p146_y_vel_SET((float)9.965751E37F, PH.base.pack) ;
        {
            float q[] =  {-7.556189E37F, 1.0221413E38F, -2.7653171E38F, -3.0230193E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float)8.596693E36F, PH.base.pack) ;
        p146_z_pos_SET((float) -9.432074E37F, PH.base.pack) ;
        p146_x_pos_SET((float)3.3645862E36F, PH.base.pack) ;
        p146_z_acc_SET((float) -2.2626255E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -2.4975996E38F, PH.base.pack) ;
        p146_x_acc_SET((float)2.2267078E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -1.9671226E38F, PH.base.pack) ;
        p146_x_vel_SET((float)1.4182401E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.0567986E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-2.3055365E37F, 1.703949E38F, -9.595443E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_remaining_SET((int8_t)(int8_t) -102, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1549919477, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -23223, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)42894, (uint16_t)50044, (uint16_t)29283, (uint16_t)17930, (uint16_t)12515, (uint16_t)14343, (uint16_t)55174, (uint16_t)14309, (uint16_t)44738, (uint16_t)8892};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_battery_SET((int16_t)(int16_t) -24231, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1950748840, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t os_custom_version[] =  {(uint8_t)104, (uint8_t)108, (uint8_t)160, (uint8_t)216, (uint8_t)150, (uint8_t)37, (uint8_t)94, (uint8_t)39};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)247, (uint8_t)30, (uint8_t)114, (uint8_t)104, (uint8_t)35, (uint8_t)70, (uint8_t)77, (uint8_t)86, (uint8_t)159, (uint8_t)148, (uint8_t)146, (uint8_t)216, (uint8_t)226, (uint8_t)80, (uint8_t)51, (uint8_t)85, (uint8_t)143, (uint8_t)211};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT), PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)169, (uint8_t)7, (uint8_t)94, (uint8_t)77, (uint8_t)239, (uint8_t)57, (uint8_t)229, (uint8_t)57};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)1467254107L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)1181809887L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)21158, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)54237, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1174098223L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)92, (uint8_t)1, (uint8_t)81, (uint8_t)99, (uint8_t)206, (uint8_t)67, (uint8_t)189, (uint8_t)57};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)3751005983L, PH.base.pack) ;
        p148_uid_SET((uint64_t)3583184188896619140L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
        p149_size_x_SET((float) -1.6792512E38F, PH.base.pack) ;
        p149_angle_x_SET((float)2.5695783E38F, PH.base.pack) ;
        p149_y_SET((float) -1.753927E38F, &PH) ;
        p149_distance_SET((float) -2.3898528E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)231, &PH) ;
        p149_z_SET((float)1.0720301E38F, &PH) ;
        p149_target_num_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)5317533209621957290L, PH.base.pack) ;
        p149_x_SET((float)1.7527618E38F, &PH) ;
        p149_size_y_SET((float)5.7778105E37F, PH.base.pack) ;
        p149_angle_y_SET((float) -1.6189391E38F, PH.base.pack) ;
        {
            float q[] =  {2.5424396E38F, 3.0043088E38F, 3.9621497E37F, -1.2407471E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_0_150(), &PH);
        {
            uint32_t ar_u32[] =  {2557929228L, 1541642112L, 2690608397L, 2596955902L};
            p150_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        p150_v1_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        {
            uint8_t ar_u8[] =  {(uint8_t)107, (uint8_t)203, (uint8_t)146, (uint8_t)253};
            p150_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)45416, (uint16_t)32761, (uint16_t)31205, (uint16_t)38254};
            p150_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t)63, (int8_t) -70, (int8_t) -80, (int8_t)102};
            p150_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_0_150(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_1_151(), &PH);
        {
            uint32_t ar_u32[] =  {957022200L, 2284539905L, 4258903440L, 4212684547L};
            p151_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_1_151(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_3_153(), &PH);
        p153_v_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        {
            uint32_t ar_u32[] =  {3482472394L, 1753280730L, 4194140847L, 610893673L};
            p153_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_3_153(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_4_154(), &PH);
        p154_v_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        {
            uint32_t ar_u32[] =  {3717773926L, 2911425666L, 3231432080L, 3680802255L};
            p154_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_4_154(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_5_155(), &PH);
        {
            char16_t* c2 = u"Dgv";
            p155_c2_SET_(c2, &PH) ;
        }
        {
            char16_t* c1 = u"mp";
            p155_c1_SET_(c1, &PH) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_5_155(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_6_156(), &PH);
        p156_v1_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        {
            uint32_t ar_u32[] =  {3640450294L, 1558377688L};
            p156_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)22132, (uint16_t)25664};
            p156_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            int16_t ar_i16[] =  {(int16_t)15790, (int16_t)23797};
            p156_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        p156_v3_SET((uint32_t)564803386L, PH.base.pack) ;
        {
            double ar_d[] =  {1.2583799612781677E308, 1.298016257020958E308};
            p156_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        {
            float ar_f[] =  {2.3929298E38F, -1.1445463E38F};
            p156_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            uint8_t ar_u8[] =  {(uint8_t)50, (uint8_t)4};
            p156_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            char16_t* ar_c = u"n";
            p156_ar_c_SET_(ar_c, &PH) ;
        }
        {
            int32_t ar_i32[] =  {1582561337, -1947838858};
            p156_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        p156_v2_SET((uint16_t)(uint16_t)29223, PH.base.pack) ;
        {
            int8_t ar_i8[] =  {(int8_t) -35, (int8_t)17};
            p156_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_6_156(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_7_157(), &PH);
        {
            uint8_t ar_u8[] =  {(uint8_t)158, (uint8_t)95};
            p157_ar_u8_SET(&ar_u8, 0, PH.base.pack) ;
        }
        {
            uint32_t ar_u32[] =  {3621943668L, 2178516584L};
            p157_ar_u32_SET(&ar_u32, 0, PH.base.pack) ;
        }
        {
            float ar_f[] =  {-2.501923E38F, 3.9260075E37F};
            p157_ar_f_SET(&ar_f, 0, PH.base.pack) ;
        }
        {
            int8_t ar_i8[] =  {(int8_t) -100, (int8_t)3};
            p157_ar_i8_SET(&ar_i8, 0, PH.base.pack) ;
        }
        {
            int32_t ar_i32[] =  {-1466887520, 326322049};
            p157_ar_i32_SET(&ar_i32, 0, PH.base.pack) ;
        }
        {
            int16_t ar_i16[] =  {(int16_t)31758, (int16_t) -19413};
            p157_ar_i16_SET(&ar_i16, 0, PH.base.pack) ;
        }
        {
            char16_t* ar_c = u"yrbtrkerlueiaboF";
            p157_ar_c_SET_(ar_c, &PH) ;
        }
        {
            double ar_d[] =  {-1.6167808777353305E308, 1.430167869280193E307};
            p157_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        {
            uint16_t ar_u16[] =  {(uint16_t)36324, (uint16_t)1823};
            p157_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_7_157(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ARRAY_TEST_8_158(), &PH);
        p158_v3_SET((uint32_t)1589646566L, PH.base.pack) ;
        {
            uint16_t ar_u16[] =  {(uint16_t)17786, (uint16_t)8819};
            p158_ar_u16_SET(&ar_u16, 0, PH.base.pack) ;
        }
        {
            double ar_d[] =  {-1.42666016685961E308, 4.885377199235113E307};
            p158_ar_d_SET(&ar_d, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ARRAY_TEST_8_158(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_vel_ratio_SET((float) -2.2226164E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)1.4933814E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -2.6097273E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -3.1252033E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)5508060286975452109L, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -2.6433677E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)1.3260667E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -3.1235797E38F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS), PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.665664E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float) -1.2695339E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)2.314555E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)3.1566484E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)4237176375741762941L, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -3.0454711E38F, PH.base.pack) ;
        p231_wind_x_SET((float)2.0469113E38F, PH.base.pack) ;
        p231_wind_z_SET((float)3.1608736E38F, PH.base.pack) ;
        p231_wind_alt_SET((float)1.6802953E36F, PH.base.pack) ;
        p231_wind_y_SET((float)1.4998136E38F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_fix_type_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2409201263L, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY), PH.base.pack) ;
        p232_ve_SET((float) -8.942892E37F, PH.base.pack) ;
        p232_alt_SET((float) -1.5644803E38F, PH.base.pack) ;
        p232_hdop_SET((float)2.9827368E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)2.9597086E38F, PH.base.pack) ;
        p232_vn_SET((float)3.6820666E37F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)2879729629079381597L, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)49358, PH.base.pack) ;
        p232_lat_SET((int32_t) -506346963, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -1.374141E38F, PH.base.pack) ;
        p232_lon_SET((int32_t) -231683496, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -9.645408E37F, PH.base.pack) ;
        p232_vdop_SET((float)2.5886877E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p232_vd_SET((float) -2.6584266E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p233_flags_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)242, (uint8_t)19, (uint8_t)162, (uint8_t)166, (uint8_t)129, (uint8_t)78, (uint8_t)127, (uint8_t)40, (uint8_t)47, (uint8_t)131, (uint8_t)247, (uint8_t)94, (uint8_t)193, (uint8_t)127, (uint8_t)57, (uint8_t)84, (uint8_t)95, (uint8_t)239, (uint8_t)108, (uint8_t)219, (uint8_t)233, (uint8_t)175, (uint8_t)219, (uint8_t)244, (uint8_t)23, (uint8_t)3, (uint8_t)16, (uint8_t)213, (uint8_t)211, (uint8_t)170, (uint8_t)151, (uint8_t)210, (uint8_t)82, (uint8_t)240, (uint8_t)220, (uint8_t)255, (uint8_t)17, (uint8_t)26, (uint8_t)41, (uint8_t)199, (uint8_t)71, (uint8_t)32, (uint8_t)149, (uint8_t)111, (uint8_t)173, (uint8_t)74, (uint8_t)135, (uint8_t)152, (uint8_t)210, (uint8_t)135, (uint8_t)32, (uint8_t)15, (uint8_t)147, (uint8_t)118, (uint8_t)254, (uint8_t)29, (uint8_t)169, (uint8_t)209, (uint8_t)250, (uint8_t)35, (uint8_t)194, (uint8_t)229, (uint8_t)99, (uint8_t)18, (uint8_t)252, (uint8_t)213, (uint8_t)17, (uint8_t)226, (uint8_t)132, (uint8_t)20, (uint8_t)5, (uint8_t)246, (uint8_t)254, (uint8_t)220, (uint8_t)164, (uint8_t)45, (uint8_t)135, (uint8_t)101, (uint8_t)177, (uint8_t)109, (uint8_t)15, (uint8_t)12, (uint8_t)157, (uint8_t)216, (uint8_t)241, (uint8_t)210, (uint8_t)11, (uint8_t)13, (uint8_t)207, (uint8_t)183, (uint8_t)20, (uint8_t)85, (uint8_t)48, (uint8_t)91, (uint8_t)79, (uint8_t)65, (uint8_t)146, (uint8_t)206, (uint8_t)253, (uint8_t)84, (uint8_t)84, (uint8_t)77, (uint8_t)40, (uint8_t)152, (uint8_t)208, (uint8_t)18, (uint8_t)148, (uint8_t)22, (uint8_t)96, (uint8_t)4, (uint8_t)73, (uint8_t)43, (uint8_t)230, (uint8_t)36, (uint8_t)174, (uint8_t)112, (uint8_t)24, (uint8_t)3, (uint8_t)150, (uint8_t)244, (uint8_t)52, (uint8_t)163, (uint8_t)70, (uint8_t)16, (uint8_t)8, (uint8_t)22, (uint8_t)24, (uint8_t)88, (uint8_t)127, (uint8_t)243, (uint8_t)186, (uint8_t)93, (uint8_t)213, (uint8_t)21, (uint8_t)67, (uint8_t)27, (uint8_t)182, (uint8_t)164, (uint8_t)200, (uint8_t)100, (uint8_t)83, (uint8_t)28, (uint8_t)86, (uint8_t)67, (uint8_t)136, (uint8_t)87, (uint8_t)170, (uint8_t)103, (uint8_t)69, (uint8_t)122, (uint8_t)107, (uint8_t)187, (uint8_t)98, (uint8_t)122, (uint8_t)105, (uint8_t)92, (uint8_t)15, (uint8_t)101, (uint8_t)203, (uint8_t)171, (uint8_t)117, (uint8_t)143, (uint8_t)143, (uint8_t)8, (uint8_t)150, (uint8_t)138, (uint8_t)109, (uint8_t)112, (uint8_t)168, (uint8_t)42, (uint8_t)104, (uint8_t)58, (uint8_t)195, (uint8_t)43, (uint8_t)21, (uint8_t)139, (uint8_t)107, (uint8_t)113, (uint8_t)95, (uint8_t)254};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_custom_mode_SET((uint32_t)355444115L, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -22577, PH.base.pack) ;
        p234_latitude_SET((int32_t)960973065, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)45, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)39731, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)13, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)4831, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -20324, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -20893, PH.base.pack) ;
        p234_longitude_SET((int32_t)1318275495, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -3635, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)35, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -23, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)23698, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED), PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_z_SET((float) -1.3241156E38F, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.0868268E37F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2792677979L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)2192051921L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6462386996895577439L, PH.base.pack) ;
        p241_vibration_y_SET((float)1.5704676E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1401918637L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_approach_y_SET((float)2.5776476E38F, PH.base.pack) ;
        p242_approach_z_SET((float)1.6606257E38F, PH.base.pack) ;
        p242_z_SET((float) -1.149653E38F, PH.base.pack) ;
        p242_y_SET((float)9.190323E37F, PH.base.pack) ;
        p242_approach_x_SET((float) -1.4133221E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)1841036408, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)4582738122282280312L, &PH) ;
        p242_longitude_SET((int32_t)1255143641, PH.base.pack) ;
        {
            float q[] =  {-7.9437185E36F, 2.6790015E38F, -2.600767E38F, 2.4565882E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_x_SET((float)7.4853957E37F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -594570178, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_y_SET((float) -1.1562075E37F, PH.base.pack) ;
        {
            float q[] =  {1.8780075E38F, 2.5285873E38F, -3.073129E38F, 1.7413351E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float)3.1565983E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p243_x_SET((float)7.143673E37F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.2381238E38F, PH.base.pack) ;
        p243_y_SET((float)3.0427342E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t) -322633197, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)6954490397893475402L, &PH) ;
        p243_altitude_SET((int32_t)53032462, PH.base.pack) ;
        p243_longitude_SET((int32_t) -282051097, PH.base.pack) ;
        p243_approach_x_SET((float)1.3486973E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)1109074266, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)34840, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_lon_SET((int32_t)1157344715, PH.base.pack) ;
        p246_lat_SET((int32_t)1716184006, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED), PH.base.pack) ;
        {
            char16_t* callsign = u"odacm";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_squawk_SET((uint16_t)(uint16_t)43240, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LARGE, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)53243, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -17320, PH.base.pack) ;
        p246_altitude_SET((int32_t)1174113838, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)1039754255L, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)47883, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)2.3378915E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.698219E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)3891010012L, PH.base.pack) ;
        p247_threat_level_SET((e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW), PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.892997E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)85, (uint8_t)98, (uint8_t)78, (uint8_t)195, (uint8_t)71, (uint8_t)15, (uint8_t)219, (uint8_t)230, (uint8_t)149, (uint8_t)116, (uint8_t)195, (uint8_t)64, (uint8_t)195, (uint8_t)72, (uint8_t)184, (uint8_t)179, (uint8_t)85, (uint8_t)249, (uint8_t)118, (uint8_t)164, (uint8_t)252, (uint8_t)160, (uint8_t)131, (uint8_t)223, (uint8_t)143, (uint8_t)72, (uint8_t)183, (uint8_t)75, (uint8_t)45, (uint8_t)223, (uint8_t)192, (uint8_t)87, (uint8_t)140, (uint8_t)103, (uint8_t)228, (uint8_t)240, (uint8_t)62, (uint8_t)206, (uint8_t)72, (uint8_t)60, (uint8_t)109, (uint8_t)13, (uint8_t)197, (uint8_t)38, (uint8_t)112, (uint8_t)143, (uint8_t)141, (uint8_t)101, (uint8_t)10, (uint8_t)178, (uint8_t)15, (uint8_t)162, (uint8_t)185, (uint8_t)137, (uint8_t)202, (uint8_t)125, (uint8_t)159, (uint8_t)23, (uint8_t)216, (uint8_t)77, (uint8_t)241, (uint8_t)190, (uint8_t)26, (uint8_t)2, (uint8_t)75, (uint8_t)122, (uint8_t)31, (uint8_t)104, (uint8_t)100, (uint8_t)76, (uint8_t)213, (uint8_t)136, (uint8_t)253, (uint8_t)36, (uint8_t)243, (uint8_t)157, (uint8_t)236, (uint8_t)43, (uint8_t)218, (uint8_t)81, (uint8_t)85, (uint8_t)155, (uint8_t)25, (uint8_t)155, (uint8_t)3, (uint8_t)107, (uint8_t)175, (uint8_t)19, (uint8_t)251, (uint8_t)67, (uint8_t)205, (uint8_t)252, (uint8_t)112, (uint8_t)29, (uint8_t)190, (uint8_t)182, (uint8_t)11, (uint8_t)83, (uint8_t)110, (uint8_t)144, (uint8_t)52, (uint8_t)253, (uint8_t)215, (uint8_t)50, (uint8_t)148, (uint8_t)13, (uint8_t)169, (uint8_t)12, (uint8_t)132, (uint8_t)194, (uint8_t)237, (uint8_t)235, (uint8_t)42, (uint8_t)190, (uint8_t)197, (uint8_t)10, (uint8_t)92, (uint8_t)109, (uint8_t)151, (uint8_t)12, (uint8_t)33, (uint8_t)133, (uint8_t)72, (uint8_t)167, (uint8_t)146, (uint8_t)12, (uint8_t)96, (uint8_t)137, (uint8_t)31, (uint8_t)168, (uint8_t)254, (uint8_t)91, (uint8_t)82, (uint8_t)77, (uint8_t)98, (uint8_t)197, (uint8_t)67, (uint8_t)126, (uint8_t)179, (uint8_t)77, (uint8_t)13, (uint8_t)120, (uint8_t)199, (uint8_t)195, (uint8_t)121, (uint8_t)50, (uint8_t)69, (uint8_t)79, (uint8_t)11, (uint8_t)112, (uint8_t)110, (uint8_t)216, (uint8_t)165, (uint8_t)248, (uint8_t)23, (uint8_t)219, (uint8_t)140, (uint8_t)88, (uint8_t)192, (uint8_t)14, (uint8_t)85, (uint8_t)197, (uint8_t)17, (uint8_t)205, (uint8_t)238, (uint8_t)188, (uint8_t)192, (uint8_t)99, (uint8_t)231, (uint8_t)158, (uint8_t)200, (uint8_t)104, (uint8_t)97, (uint8_t)125, (uint8_t)186, (uint8_t)99, (uint8_t)186, (uint8_t)145, (uint8_t)219, (uint8_t)27, (uint8_t)72, (uint8_t)51, (uint8_t)33, (uint8_t)188, (uint8_t)2, (uint8_t)244, (uint8_t)134, (uint8_t)186, (uint8_t)139, (uint8_t)158, (uint8_t)58, (uint8_t)65, (uint8_t)130, (uint8_t)52, (uint8_t)141, (uint8_t)236, (uint8_t)12, (uint8_t)227, (uint8_t)166, (uint8_t)34, (uint8_t)45, (uint8_t)240, (uint8_t)138, (uint8_t)201, (uint8_t)218, (uint8_t)147, (uint8_t)203, (uint8_t)118, (uint8_t)209, (uint8_t)87, (uint8_t)79, (uint8_t)121, (uint8_t)119, (uint8_t)72, (uint8_t)127, (uint8_t)83, (uint8_t)208, (uint8_t)157, (uint8_t)180, (uint8_t)22, (uint8_t)129, (uint8_t)107, (uint8_t)218, (uint8_t)12, (uint8_t)237, (uint8_t)170, (uint8_t)241, (uint8_t)6, (uint8_t)20, (uint8_t)239, (uint8_t)196, (uint8_t)77, (uint8_t)80, (uint8_t)199, (uint8_t)71, (uint8_t)95, (uint8_t)233, (uint8_t)155, (uint8_t)220, (uint8_t)129, (uint8_t)15, (uint8_t)163, (uint8_t)127, (uint8_t)110, (uint8_t)44, (uint8_t)77, (uint8_t)70, (uint8_t)126, (uint8_t)146};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)27798, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)113, (int8_t)122, (int8_t) -2, (int8_t)8, (int8_t) -115, (int8_t) -114, (int8_t) -28, (int8_t)12, (int8_t) -99, (int8_t) -127, (int8_t) -27, (int8_t) -92, (int8_t)59, (int8_t) -36, (int8_t)92, (int8_t) -24, (int8_t) -34, (int8_t)59, (int8_t)92, (int8_t) -105, (int8_t)118, (int8_t) -18, (int8_t)0, (int8_t) -126, (int8_t)7, (int8_t)104, (int8_t) -99, (int8_t) -57, (int8_t) -13, (int8_t) -15, (int8_t)40, (int8_t) -12};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)61589, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_z_SET((float) -7.546532E36F, PH.base.pack) ;
        {
            char16_t* name = u"nd";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)3282488120697872514L, PH.base.pack) ;
        p250_x_SET((float) -1.4127459E38F, PH.base.pack) ;
        p250_y_SET((float) -1.9085707E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float)5.5621735E37F, PH.base.pack) ;
        {
            char16_t* name = u"oglicu";
            p251_name_SET_(name, &PH) ;
        }
        p251_time_boot_ms_SET((uint32_t)2854253318L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t) -705439287, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)2871683760L, PH.base.pack) ;
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
            char16_t* text = u"polukxsadxv";
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
        p254_time_boot_ms_SET((uint32_t)322082211L, PH.base.pack) ;
        p254_value_SET((float)3.4438005E37F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)7560929290111555814L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)70, (uint8_t)184, (uint8_t)194, (uint8_t)100, (uint8_t)124, (uint8_t)86, (uint8_t)4, (uint8_t)181, (uint8_t)20, (uint8_t)88, (uint8_t)21, (uint8_t)202, (uint8_t)62, (uint8_t)115, (uint8_t)229, (uint8_t)247, (uint8_t)9, (uint8_t)235, (uint8_t)46, (uint8_t)15, (uint8_t)62, (uint8_t)122, (uint8_t)253, (uint8_t)39, (uint8_t)229, (uint8_t)15, (uint8_t)80, (uint8_t)79, (uint8_t)57, (uint8_t)232, (uint8_t)45, (uint8_t)53};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)429217244L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)1589180048L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        {
            char16_t* tune = u"tjucyymxjfaulapdaoqcjktsybk";
            p258_tune_SET_(tune, &PH) ;
        }
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_firmware_version_SET((uint32_t)1182861284L, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)29587, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)37572, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)35687, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE), PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)2735639571L, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -7.8216775E37F, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -2.9153854E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)24, (uint8_t)217, (uint8_t)116, (uint8_t)13, (uint8_t)111, (uint8_t)227, (uint8_t)117, (uint8_t)81, (uint8_t)131, (uint8_t)214, (uint8_t)122, (uint8_t)113, (uint8_t)145, (uint8_t)6, (uint8_t)206, (uint8_t)111, (uint8_t)239, (uint8_t)49, (uint8_t)186, (uint8_t)120, (uint8_t)221, (uint8_t)20, (uint8_t)144, (uint8_t)22, (uint8_t)222, (uint8_t)222, (uint8_t)255, (uint8_t)82, (uint8_t)124, (uint8_t)62, (uint8_t)14, (uint8_t)11};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)204, (uint8_t)172, (uint8_t)63, (uint8_t)180, (uint8_t)229, (uint8_t)119, (uint8_t)249, (uint8_t)128, (uint8_t)37, (uint8_t)2, (uint8_t)139, (uint8_t)77, (uint8_t)135, (uint8_t)152, (uint8_t)11, (uint8_t)209, (uint8_t)191, (uint8_t)169, (uint8_t)156, (uint8_t)141, (uint8_t)70, (uint8_t)69, (uint8_t)47, (uint8_t)215, (uint8_t)217, (uint8_t)150, (uint8_t)81, (uint8_t)198, (uint8_t)231, (uint8_t)221, (uint8_t)109, (uint8_t)210};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_focal_length_SET((float)1.1062894E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"qfpeebumbksZqsPrxvdnifkhmatobkevwydwarcavxaxdfvtcrteqrbUsleaTymjkzpjhnyduyByodnbt";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)1149194277L, PH.base.pack) ;
        p260_mode_id_SET((e_CAMERA_MODE_CAMERA_MODE_IMAGE), PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_status_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p261_read_speed_SET((float)1.2982745E38F, PH.base.pack) ;
        p261_write_speed_SET((float)2.634955E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.631183E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)1786781158L, PH.base.pack) ;
        p261_used_capacity_SET((float)6.0241033E37F, PH.base.pack) ;
        p261_available_capacity_SET((float)3.179876E38F, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_time_boot_ms_SET((uint32_t)3939513950L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p262_available_capacity_SET((float) -2.6755825E38F, PH.base.pack) ;
        p262_image_interval_SET((float)1.2733833E38F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)1009750268L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)6354560211876425049L, PH.base.pack) ;
        p263_lat_SET((int32_t)1223528164, PH.base.pack) ;
        p263_lon_SET((int32_t)38337045, PH.base.pack) ;
        p263_image_index_SET((int32_t)2021227512, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -896008055, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)71, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1698069613L, PH.base.pack) ;
        p263_alt_SET((int32_t)1002221480, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        {
            float q[] =  {-9.680622E37F, -1.0260593E38F, 1.7954011E38F, 1.0507582E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            char16_t* file_url = u"tmietjvszicbvyywsbgnljlskimpxnzkmyotpgtwupyWhQxozwnlqdxtdwjenwpkcatgkotajvnigqkyolad";
            p263_file_url_SET_(file_url, &PH) ;
        }
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)3376908904L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)7321033113673573919L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)5272936744596671832L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)3604298498276461216L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float) -3.3693192E37F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)724555417L, PH.base.pack) ;
        p265_roll_SET((float)3.1429431E38F, PH.base.pack) ;
        p265_pitch_SET((float) -2.2857483E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)184, (uint8_t)233, (uint8_t)57, (uint8_t)103, (uint8_t)138, (uint8_t)1, (uint8_t)151, (uint8_t)149, (uint8_t)50, (uint8_t)47, (uint8_t)139, (uint8_t)89, (uint8_t)58, (uint8_t)207, (uint8_t)46, (uint8_t)153, (uint8_t)63, (uint8_t)175, (uint8_t)151, (uint8_t)161, (uint8_t)93, (uint8_t)111, (uint8_t)7, (uint8_t)133, (uint8_t)202, (uint8_t)79, (uint8_t)99, (uint8_t)178, (uint8_t)47, (uint8_t)50, (uint8_t)32, (uint8_t)76, (uint8_t)198, (uint8_t)13, (uint8_t)199, (uint8_t)227, (uint8_t)87, (uint8_t)87, (uint8_t)82, (uint8_t)217, (uint8_t)142, (uint8_t)252, (uint8_t)246, (uint8_t)250, (uint8_t)201, (uint8_t)185, (uint8_t)110, (uint8_t)207, (uint8_t)89, (uint8_t)45, (uint8_t)16, (uint8_t)35, (uint8_t)59, (uint8_t)38, (uint8_t)99, (uint8_t)20, (uint8_t)25, (uint8_t)134, (uint8_t)218, (uint8_t)70, (uint8_t)124, (uint8_t)1, (uint8_t)166, (uint8_t)28, (uint8_t)12, (uint8_t)208, (uint8_t)48, (uint8_t)1, (uint8_t)197, (uint8_t)236, (uint8_t)203, (uint8_t)186, (uint8_t)139, (uint8_t)87, (uint8_t)40, (uint8_t)180, (uint8_t)191, (uint8_t)186, (uint8_t)153, (uint8_t)79, (uint8_t)197, (uint8_t)58, (uint8_t)220, (uint8_t)188, (uint8_t)103, (uint8_t)92, (uint8_t)81, (uint8_t)232, (uint8_t)184, (uint8_t)213, (uint8_t)151, (uint8_t)76, (uint8_t)38, (uint8_t)229, (uint8_t)87, (uint8_t)224, (uint8_t)23, (uint8_t)211, (uint8_t)161, (uint8_t)197, (uint8_t)156, (uint8_t)139, (uint8_t)240, (uint8_t)246, (uint8_t)115, (uint8_t)140, (uint8_t)173, (uint8_t)189, (uint8_t)93, (uint8_t)32, (uint8_t)34, (uint8_t)212, (uint8_t)186, (uint8_t)218, (uint8_t)224, (uint8_t)40, (uint8_t)51, (uint8_t)232, (uint8_t)236, (uint8_t)46, (uint8_t)138, (uint8_t)101, (uint8_t)146, (uint8_t)174, (uint8_t)150, (uint8_t)219, (uint8_t)180, (uint8_t)216, (uint8_t)67, (uint8_t)41, (uint8_t)192, (uint8_t)101, (uint8_t)74, (uint8_t)206, (uint8_t)84, (uint8_t)193, (uint8_t)142, (uint8_t)251, (uint8_t)120, (uint8_t)138, (uint8_t)81, (uint8_t)130, (uint8_t)177, (uint8_t)62, (uint8_t)220, (uint8_t)213, (uint8_t)215, (uint8_t)185, (uint8_t)16, (uint8_t)230, (uint8_t)200, (uint8_t)178, (uint8_t)38, (uint8_t)246, (uint8_t)200, (uint8_t)57, (uint8_t)75, (uint8_t)79, (uint8_t)117, (uint8_t)71, (uint8_t)227, (uint8_t)32, (uint8_t)123, (uint8_t)156, (uint8_t)94, (uint8_t)128, (uint8_t)194, (uint8_t)51, (uint8_t)181, (uint8_t)176, (uint8_t)142, (uint8_t)225, (uint8_t)227, (uint8_t)97, (uint8_t)110, (uint8_t)165, (uint8_t)151, (uint8_t)205, (uint8_t)103, (uint8_t)128, (uint8_t)218, (uint8_t)149, (uint8_t)222, (uint8_t)29, (uint8_t)42, (uint8_t)241, (uint8_t)174, (uint8_t)230, (uint8_t)153, (uint8_t)156, (uint8_t)164, (uint8_t)57, (uint8_t)227, (uint8_t)211, (uint8_t)120, (uint8_t)254, (uint8_t)174, (uint8_t)217, (uint8_t)66, (uint8_t)50, (uint8_t)111, (uint8_t)158, (uint8_t)242, (uint8_t)153, (uint8_t)114, (uint8_t)100, (uint8_t)216, (uint8_t)123, (uint8_t)88, (uint8_t)173, (uint8_t)195, (uint8_t)201, (uint8_t)86, (uint8_t)72, (uint8_t)131, (uint8_t)120, (uint8_t)221, (uint8_t)238, (uint8_t)200, (uint8_t)121, (uint8_t)239, (uint8_t)95, (uint8_t)23, (uint8_t)57, (uint8_t)222, (uint8_t)7, (uint8_t)220, (uint8_t)64, (uint8_t)63, (uint8_t)204, (uint8_t)131, (uint8_t)183, (uint8_t)101, (uint8_t)135, (uint8_t)196, (uint8_t)202, (uint8_t)127, (uint8_t)232, (uint8_t)68, (uint8_t)95, (uint8_t)182, (uint8_t)94, (uint8_t)195, (uint8_t)58, (uint8_t)40, (uint8_t)125, (uint8_t)228, (uint8_t)135, (uint8_t)220};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)18057, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)224, (uint8_t)119, (uint8_t)11, (uint8_t)165, (uint8_t)241, (uint8_t)83, (uint8_t)182, (uint8_t)167, (uint8_t)250, (uint8_t)7, (uint8_t)84, (uint8_t)161, (uint8_t)128, (uint8_t)216, (uint8_t)155, (uint8_t)189, (uint8_t)10, (uint8_t)253, (uint8_t)105, (uint8_t)63, (uint8_t)214, (uint8_t)224, (uint8_t)147, (uint8_t)84, (uint8_t)202, (uint8_t)218, (uint8_t)53, (uint8_t)85, (uint8_t)48, (uint8_t)106, (uint8_t)30, (uint8_t)198, (uint8_t)99, (uint8_t)194, (uint8_t)149, (uint8_t)239, (uint8_t)208, (uint8_t)220, (uint8_t)246, (uint8_t)254, (uint8_t)215, (uint8_t)251, (uint8_t)2, (uint8_t)127, (uint8_t)37, (uint8_t)11, (uint8_t)56, (uint8_t)13, (uint8_t)101, (uint8_t)72, (uint8_t)14, (uint8_t)226, (uint8_t)224, (uint8_t)81, (uint8_t)2, (uint8_t)166, (uint8_t)98, (uint8_t)42, (uint8_t)187, (uint8_t)10, (uint8_t)23, (uint8_t)72, (uint8_t)21, (uint8_t)28, (uint8_t)68, (uint8_t)28, (uint8_t)225, (uint8_t)7, (uint8_t)152, (uint8_t)84, (uint8_t)93, (uint8_t)107, (uint8_t)227, (uint8_t)248, (uint8_t)121, (uint8_t)117, (uint8_t)241, (uint8_t)145, (uint8_t)118, (uint8_t)201, (uint8_t)48, (uint8_t)211, (uint8_t)117, (uint8_t)220, (uint8_t)135, (uint8_t)124, (uint8_t)140, (uint8_t)110, (uint8_t)122, (uint8_t)101, (uint8_t)237, (uint8_t)44, (uint8_t)180, (uint8_t)78, (uint8_t)55, (uint8_t)4, (uint8_t)199, (uint8_t)13, (uint8_t)218, (uint8_t)170, (uint8_t)169, (uint8_t)240, (uint8_t)66, (uint8_t)205, (uint8_t)204, (uint8_t)116, (uint8_t)146, (uint8_t)31, (uint8_t)167, (uint8_t)3, (uint8_t)232, (uint8_t)121, (uint8_t)223, (uint8_t)108, (uint8_t)1, (uint8_t)137, (uint8_t)184, (uint8_t)26, (uint8_t)245, (uint8_t)52, (uint8_t)86, (uint8_t)147, (uint8_t)132, (uint8_t)228, (uint8_t)94, (uint8_t)218, (uint8_t)176, (uint8_t)235, (uint8_t)137, (uint8_t)54, (uint8_t)154, (uint8_t)212, (uint8_t)178, (uint8_t)163, (uint8_t)137, (uint8_t)102, (uint8_t)179, (uint8_t)104, (uint8_t)167, (uint8_t)54, (uint8_t)237, (uint8_t)74, (uint8_t)67, (uint8_t)174, (uint8_t)145, (uint8_t)173, (uint8_t)249, (uint8_t)79, (uint8_t)158, (uint8_t)228, (uint8_t)53, (uint8_t)97, (uint8_t)55, (uint8_t)229, (uint8_t)110, (uint8_t)64, (uint8_t)35, (uint8_t)199, (uint8_t)183, (uint8_t)54, (uint8_t)225, (uint8_t)115, (uint8_t)115, (uint8_t)254, (uint8_t)98, (uint8_t)122, (uint8_t)165, (uint8_t)66, (uint8_t)37, (uint8_t)217, (uint8_t)157, (uint8_t)180, (uint8_t)248, (uint8_t)149, (uint8_t)26, (uint8_t)54, (uint8_t)146, (uint8_t)21, (uint8_t)209, (uint8_t)101, (uint8_t)66, (uint8_t)47, (uint8_t)186, (uint8_t)187, (uint8_t)88, (uint8_t)59, (uint8_t)179, (uint8_t)28, (uint8_t)229, (uint8_t)117, (uint8_t)222, (uint8_t)100, (uint8_t)49, (uint8_t)89, (uint8_t)94, (uint8_t)37, (uint8_t)151, (uint8_t)14, (uint8_t)51, (uint8_t)38, (uint8_t)83, (uint8_t)154, (uint8_t)71, (uint8_t)138, (uint8_t)245, (uint8_t)26, (uint8_t)53, (uint8_t)209, (uint8_t)45, (uint8_t)241, (uint8_t)74, (uint8_t)204, (uint8_t)5, (uint8_t)211, (uint8_t)239, (uint8_t)120, (uint8_t)53, (uint8_t)31, (uint8_t)104, (uint8_t)166, (uint8_t)114, (uint8_t)231, (uint8_t)194, (uint8_t)2, (uint8_t)0, (uint8_t)143, (uint8_t)186, (uint8_t)158, (uint8_t)194, (uint8_t)230, (uint8_t)220, (uint8_t)254, (uint8_t)16, (uint8_t)46, (uint8_t)60, (uint8_t)83, (uint8_t)8, (uint8_t)227, (uint8_t)253, (uint8_t)145, (uint8_t)123, (uint8_t)238, (uint8_t)187, (uint8_t)166, (uint8_t)104, (uint8_t)68, (uint8_t)94, (uint8_t)145, (uint8_t)196};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)2573, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)55826, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_bitrate_SET((uint32_t)3112803489L, PH.base.pack) ;
        {
            char16_t* uri = u"iwtlpyiqsdoflRicvtotxpofhikmpubpfkegpepzbtCfhpgubhnliaytjnMbkcnncdrJq";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_h_SET((uint16_t)(uint16_t)34337, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)60307, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)4994, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p269_framerate_SET((float)1.0813347E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)18637, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)50706, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)41664, PH.base.pack) ;
        p270_framerate_SET((float) -2.5235058E38F, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2775779314L, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        {
            char16_t* uri = u"sfwixrrZebQoSIvxcoqEeenyigqiixhthNbhdztpepBLkoigwfogleosqdbrifgitnfdiQzlaxliwmxWkndifhtxymernVhqbikpudtyggsribwwsraodhwblDspzmoCohmeknquaFkubjhucvqadoupbvUihdkHyXduAgngyosZgSdaiqretyytbYhgmhbkcrakruyyjinuahwBdijxamkvvpevjs";
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
            char16_t* ssid = u"szcgzXNtrchdjrlnejqbqa";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"jcogjxiCttlxnrfdkiYyhsnoafntidrk";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        {
            uint8_t library_version_hash[] =  {(uint8_t)88, (uint8_t)122, (uint8_t)232, (uint8_t)248, (uint8_t)83, (uint8_t)101, (uint8_t)233, (uint8_t)0};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t spec_version_hash[] =  {(uint8_t)98, (uint8_t)186, (uint8_t)177, (uint8_t)248, (uint8_t)141, (uint8_t)190, (uint8_t)127, (uint8_t)249};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)21003, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)35253, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)55416, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)58257, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1829974794L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)4790528632306991987L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            uint8_t hw_unique_id[] =  {(uint8_t)255, (uint8_t)100, (uint8_t)136, (uint8_t)89, (uint8_t)245, (uint8_t)195, (uint8_t)119, (uint8_t)41, (uint8_t)50, (uint8_t)226, (uint8_t)154, (uint8_t)18, (uint8_t)129, (uint8_t)238, (uint8_t)78, (uint8_t)225};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_time_usec_SET((uint64_t)2976849736391526273L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)1079477178L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)4130534608L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        {
            char16_t* name = u"fjFFFoxhreublhhCgviKwaifazicehlrrEevocnctqdughvfmkEvjfDohIijanbclcZhvkwzytlP";
            p311_name_SET_(name, &PH) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t) -2707, PH.base.pack) ;
        {
            char16_t* param_id = u"is";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_count_SET((uint16_t)(uint16_t)54931, PH.base.pack) ;
        {
            char16_t* param_id = u"erjewpbypevjdNp";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)45008, PH.base.pack) ;
        {
            char16_t* param_value = u"yvzdelvncvoybqvaxyqMswfykqgPNwzhSHxgezUbiiUaphnynfzrbrabludfdqxotfwgvnuRefpsfl";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            char16_t* param_value = u"qkylnyxzylzuQJxptqkvygjcoizjJc";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        {
            char16_t* param_id = u"i";
            p323_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"qpvoLkQevpbrdq";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
        {
            char16_t* param_value = u"yyckhceagszuqXkyfhjuaxwcoeJcrbqnbyXnxfentbaghBktipvpftysiigvqhxsRdotyfrqnmmiThigspcrcelxLgktipwJyiwdeksdcvPvbUNuhtubwszir";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_increment_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)5045172981299597924L, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)52614, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)38686, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)40688, (uint16_t)8771, (uint16_t)27197, (uint16_t)29368, (uint16_t)10304, (uint16_t)2747, (uint16_t)33826, (uint16_t)38649, (uint16_t)13239, (uint16_t)58392, (uint16_t)58742, (uint16_t)21004, (uint16_t)51278, (uint16_t)44517, (uint16_t)60677, (uint16_t)6153, (uint16_t)9095, (uint16_t)45364, (uint16_t)20957, (uint16_t)22814, (uint16_t)21335, (uint16_t)17102, (uint16_t)36962, (uint16_t)27918, (uint16_t)26440, (uint16_t)26678, (uint16_t)36417, (uint16_t)26738, (uint16_t)56066, (uint16_t)57148, (uint16_t)16029, (uint16_t)36844, (uint16_t)3664, (uint16_t)35953, (uint16_t)60420, (uint16_t)17093, (uint16_t)52179, (uint16_t)44681, (uint16_t)55402, (uint16_t)35930, (uint16_t)5750, (uint16_t)57283, (uint16_t)28035, (uint16_t)13275, (uint16_t)23120, (uint16_t)22745, (uint16_t)18863, (uint16_t)62778, (uint16_t)52659, (uint16_t)3871, (uint16_t)3996, (uint16_t)38198, (uint16_t)34833, (uint16_t)1214, (uint16_t)23753, (uint16_t)7457, (uint16_t)8114, (uint16_t)5829, (uint16_t)59042, (uint16_t)21006, (uint16_t)56260, (uint16_t)10638, (uint16_t)52725, (uint16_t)29414, (uint16_t)42113, (uint16_t)65123, (uint16_t)11936, (uint16_t)63549, (uint16_t)19640, (uint16_t)21190, (uint16_t)40550, (uint16_t)61212};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


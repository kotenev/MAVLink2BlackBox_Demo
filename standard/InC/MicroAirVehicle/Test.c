
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
    return  _en__J(get_bits(data, 276, 7));
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
    return  _en__J(get_bits(data, 276, 7));
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
INLINER void p139_time_usec_SET(uint64_t  src, Pack * dst)//Timestamp (micros since boot or Unix epoch)
{
    uint8_t * data = dst->data;
    set_bytes((src), 8, data,  0);
}
/**
*Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
*	 this field to difference between instances*/
INLINER void p139_group_mlx_SET(uint8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  8);
}
INLINER void p139_target_system_SET(uint8_t  src, Pack * dst)//System ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  9);
}
INLINER void p139_target_component_SET(uint8_t  src, Pack * dst)//Component ID
{
    uint8_t * data = dst->data;
    set_bytes((src), 1, data,  10);
}
/**
*Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
*	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
*	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
*	 mixer to repurpose them as generic outputs*/
INLINER void p139_controls_SET(float*  src, int32_t pos, Pack * dst)
{
    uint8_t * data = dst->data;
    for(int32_t BYTE =  11, src_max = pos + 8; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}
Pack * c_TEST_Channel_new_SET_ACTUATOR_CONTROL_TARGET_139()
{
    return  zero2empty(c_CommunicationChannel.process(NULL, 139));
};
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
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED));
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_POWEROFF);
    assert(p0_custom_mode_GET(pack) == (uint32_t)1060501840L);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING));
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)1857);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)27432);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)33591);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)10059);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)105);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -13001);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)40903);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL));
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)11524);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)32220);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)8325);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)7634622423534182884L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1105336511L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_afz_GET(pack) == (float)9.214037E37F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)22294);
    assert(p3_afx_GET(pack) == (float)8.729704E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1074675970L);
    assert(p3_x_GET(pack) == (float) -2.9445163E38F);
    assert(p3_yaw_GET(pack) == (float) -1.7793626E38F);
    assert(p3_vz_GET(pack) == (float)3.3353678E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -1.0090224E38F);
    assert(p3_y_GET(pack) == (float)1.9573801E38F);
    assert(p3_afy_GET(pack) == (float) -2.8402219E38F);
    assert(p3_z_GET(pack) == (float) -1.8385365E38F);
    assert(p3_vx_GET(pack) == (float) -3.068526E38F);
    assert(p3_vy_GET(pack) == (float)1.3642794E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p4_time_usec_GET(pack) == (uint64_t)7694752356110336154L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p4_seq_GET(pack) == (uint32_t)219616255L);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p5_passkey_LEN(ph) == 24);
    {
        char16_t * exemplary = u"unsSvpetKkoNCwoqskxfbuvb";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)158);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 16);
    {
        char16_t * exemplary = u"yrasWkmvgysvonvz";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)249607035L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"Fssxj";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)15820);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)219);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_value_GET(pack) == (float) -7.9419654E37F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)8502);
    assert(p22_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"zvxoroblihnbzntd";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)15042);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float)1.1738127E38F);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p23_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"rlffz";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)19212);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1049345780);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)3780895093L);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)22006);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)2179923220L);
    assert(p24_lon_GET(pack) == (int32_t) -323317869);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p24_alt_GET(pack) == (int32_t)515059622);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1189866364L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)54292);
    assert(p24_lat_GET(pack) == (int32_t) -892740890);
    assert(p24_time_usec_GET(pack) == (uint64_t)8283869478514346971L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)30746);
    assert(p24_h_acc_TRY(ph) == (uint32_t)1608262074L);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)69);
    {
        uint8_t exemplary[] =  {(uint8_t)96, (uint8_t)241, (uint8_t)155, (uint8_t)231, (uint8_t)39, (uint8_t)188, (uint8_t)69, (uint8_t)58, (uint8_t)12, (uint8_t)159, (uint8_t)183, (uint8_t)119, (uint8_t)80, (uint8_t)84, (uint8_t)133, (uint8_t)59, (uint8_t)210, (uint8_t)47, (uint8_t)225, (uint8_t)146} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)50, (uint8_t)9, (uint8_t)106, (uint8_t)69, (uint8_t)164, (uint8_t)226, (uint8_t)59, (uint8_t)192, (uint8_t)221, (uint8_t)165, (uint8_t)191, (uint8_t)177, (uint8_t)173, (uint8_t)112, (uint8_t)14, (uint8_t)178, (uint8_t)239, (uint8_t)41, (uint8_t)73, (uint8_t)204} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)113, (uint8_t)47, (uint8_t)234, (uint8_t)129, (uint8_t)45, (uint8_t)189, (uint8_t)138, (uint8_t)252, (uint8_t)240, (uint8_t)149, (uint8_t)79, (uint8_t)159, (uint8_t)56, (uint8_t)68, (uint8_t)250, (uint8_t)239, (uint8_t)123, (uint8_t)148, (uint8_t)88, (uint8_t)230} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)103, (uint8_t)211, (uint8_t)75, (uint8_t)182, (uint8_t)67, (uint8_t)109, (uint8_t)130, (uint8_t)110, (uint8_t)185, (uint8_t)27, (uint8_t)29, (uint8_t)56, (uint8_t)241, (uint8_t)189, (uint8_t)214, (uint8_t)227, (uint8_t)87, (uint8_t)193, (uint8_t)156, (uint8_t)186} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)129, (uint8_t)47, (uint8_t)10, (uint8_t)40, (uint8_t)76, (uint8_t)31, (uint8_t)49, (uint8_t)126, (uint8_t)105, (uint8_t)118, (uint8_t)255, (uint8_t)49, (uint8_t)143, (uint8_t)181, (uint8_t)180, (uint8_t)134, (uint8_t)112, (uint8_t)118, (uint8_t)10} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)12711);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)11921);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)2032376289L);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -4552);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -8344);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)22378);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -31342);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)11430);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -19170);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -11756);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -8216);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)30209);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -12857);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)5434);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)18836);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)14994);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)4416);
    assert(p27_time_usec_GET(pack) == (uint64_t)6361320954881913915L);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)14861);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -19606);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)6384161350934797425L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -20992);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)22159);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)4889);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)1097);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -14326);
    assert(p29_press_abs_GET(pack) == (float) -1.5992514E38F);
    assert(p29_press_diff_GET(pack) == (float) -1.4760601E37F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1939714557L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_roll_GET(pack) == (float) -5.9697794E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3148874044L);
    assert(p30_pitchspeed_GET(pack) == (float)4.450594E37F);
    assert(p30_rollspeed_GET(pack) == (float)2.3539909E38F);
    assert(p30_yaw_GET(pack) == (float) -6.862133E36F);
    assert(p30_pitch_GET(pack) == (float) -2.0181127E38F);
    assert(p30_yawspeed_GET(pack) == (float)2.4052636E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float) -2.4151715E37F);
    assert(p31_q1_GET(pack) == (float)1.2429265E38F);
    assert(p31_yawspeed_GET(pack) == (float) -8.991098E37F);
    assert(p31_rollspeed_GET(pack) == (float)2.5469498E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1726364953L);
    assert(p31_q2_GET(pack) == (float) -1.9803148E38F);
    assert(p31_pitchspeed_GET(pack) == (float)8.0957517E36F);
    assert(p31_q4_GET(pack) == (float)2.5400593E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_y_GET(pack) == (float) -7.6629773E37F);
    assert(p32_vz_GET(pack) == (float)2.7021467E38F);
    assert(p32_vx_GET(pack) == (float) -1.8877108E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1490719189L);
    assert(p32_x_GET(pack) == (float)1.606349E38F);
    assert(p32_vy_GET(pack) == (float)2.0215652E38F);
    assert(p32_z_GET(pack) == (float) -2.1291113E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)55671);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)5634);
    assert(p33_lon_GET(pack) == (int32_t)1536839455);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -32615);
    assert(p33_alt_GET(pack) == (int32_t)963642354);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1679850299L);
    assert(p33_relative_alt_GET(pack) == (int32_t) -1260420412);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -13740);
    assert(p33_lat_GET(pack) == (int32_t)1125743699);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -15855);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)23161);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -4473);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -13852);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)75113065L);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)14103);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)17247);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)30199);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -27963);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)64603);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)50601);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)60141);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)32706);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)28564);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)3889897813L);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)27119);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)34443);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)63074);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)11094);
    assert(p36_time_usec_GET(pack) == (uint32_t)3905139285L);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)37897);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)54740);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)45342);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)4824);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)21393);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)10501);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)44182);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)36039);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)49598);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)35025);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)24880);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)57421);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)63643);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)23341);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)36822);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -8621);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -28643);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)190);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)13894);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)30602);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_STORAGE_FORMAT);
    assert(p39_z_GET(pack) == (float) -1.2325662E38F);
    assert(p39_x_GET(pack) == (float)1.169651E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p39_param2_GET(pack) == (float) -5.1676233E36F);
    assert(p39_param1_GET(pack) == (float) -1.2874269E38F);
    assert(p39_param4_GET(pack) == (float)2.9200435E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p39_y_GET(pack) == (float)1.0887765E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p39_param3_GET(pack) == (float) -6.1445824E37F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)12607);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)12883);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)28654);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)9249);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)13073);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)151);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)67);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)40595);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_longitude_GET(pack) == (int32_t)1942755452);
    assert(p48_time_usec_TRY(ph) == (uint64_t)2297995389226948791L);
    assert(p48_altitude_GET(pack) == (int32_t) -1897744154);
    assert(p48_latitude_GET(pack) == (int32_t)1880154801);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)103);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_longitude_GET(pack) == (int32_t)1654509081);
    assert(p49_altitude_GET(pack) == (int32_t)1962497384);
    assert(p49_latitude_GET(pack) == (int32_t) -2136336510);
    assert(p49_time_usec_TRY(ph) == (uint64_t)4614774793136427420L);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_scale_GET(pack) == (float)2.6056047E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p50_param_value0_GET(pack) == (float)2.1272854E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)9827);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p50_param_value_min_GET(pack) == (float) -1.5213065E38F);
    assert(p50_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"yxdxYfoxjpgt";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float) -2.294345E37F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)57906);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p54_p2x_GET(pack) == (float)7.4545096E37F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p54_p2z_GET(pack) == (float) -3.173437E38F);
    assert(p54_p1x_GET(pack) == (float)2.7934573E38F);
    assert(p54_p1z_GET(pack) == (float) -2.6240566E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p54_p1y_GET(pack) == (float)1.0995837E38F);
    assert(p54_p2y_GET(pack) == (float) -2.6984233E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)1.7487183E38F);
    assert(p55_p1y_GET(pack) == (float) -1.3722208E38F);
    assert(p55_p2x_GET(pack) == (float) -8.367556E37F);
    assert(p55_p2y_GET(pack) == (float)3.036551E37F);
    assert(p55_p1x_GET(pack) == (float)1.2510541E38F);
    assert(p55_p1z_GET(pack) == (float) -2.8225478E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float)1.7565254E38F);
    assert(p61_rollspeed_GET(pack) == (float) -3.3065735E37F);
    {
        float exemplary[] =  {3.2924492E38F, -2.372499E38F, -3.0619506E38F, -1.7453472E38F, 1.8144025E38F, 3.1344575E36F, 2.2472502E38F, -2.9819428E38F, 1.3755758E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float)2.0389326E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)7935005055702342539L);
    {
        float exemplary[] =  {-2.1568913E38F, 1.4918001E38F, 2.3371337E38F, 3.3887769E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)858);
    assert(p62_xtrack_error_GET(pack) == (float) -2.086882E38F);
    assert(p62_nav_pitch_GET(pack) == (float)2.8886658E38F);
    assert(p62_aspd_error_GET(pack) == (float)2.7348182E38F);
    assert(p62_nav_roll_GET(pack) == (float)1.3077961E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)29735);
    assert(p62_alt_error_GET(pack) == (float) -2.8065545E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)29523);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vz_GET(pack) == (float)1.6707489E36F);
    {
        float exemplary[] =  {-9.47555E37F, -7.827303E37F, 2.899872E38F, -3.0329425E38F, -2.3923755E38F, -1.7897152E38F, -2.8081144E38F, 1.2352173E38F, -1.0865552E38F, 3.3407814E38F, -3.1845638E38F, 1.2975235E38F, -3.1763426E38F, -1.1650083E38F, -3.1442382E38F, 1.8813612E38F, -2.2833734E38F, 3.7725054E37F, 9.340371E37F, -1.0670057E38F, -2.3884799E38F, -3.2039092E38F, -2.480092E38F, -1.6912496E38F, -1.7302471E37F, 2.9905823E38F, 1.1005789E38F, -3.3061685E38F, -2.2753556E38F, -2.6101327E38F, 2.7677346E38F, 1.3077347E38F, 1.4976541E38F, -1.3547927E38F, 2.6614951E38F, -2.3651273E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vy_GET(pack) == (float)2.0571779E38F);
    assert(p63_relative_alt_GET(pack) == (int32_t)125838608);
    assert(p63_time_usec_GET(pack) == (uint64_t)6185445466815249261L);
    assert(p63_vx_GET(pack) == (float) -7.0632046E37F);
    assert(p63_lon_GET(pack) == (int32_t)168131907);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p63_alt_GET(pack) == (int32_t)1451700335);
    assert(p63_lat_GET(pack) == (int32_t) -1350483138);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_ay_GET(pack) == (float) -2.421818E38F);
    assert(p64_z_GET(pack) == (float)1.2196505E38F);
    assert(p64_az_GET(pack) == (float) -2.7940283E38F);
    assert(p64_vx_GET(pack) == (float) -1.0162065E38F);
    {
        float exemplary[] =  {-2.0377043E38F, -2.071029E38F, 3.0549292E38F, 2.6636452E38F, -1.8794192E38F, 2.8698894E38F, 1.0656874E36F, 3.0693095E38F, 1.298982E38F, -1.5649746E38F, -2.8529138E38F, 9.932878E36F, 2.9926576E37F, 2.0676016E38F, -2.1946105E38F, -2.6913827E38F, 8.669231E37F, -1.0735569E38F, -1.3451566E38F, -2.5044446E38F, 1.3240429E38F, 1.1653121E38F, -1.4157384E38F, 2.882579E38F, 1.4770544E38F, 1.8289164E38F, 1.8928488E38F, -2.660423E38F, -4.979486E37F, 1.0063902E38F, 2.7669657E38F, 5.0793715E37F, 1.896898E38F, -2.5899305E38F, 1.9656882E38F, 3.6805852E37F, 9.101184E37F, 2.7958472E38F, 2.6063777E38F, 7.551962E37F, -2.6833013E38F, -1.3314866E38F, 1.0726121E38F, -2.903881E38F, 7.7668557E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_x_GET(pack) == (float)1.3050805E38F);
    assert(p64_vz_GET(pack) == (float)1.6880419E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_vy_GET(pack) == (float) -2.3622876E38F);
    assert(p64_y_GET(pack) == (float)2.3888334E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)2294485730366572808L);
    assert(p64_ax_GET(pack) == (float) -1.2626822E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)63884);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)34161);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)5632);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)17653);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)33041);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)51841);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)1559);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)10412);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)34750);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)17732);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)10682);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)52473);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)37565);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)983290654L);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)52030);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)17230);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)39606);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)27235);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)60811);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)25617);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)114);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)25103);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)121);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t)2939);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -21068);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -7987);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)19348);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)62405);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)8707);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)53706);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)35545);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)52556);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)65515);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)2560);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)29773);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)36856);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)46032);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL);
    assert(p73_param2_GET(pack) == (float) -2.412339E38F);
    assert(p73_param1_GET(pack) == (float)1.9862022E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p73_param3_GET(pack) == (float)1.6439301E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p73_param4_GET(pack) == (float) -2.3288741E38F);
    assert(p73_z_GET(pack) == (float) -2.5875047E38F);
    assert(p73_x_GET(pack) == (int32_t) -1721357994);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p73_y_GET(pack) == (int32_t) -666837606);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float) -3.2368858E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)2897);
    assert(p74_airspeed_GET(pack) == (float) -1.5876115E38F);
    assert(p74_alt_GET(pack) == (float)3.1604836E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)23529);
    assert(p74_climb_GET(pack) == (float) -1.6707182E37F);
};


void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param4_GET(pack) == (float)1.1004004E38F);
    assert(p75_param2_GET(pack) == (float) -6.614056E36F);
    assert(p75_param1_GET(pack) == (float) -3.0052724E38F);
    assert(p75_y_GET(pack) == (int32_t)1702662376);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p75_param3_GET(pack) == (float)2.591768E37F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p75_z_GET(pack) == (float) -1.2996527E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p75_x_GET(pack) == (int32_t) -1085923689);
};


void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param5_GET(pack) == (float)1.6550932E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
    assert(p76_param3_GET(pack) == (float) -1.947417E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p76_param2_GET(pack) == (float)2.6724991E38F);
    assert(p76_param6_GET(pack) == (float)2.7348268E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p76_param7_GET(pack) == (float)1.83045E38F);
    assert(p76_param4_GET(pack) == (float) -9.478394E37F);
    assert(p76_param1_GET(pack) == (float) -2.6371179E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)220);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)142);
    assert(p77_result_param2_TRY(ph) == (int32_t) -1163543969);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)145);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_pitch_GET(pack) == (float) -2.174981E38F);
    assert(p81_yaw_GET(pack) == (float) -2.4052832E36F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)450653911L);
    assert(p81_thrust_GET(pack) == (float) -3.1473649E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p81_roll_GET(pack) == (float) -2.050227E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)87);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.6124421E38F);
    assert(p82_thrust_GET(pack) == (float) -2.03571E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)142);
    {
        float exemplary[] =  {-1.3717455E38F, 2.3196087E38F, -1.800006E38F, -8.394787E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_yaw_rate_GET(pack) == (float)2.0367486E38F);
    assert(p82_body_roll_rate_GET(pack) == (float) -1.6304781E38F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)881908173L);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3731750320L);
    assert(p83_body_pitch_rate_GET(pack) == (float)1.9702304E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)1.0140239E38F);
    {
        float exemplary[] =  {2.7537884E38F, 1.1154149E38F, -1.7469682E38F, 1.549366E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_thrust_GET(pack) == (float) -1.7353663E37F);
    assert(p83_body_yaw_rate_GET(pack) == (float)3.1208087E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_yaw_rate_GET(pack) == (float) -1.6679453E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)292776474L);
    assert(p84_afz_GET(pack) == (float)9.497912E37F);
    assert(p84_y_GET(pack) == (float) -2.1754468E38F);
    assert(p84_x_GET(pack) == (float) -3.3474953E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p84_z_GET(pack) == (float) -1.7374338E38F);
    assert(p84_vz_GET(pack) == (float) -9.860154E37F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p84_vy_GET(pack) == (float)9.152354E37F);
    assert(p84_afx_GET(pack) == (float)2.2206561E38F);
    assert(p84_afy_GET(pack) == (float)2.6483706E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)62633);
    assert(p84_yaw_GET(pack) == (float) -5.50723E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p84_vx_GET(pack) == (float)1.6197272E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_lon_int_GET(pack) == (int32_t)1930383390);
    assert(p86_vz_GET(pack) == (float)3.3718283E38F);
    assert(p86_vx_GET(pack) == (float) -2.2100614E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)61107920);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p86_yaw_rate_GET(pack) == (float) -1.9643246E37F);
    assert(p86_yaw_GET(pack) == (float) -3.399446E38F);
    assert(p86_vy_GET(pack) == (float) -5.708503E37F);
    assert(p86_afz_GET(pack) == (float) -3.5330718E37F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p86_afy_GET(pack) == (float) -1.9833387E38F);
    assert(p86_alt_GET(pack) == (float) -1.2333781E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)18124);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2659883812L);
    assert(p86_afx_GET(pack) == (float)1.6076738E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)215);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)19527);
    assert(p87_lat_int_GET(pack) == (int32_t) -2100545874);
    assert(p87_alt_GET(pack) == (float) -1.8839837E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2906881351L);
    assert(p87_yaw_rate_GET(pack) == (float) -4.518336E37F);
    assert(p87_vy_GET(pack) == (float) -1.6507752E38F);
    assert(p87_afx_GET(pack) == (float) -2.8697687E38F);
    assert(p87_afz_GET(pack) == (float) -1.388324E38F);
    assert(p87_vx_GET(pack) == (float)6.403594E37F);
    assert(p87_lon_int_GET(pack) == (int32_t) -880345366);
    assert(p87_yaw_GET(pack) == (float) -2.57991E37F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p87_afy_GET(pack) == (float) -2.272436E38F);
    assert(p87_vz_GET(pack) == (float) -1.7957315E38F);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float) -3.2715452E38F);
    assert(p89_z_GET(pack) == (float)1.7574874E38F);
    assert(p89_y_GET(pack) == (float)7.832998E37F);
    assert(p89_x_GET(pack) == (float) -2.7154958E38F);
    assert(p89_roll_GET(pack) == (float)5.030443E37F);
    assert(p89_pitch_GET(pack) == (float) -1.5732044E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2372448754L);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -27690);
    assert(p90_lon_GET(pack) == (int32_t) -529783883);
    assert(p90_rollspeed_GET(pack) == (float) -2.3859267E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)8772);
    assert(p90_pitchspeed_GET(pack) == (float)2.1119138E38F);
    assert(p90_yaw_GET(pack) == (float)2.7065188E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)23885);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)26587);
    assert(p90_roll_GET(pack) == (float) -2.3571089E38F);
    assert(p90_yawspeed_GET(pack) == (float)1.3272538E38F);
    assert(p90_alt_GET(pack) == (int32_t) -784558480);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)10214);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -25168);
    assert(p90_pitch_GET(pack) == (float)5.9599597E37F);
    assert(p90_time_usec_GET(pack) == (uint64_t)5756680667953247296L);
    assert(p90_lat_GET(pack) == (int32_t)1447341091);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_throttle_GET(pack) == (float) -1.2361726E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p91_aux3_GET(pack) == (float)3.2303009E38F);
    assert(p91_pitch_elevator_GET(pack) == (float) -1.9582243E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p91_time_usec_GET(pack) == (uint64_t)5414419863471487181L);
    assert(p91_aux1_GET(pack) == (float)2.4524768E38F);
    assert(p91_aux4_GET(pack) == (float)9.286032E37F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.1384934E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.4260721E38F);
    assert(p91_aux2_GET(pack) == (float)1.3445593E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)22742);
    assert(p92_time_usec_GET(pack) == (uint64_t)3064731715882247312L);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)19828);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)10163);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)50620);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)43048);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)63392);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)62594);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)23808);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)8945);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)3362);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)31413);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)23641);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.1401436E38F, 1.9270877E38F, -3.0882139E38F, 7.986186E37F, -1.47155E38F, -5.7408975E37F, -2.6365013E38F, 3.3117346E38F, 5.3224065E37F, 5.0036067E36F, -2.6785273E38F, 1.9791021E37F, -5.3181147E37F, -2.9705142E38F, -1.4168404E38F, 3.0350504E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_DISARMED);
    assert(p93_flags_GET(pack) == (uint64_t)2598394582242494798L);
    assert(p93_time_usec_GET(pack) == (uint64_t)7259605327592818091L);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_time_usec_GET(pack) == (uint64_t)1355266182916426271L);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -8.769822E37F);
    assert(p100_flow_rate_y_TRY(ph) == (float)1.3554497E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p100_ground_distance_GET(pack) == (float)2.2481607E38F);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -1.3724552E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.2600626E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -30041);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)9642);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_x_GET(pack) == (float) -2.4737396E38F);
    assert(p101_pitch_GET(pack) == (float)2.6341733E38F);
    assert(p101_roll_GET(pack) == (float)1.6704782E38F);
    assert(p101_z_GET(pack) == (float) -3.3105097E38F);
    assert(p101_usec_GET(pack) == (uint64_t)9074240654075090863L);
    assert(p101_yaw_GET(pack) == (float)3.0502856E38F);
    assert(p101_y_GET(pack) == (float)2.5222981E38F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float)1.5985928E37F);
    assert(p102_pitch_GET(pack) == (float) -1.7474474E38F);
    assert(p102_yaw_GET(pack) == (float)1.3625497E37F);
    assert(p102_y_GET(pack) == (float) -2.886323E38F);
    assert(p102_usec_GET(pack) == (uint64_t)3158962391191535772L);
    assert(p102_x_GET(pack) == (float)2.6751815E38F);
    assert(p102_roll_GET(pack) == (float)3.204445E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_y_GET(pack) == (float)3.2706298E38F);
    assert(p103_usec_GET(pack) == (uint64_t)176232725612833455L);
    assert(p103_z_GET(pack) == (float) -3.3275185E38F);
    assert(p103_x_GET(pack) == (float) -1.6305857E38F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float)1.0963697E38F);
    assert(p104_usec_GET(pack) == (uint64_t)8940800004365184444L);
    assert(p104_y_GET(pack) == (float) -2.1407958E38F);
    assert(p104_pitch_GET(pack) == (float)1.4328009E38F);
    assert(p104_z_GET(pack) == (float)2.091895E38F);
    assert(p104_roll_GET(pack) == (float)1.5303244E38F);
    assert(p104_yaw_GET(pack) == (float)1.5150606E38F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_yacc_GET(pack) == (float)3.0881003E38F);
    assert(p105_zacc_GET(pack) == (float) -1.6470903E38F);
    assert(p105_abs_pressure_GET(pack) == (float)1.1688609E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)1276669002784845366L);
    assert(p105_ymag_GET(pack) == (float)3.8880685E37F);
    assert(p105_xgyro_GET(pack) == (float)2.3537682E38F);
    assert(p105_temperature_GET(pack) == (float) -1.7549057E38F);
    assert(p105_zgyro_GET(pack) == (float)1.6982933E38F);
    assert(p105_pressure_alt_GET(pack) == (float)2.4878684E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)44005);
    assert(p105_ygyro_GET(pack) == (float) -2.247365E38F);
    assert(p105_xacc_GET(pack) == (float)1.51999E38F);
    assert(p105_diff_pressure_GET(pack) == (float) -2.2073342E38F);
    assert(p105_zmag_GET(pack) == (float) -1.559563E38F);
    assert(p105_xmag_GET(pack) == (float) -2.5997867E38F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1452688228L);
    assert(p106_integrated_ygyro_GET(pack) == (float) -1.7499715E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p106_integrated_zgyro_GET(pack) == (float)2.8053854E38F);
    assert(p106_integrated_y_GET(pack) == (float)6.6494526E37F);
    assert(p106_time_usec_GET(pack) == (uint64_t)9156898140728557126L);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p106_distance_GET(pack) == (float)2.1020458E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)2264475641L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -21101);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.393106E38F);
    assert(p106_integrated_x_GET(pack) == (float)1.6443057E38F);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_yacc_GET(pack) == (float) -2.0126888E38F);
    assert(p107_zmag_GET(pack) == (float) -2.2100229E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -2.0265759E37F);
    assert(p107_temperature_GET(pack) == (float) -9.36641E37F);
    assert(p107_ygyro_GET(pack) == (float) -2.0316864E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2770239675L);
    assert(p107_pressure_alt_GET(pack) == (float)1.1886964E38F);
    assert(p107_xgyro_GET(pack) == (float) -2.3846102E38F);
    assert(p107_xacc_GET(pack) == (float) -1.5529292E38F);
    assert(p107_ymag_GET(pack) == (float)3.2262726E38F);
    assert(p107_abs_pressure_GET(pack) == (float) -1.222173E38F);
    assert(p107_xmag_GET(pack) == (float)2.1292374E38F);
    assert(p107_zgyro_GET(pack) == (float)3.6043954E37F);
    assert(p107_time_usec_GET(pack) == (uint64_t)264837455914001734L);
    assert(p107_zacc_GET(pack) == (float)3.758573E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_vn_GET(pack) == (float) -2.8793454E38F);
    assert(p108_q3_GET(pack) == (float) -9.928618E37F);
    assert(p108_q2_GET(pack) == (float)2.8743123E38F);
    assert(p108_q1_GET(pack) == (float)7.8641276E37F);
    assert(p108_roll_GET(pack) == (float)3.1525858E38F);
    assert(p108_ve_GET(pack) == (float)1.705164E38F);
    assert(p108_lat_GET(pack) == (float)1.5458847E38F);
    assert(p108_vd_GET(pack) == (float)1.2222868E38F);
    assert(p108_xgyro_GET(pack) == (float)4.555993E37F);
    assert(p108_q4_GET(pack) == (float)9.974749E37F);
    assert(p108_lon_GET(pack) == (float)1.4632474E38F);
    assert(p108_ygyro_GET(pack) == (float) -1.1840725E38F);
    assert(p108_alt_GET(pack) == (float)9.407917E37F);
    assert(p108_zgyro_GET(pack) == (float) -9.397873E37F);
    assert(p108_std_dev_horz_GET(pack) == (float) -1.364973E38F);
    assert(p108_pitch_GET(pack) == (float) -2.9987922E38F);
    assert(p108_zacc_GET(pack) == (float) -2.3460213E38F);
    assert(p108_yacc_GET(pack) == (float)1.812067E38F);
    assert(p108_xacc_GET(pack) == (float) -9.227157E37F);
    assert(p108_std_dev_vert_GET(pack) == (float) -3.01782E38F);
    assert(p108_yaw_GET(pack) == (float)7.089557E37F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)10288);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)61437);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)145, (uint8_t)95, (uint8_t)188, (uint8_t)53, (uint8_t)106, (uint8_t)173, (uint8_t)129, (uint8_t)239, (uint8_t)93, (uint8_t)203, (uint8_t)71, (uint8_t)59, (uint8_t)109, (uint8_t)152, (uint8_t)37, (uint8_t)117, (uint8_t)50, (uint8_t)222, (uint8_t)50, (uint8_t)158, (uint8_t)15, (uint8_t)236, (uint8_t)1, (uint8_t)103, (uint8_t)63, (uint8_t)122, (uint8_t)246, (uint8_t)219, (uint8_t)203, (uint8_t)105, (uint8_t)157, (uint8_t)11, (uint8_t)33, (uint8_t)229, (uint8_t)143, (uint8_t)85, (uint8_t)213, (uint8_t)28, (uint8_t)144, (uint8_t)76, (uint8_t)159, (uint8_t)21, (uint8_t)178, (uint8_t)46, (uint8_t)153, (uint8_t)105, (uint8_t)0, (uint8_t)195, (uint8_t)23, (uint8_t)84, (uint8_t)67, (uint8_t)116, (uint8_t)246, (uint8_t)78, (uint8_t)70, (uint8_t)239, (uint8_t)232, (uint8_t)8, (uint8_t)145, (uint8_t)87, (uint8_t)130, (uint8_t)167, (uint8_t)255, (uint8_t)76, (uint8_t)8, (uint8_t)29, (uint8_t)16, (uint8_t)177, (uint8_t)35, (uint8_t)236, (uint8_t)138, (uint8_t)112, (uint8_t)250, (uint8_t)253, (uint8_t)225, (uint8_t)178, (uint8_t)54, (uint8_t)194, (uint8_t)226, (uint8_t)207, (uint8_t)208, (uint8_t)103, (uint8_t)195, (uint8_t)203, (uint8_t)188, (uint8_t)202, (uint8_t)46, (uint8_t)98, (uint8_t)157, (uint8_t)50, (uint8_t)148, (uint8_t)214, (uint8_t)157, (uint8_t)170, (uint8_t)44, (uint8_t)243, (uint8_t)74, (uint8_t)115, (uint8_t)99, (uint8_t)148, (uint8_t)14, (uint8_t)207, (uint8_t)109, (uint8_t)116, (uint8_t)138, (uint8_t)236, (uint8_t)117, (uint8_t)204, (uint8_t)131, (uint8_t)77, (uint8_t)123, (uint8_t)175, (uint8_t)228, (uint8_t)5, (uint8_t)71, (uint8_t)122, (uint8_t)120, (uint8_t)88, (uint8_t)170, (uint8_t)198, (uint8_t)121, (uint8_t)19, (uint8_t)194, (uint8_t)130, (uint8_t)213, (uint8_t)28, (uint8_t)35, (uint8_t)122, (uint8_t)172, (uint8_t)148, (uint8_t)44, (uint8_t)143, (uint8_t)219, (uint8_t)192, (uint8_t)41, (uint8_t)82, (uint8_t)179, (uint8_t)94, (uint8_t)85, (uint8_t)193, (uint8_t)108, (uint8_t)98, (uint8_t)223, (uint8_t)0, (uint8_t)88, (uint8_t)11, (uint8_t)21, (uint8_t)46, (uint8_t)196, (uint8_t)13, (uint8_t)195, (uint8_t)201, (uint8_t)83, (uint8_t)235, (uint8_t)134, (uint8_t)254, (uint8_t)235, (uint8_t)211, (uint8_t)110, (uint8_t)213, (uint8_t)28, (uint8_t)94, (uint8_t)15, (uint8_t)32, (uint8_t)58, (uint8_t)38, (uint8_t)75, (uint8_t)14, (uint8_t)151, (uint8_t)174, (uint8_t)154, (uint8_t)190, (uint8_t)113, (uint8_t)237, (uint8_t)113, (uint8_t)205, (uint8_t)45, (uint8_t)72, (uint8_t)167, (uint8_t)20, (uint8_t)236, (uint8_t)78, (uint8_t)21, (uint8_t)34, (uint8_t)227, (uint8_t)151, (uint8_t)239, (uint8_t)180, (uint8_t)151, (uint8_t)238, (uint8_t)50, (uint8_t)202, (uint8_t)40, (uint8_t)98, (uint8_t)233, (uint8_t)17, (uint8_t)139, (uint8_t)141, (uint8_t)165, (uint8_t)202, (uint8_t)162, (uint8_t)204, (uint8_t)132, (uint8_t)124, (uint8_t)72, (uint8_t)39, (uint8_t)78, (uint8_t)158, (uint8_t)20, (uint8_t)227, (uint8_t)199, (uint8_t)86, (uint8_t)40, (uint8_t)90, (uint8_t)39, (uint8_t)85, (uint8_t)73, (uint8_t)183, (uint8_t)57, (uint8_t)57, (uint8_t)117, (uint8_t)162, (uint8_t)214, (uint8_t)249, (uint8_t)235, (uint8_t)245, (uint8_t)12, (uint8_t)29, (uint8_t)131, (uint8_t)123, (uint8_t)22, (uint8_t)158, (uint8_t)173, (uint8_t)166, (uint8_t)27, (uint8_t)162, (uint8_t)4, (uint8_t)197, (uint8_t)150, (uint8_t)25, (uint8_t)79, (uint8_t)176, (uint8_t)91, (uint8_t)91, (uint8_t)9, (uint8_t)180, (uint8_t)240, (uint8_t)86, (uint8_t)146, (uint8_t)3, (uint8_t)69} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t)3280321088926868591L);
    assert(p111_tc1_GET(pack) == (int64_t) -4762151346494955480L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)4955804822926667329L);
    assert(p112_seq_GET(pack) == (uint32_t)3380251666L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lat_GET(pack) == (int32_t) -111109987);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)65329);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p113_lon_GET(pack) == (int32_t)1526053565);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -27480);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)53945);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)12053);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -25072);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)12735);
    assert(p113_alt_GET(pack) == (int32_t) -1377047855);
    assert(p113_time_usec_GET(pack) == (uint64_t)5816670287078710709L);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)25057);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)47);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_usec_GET(pack) == (uint64_t)6706405504069426863L);
    assert(p114_integrated_y_GET(pack) == (float)2.6968279E38F);
    assert(p114_distance_GET(pack) == (float)1.3868266E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)9600);
    assert(p114_integrated_zgyro_GET(pack) == (float)3.257685E37F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.6760674E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float) -3.0507764E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p114_integrated_x_GET(pack) == (float)3.0666058E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3690541706L);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)717073274L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_pitchspeed_GET(pack) == (float)1.7492317E38F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)20137);
    assert(p115_time_usec_GET(pack) == (uint64_t)7409921347770046185L);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)59408);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -25396);
    assert(p115_lon_GET(pack) == (int32_t) -1790890013);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)21586);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)1023);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)15850);
    assert(p115_yawspeed_GET(pack) == (float) -1.2281251E38F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)26429);
    {
        float exemplary[] =  {-7.6124153E37F, -1.552223E38F, -2.6259518E38F, 2.9674731E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)16551);
    assert(p115_alt_GET(pack) == (int32_t)1170515319);
    assert(p115_lat_GET(pack) == (int32_t)1278509821);
    assert(p115_rollspeed_GET(pack) == (float)2.154888E38F);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)15634);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -21078);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)9903);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -30706);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -9056);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)1870412910L);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -21197);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -32265);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)27006);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -23421);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)17836);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)47350);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)20670);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)60668);
    assert(p118_time_utc_GET(pack) == (uint32_t)2895104652L);
    assert(p118_size_GET(pack) == (uint32_t)244851988L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)2257);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)37888);
    assert(p119_count_GET(pack) == (uint32_t)3621370107L);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p119_ofs_GET(pack) == (uint32_t)2600576083L);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)64);
    {
        uint8_t exemplary[] =  {(uint8_t)72, (uint8_t)236, (uint8_t)184, (uint8_t)145, (uint8_t)222, (uint8_t)173, (uint8_t)5, (uint8_t)243, (uint8_t)179, (uint8_t)114, (uint8_t)180, (uint8_t)3, (uint8_t)127, (uint8_t)123, (uint8_t)27, (uint8_t)186, (uint8_t)136, (uint8_t)124, (uint8_t)129, (uint8_t)62, (uint8_t)70, (uint8_t)250, (uint8_t)138, (uint8_t)254, (uint8_t)122, (uint8_t)208, (uint8_t)229, (uint8_t)48, (uint8_t)220, (uint8_t)68, (uint8_t)138, (uint8_t)226, (uint8_t)49, (uint8_t)172, (uint8_t)63, (uint8_t)172, (uint8_t)140, (uint8_t)158, (uint8_t)8, (uint8_t)154, (uint8_t)74, (uint8_t)185, (uint8_t)86, (uint8_t)110, (uint8_t)225, (uint8_t)176, (uint8_t)233, (uint8_t)43, (uint8_t)165, (uint8_t)60, (uint8_t)192, (uint8_t)77, (uint8_t)208, (uint8_t)48, (uint8_t)99, (uint8_t)11, (uint8_t)107, (uint8_t)105, (uint8_t)0, (uint8_t)17, (uint8_t)106, (uint8_t)81, (uint8_t)155, (uint8_t)138, (uint8_t)52, (uint8_t)90, (uint8_t)15, (uint8_t)109, (uint8_t)18, (uint8_t)71, (uint8_t)156, (uint8_t)84, (uint8_t)241, (uint8_t)38, (uint8_t)41, (uint8_t)13, (uint8_t)103, (uint8_t)58, (uint8_t)62, (uint8_t)173, (uint8_t)233, (uint8_t)118, (uint8_t)247, (uint8_t)98, (uint8_t)196, (uint8_t)162, (uint8_t)232, (uint8_t)165, (uint8_t)248, (uint8_t)171} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)1140444340L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)7531);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)89);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)207, (uint8_t)6, (uint8_t)18, (uint8_t)188, (uint8_t)170, (uint8_t)222, (uint8_t)213, (uint8_t)61, (uint8_t)11, (uint8_t)172, (uint8_t)120, (uint8_t)82, (uint8_t)209, (uint8_t)4, (uint8_t)125, (uint8_t)222, (uint8_t)210, (uint8_t)49, (uint8_t)136, (uint8_t)244, (uint8_t)169, (uint8_t)232, (uint8_t)8, (uint8_t)135, (uint8_t)164, (uint8_t)112, (uint8_t)5, (uint8_t)118, (uint8_t)12, (uint8_t)11, (uint8_t)60, (uint8_t)149, (uint8_t)150, (uint8_t)34, (uint8_t)248, (uint8_t)131, (uint8_t)188, (uint8_t)81, (uint8_t)36, (uint8_t)107, (uint8_t)0, (uint8_t)238, (uint8_t)205, (uint8_t)224, (uint8_t)116, (uint8_t)30, (uint8_t)7, (uint8_t)237, (uint8_t)120, (uint8_t)175, (uint8_t)93, (uint8_t)219, (uint8_t)227, (uint8_t)243, (uint8_t)207, (uint8_t)246, (uint8_t)159, (uint8_t)241, (uint8_t)201, (uint8_t)182, (uint8_t)202, (uint8_t)120, (uint8_t)93, (uint8_t)170, (uint8_t)182, (uint8_t)9, (uint8_t)154, (uint8_t)127, (uint8_t)5, (uint8_t)163, (uint8_t)236, (uint8_t)160, (uint8_t)164, (uint8_t)44, (uint8_t)16, (uint8_t)153, (uint8_t)237, (uint8_t)161, (uint8_t)67, (uint8_t)41, (uint8_t)190, (uint8_t)158, (uint8_t)44, (uint8_t)196, (uint8_t)160, (uint8_t)203, (uint8_t)137, (uint8_t)231, (uint8_t)69, (uint8_t)144, (uint8_t)74, (uint8_t)212, (uint8_t)46, (uint8_t)52, (uint8_t)227, (uint8_t)64, (uint8_t)214, (uint8_t)62, (uint8_t)31, (uint8_t)168, (uint8_t)141, (uint8_t)13, (uint8_t)23, (uint8_t)209, (uint8_t)79, (uint8_t)26, (uint8_t)30, (uint8_t)156, (uint8_t)142} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)216);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_dgps_age_GET(pack) == (uint32_t)1492970984L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)16916);
    assert(p124_lat_GET(pack) == (int32_t)1822807975);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED);
    assert(p124_time_usec_GET(pack) == (uint64_t)4131071916201263244L);
    assert(p124_alt_GET(pack) == (int32_t)1871906893);
    assert(p124_lon_GET(pack) == (int32_t) -2132123142);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)38499);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)38232);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)39860);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)201);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)35956);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED));
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)60155);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)3605374248L);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)86, (uint8_t)141, (uint8_t)172, (uint8_t)156, (uint8_t)122, (uint8_t)68, (uint8_t)225, (uint8_t)151, (uint8_t)102, (uint8_t)160, (uint8_t)185, (uint8_t)192, (uint8_t)159, (uint8_t)185, (uint8_t)233, (uint8_t)183, (uint8_t)43, (uint8_t)75, (uint8_t)95, (uint8_t)233, (uint8_t)101, (uint8_t)79, (uint8_t)45, (uint8_t)164, (uint8_t)52, (uint8_t)135, (uint8_t)169, (uint8_t)186, (uint8_t)82, (uint8_t)207, (uint8_t)250, (uint8_t)122, (uint8_t)1, (uint8_t)68, (uint8_t)48, (uint8_t)205, (uint8_t)141, (uint8_t)243, (uint8_t)247, (uint8_t)218, (uint8_t)122, (uint8_t)140, (uint8_t)169, (uint8_t)236, (uint8_t)107, (uint8_t)249, (uint8_t)207, (uint8_t)88, (uint8_t)118, (uint8_t)200, (uint8_t)95, (uint8_t)241, (uint8_t)202, (uint8_t)8, (uint8_t)2, (uint8_t)143, (uint8_t)200, (uint8_t)191, (uint8_t)220, (uint8_t)234, (uint8_t)35, (uint8_t)145, (uint8_t)243, (uint8_t)98, (uint8_t)140, (uint8_t)71, (uint8_t)50, (uint8_t)245, (uint8_t)105} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)27450);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE));
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -1232844035);
    assert(p127_tow_GET(pack) == (uint32_t)1290372543L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1221428579);
    assert(p127_accuracy_GET(pack) == (uint32_t)2957776115L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)286814817);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)2050411559L);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)12878);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)210161443);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_tow_GET(pack) == (uint32_t)1202810911L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)354316918);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)158353155L);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)107695784);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -601496323);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -614350915);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)46598);
    assert(p128_accuracy_GET(pack) == (uint32_t)2197929479L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -8844);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)28897);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)31208);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)22595);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)17033);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)3403146973L);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -29553);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)11748);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -4164);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)10427);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_size_GET(pack) == (uint32_t)1858162012L);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)15324);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)37769);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)53860);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)137, (uint8_t)214, (uint8_t)216, (uint8_t)170, (uint8_t)198, (uint8_t)208, (uint8_t)34, (uint8_t)156, (uint8_t)131, (uint8_t)59, (uint8_t)23, (uint8_t)246, (uint8_t)250, (uint8_t)68, (uint8_t)213, (uint8_t)192, (uint8_t)80, (uint8_t)51, (uint8_t)47, (uint8_t)122, (uint8_t)47, (uint8_t)152, (uint8_t)74, (uint8_t)101, (uint8_t)60, (uint8_t)120, (uint8_t)235, (uint8_t)24, (uint8_t)105, (uint8_t)148, (uint8_t)117, (uint8_t)176, (uint8_t)210, (uint8_t)112, (uint8_t)166, (uint8_t)173, (uint8_t)176, (uint8_t)233, (uint8_t)30, (uint8_t)187, (uint8_t)53, (uint8_t)241, (uint8_t)95, (uint8_t)33, (uint8_t)228, (uint8_t)200, (uint8_t)166, (uint8_t)206, (uint8_t)61, (uint8_t)102, (uint8_t)167, (uint8_t)36, (uint8_t)87, (uint8_t)204, (uint8_t)156, (uint8_t)244, (uint8_t)184, (uint8_t)47, (uint8_t)217, (uint8_t)1, (uint8_t)16, (uint8_t)7, (uint8_t)61, (uint8_t)152, (uint8_t)201, (uint8_t)52, (uint8_t)73, (uint8_t)174, (uint8_t)213, (uint8_t)111, (uint8_t)43, (uint8_t)132, (uint8_t)246, (uint8_t)196, (uint8_t)35, (uint8_t)79, (uint8_t)193, (uint8_t)90, (uint8_t)194, (uint8_t)128, (uint8_t)18, (uint8_t)74, (uint8_t)70, (uint8_t)83, (uint8_t)3, (uint8_t)142, (uint8_t)42, (uint8_t)196, (uint8_t)244, (uint8_t)130, (uint8_t)3, (uint8_t)42, (uint8_t)121, (uint8_t)240, (uint8_t)33, (uint8_t)234, (uint8_t)14, (uint8_t)31, (uint8_t)109, (uint8_t)227, (uint8_t)39, (uint8_t)68, (uint8_t)17, (uint8_t)206, (uint8_t)80, (uint8_t)114, (uint8_t)116, (uint8_t)36, (uint8_t)58, (uint8_t)97, (uint8_t)179, (uint8_t)163, (uint8_t)82, (uint8_t)69, (uint8_t)82, (uint8_t)149, (uint8_t)162, (uint8_t)127, (uint8_t)85, (uint8_t)20, (uint8_t)97, (uint8_t)24, (uint8_t)84, (uint8_t)98, (uint8_t)14, (uint8_t)40, (uint8_t)204, (uint8_t)76, (uint8_t)88, (uint8_t)244, (uint8_t)191, (uint8_t)84, (uint8_t)82, (uint8_t)254, (uint8_t)175, (uint8_t)141, (uint8_t)15, (uint8_t)183, (uint8_t)62, (uint8_t)180, (uint8_t)81, (uint8_t)119, (uint8_t)46, (uint8_t)145, (uint8_t)24, (uint8_t)13, (uint8_t)54, (uint8_t)175, (uint8_t)96, (uint8_t)207, (uint8_t)252, (uint8_t)112, (uint8_t)206, (uint8_t)161, (uint8_t)27, (uint8_t)137, (uint8_t)121, (uint8_t)214, (uint8_t)78, (uint8_t)156, (uint8_t)171, (uint8_t)114, (uint8_t)26, (uint8_t)140, (uint8_t)189, (uint8_t)157, (uint8_t)229, (uint8_t)200, (uint8_t)29, (uint8_t)16, (uint8_t)140, (uint8_t)105, (uint8_t)5, (uint8_t)202, (uint8_t)109, (uint8_t)104, (uint8_t)29, (uint8_t)142, (uint8_t)150, (uint8_t)255, (uint8_t)207, (uint8_t)33, (uint8_t)148, (uint8_t)253, (uint8_t)170, (uint8_t)28, (uint8_t)164, (uint8_t)149, (uint8_t)122, (uint8_t)112, (uint8_t)183, (uint8_t)211, (uint8_t)181, (uint8_t)217, (uint8_t)253, (uint8_t)138, (uint8_t)65, (uint8_t)27, (uint8_t)63, (uint8_t)51, (uint8_t)181, (uint8_t)167, (uint8_t)254, (uint8_t)55, (uint8_t)200, (uint8_t)192, (uint8_t)85, (uint8_t)250, (uint8_t)166, (uint8_t)101, (uint8_t)96, (uint8_t)172, (uint8_t)99, (uint8_t)99, (uint8_t)87, (uint8_t)228, (uint8_t)147, (uint8_t)247, (uint8_t)164, (uint8_t)92, (uint8_t)229, (uint8_t)8, (uint8_t)242, (uint8_t)247, (uint8_t)46, (uint8_t)161, (uint8_t)43, (uint8_t)51, (uint8_t)133, (uint8_t)203, (uint8_t)111, (uint8_t)48, (uint8_t)14, (uint8_t)156, (uint8_t)149, (uint8_t)183, (uint8_t)41, (uint8_t)53, (uint8_t)207, (uint8_t)111, (uint8_t)199, (uint8_t)239, (uint8_t)214, (uint8_t)13, (uint8_t)128, (uint8_t)215, (uint8_t)37, (uint8_t)160, (uint8_t)80, (uint8_t)170, (uint8_t)117, (uint8_t)145, (uint8_t)191} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)57203);
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)4018740351L);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)1845);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)5509);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)2404);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)27589);
    assert(p133_lat_GET(pack) == (int32_t) -336534651);
    assert(p133_mask_GET(pack) == (uint64_t)1264647595439572164L);
    assert(p133_lon_GET(pack) == (int32_t) -1259362855);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)32174);
    assert(p134_lon_GET(pack) == (int32_t) -1434535575);
    {
        int16_t exemplary[] =  {(int16_t) -7288, (int16_t)16383, (int16_t)28963, (int16_t)6616, (int16_t)8316, (int16_t) -23706, (int16_t)8363, (int16_t) -14891, (int16_t)11849, (int16_t) -27069, (int16_t) -16468, (int16_t) -22286, (int16_t) -8726, (int16_t)3437, (int16_t) -26928, (int16_t) -28349} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p134_lat_GET(pack) == (int32_t)1660288323);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -374606277);
    assert(p135_lat_GET(pack) == (int32_t)470720237);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)6574);
    assert(p136_lon_GET(pack) == (int32_t) -2014111959);
    assert(p136_lat_GET(pack) == (int32_t)1805752955);
    assert(p136_terrain_height_GET(pack) == (float)9.793948E37F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)27745);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)51990);
    assert(p136_current_height_GET(pack) == (float) -1.8616023E38F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -16157);
    assert(p137_press_diff_GET(pack) == (float) -2.88736E38F);
    assert(p137_press_abs_GET(pack) == (float)1.5324353E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3972778260L);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-6.451227E37F, 9.145786E37F, 2.1323398E38F, -2.6745148E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_y_GET(pack) == (float) -9.013189E37F);
    assert(p138_x_GET(pack) == (float)5.183905E37F);
    assert(p138_time_usec_GET(pack) == (uint64_t)2069468069690135340L);
    assert(p138_z_GET(pack) == (float) -1.2950497E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)6062249554009329196L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)13);
    {
        float exemplary[] =  {-2.0456471E38F, -2.9213493E38F, -8.0701847E37F, -2.42146E38F, -3.361189E38F, -1.3080889E38F, 2.0437876E38F, 1.500458E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)147);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)5041773500179476783L);
    {
        float exemplary[] =  {3.2984989E38F, -1.3556889E38F, -3.3983333E38F, 3.2478156E38F, 6.5447476E36F, -9.645796E36F, -3.0551243E38F, 2.7261915E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)181);
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_relative_GET(pack) == (float) -4.650509E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.7098068E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)7.823466E36F);
    assert(p141_bottom_clearance_GET(pack) == (float) -7.200015E37F);
    assert(p141_altitude_local_GET(pack) == (float)2.0565946E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)6240076487266775004L);
    assert(p141_altitude_amsl_GET(pack) == (float) -7.6280677E37F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)106, (uint8_t)15, (uint8_t)22, (uint8_t)42, (uint8_t)39, (uint8_t)141, (uint8_t)5, (uint8_t)23, (uint8_t)194, (uint8_t)24, (uint8_t)76, (uint8_t)114, (uint8_t)253, (uint8_t)69, (uint8_t)183, (uint8_t)212, (uint8_t)2, (uint8_t)157, (uint8_t)67, (uint8_t)141, (uint8_t)82, (uint8_t)93, (uint8_t)100, (uint8_t)198, (uint8_t)3, (uint8_t)130, (uint8_t)238, (uint8_t)186, (uint8_t)176, (uint8_t)249, (uint8_t)0, (uint8_t)106, (uint8_t)107, (uint8_t)255, (uint8_t)134, (uint8_t)211, (uint8_t)114, (uint8_t)204, (uint8_t)255, (uint8_t)197, (uint8_t)87, (uint8_t)239, (uint8_t)96, (uint8_t)61, (uint8_t)186, (uint8_t)112, (uint8_t)225, (uint8_t)84, (uint8_t)176, (uint8_t)88, (uint8_t)156, (uint8_t)226, (uint8_t)250, (uint8_t)96, (uint8_t)199, (uint8_t)111, (uint8_t)52, (uint8_t)182, (uint8_t)213, (uint8_t)11, (uint8_t)197, (uint8_t)155, (uint8_t)18, (uint8_t)210, (uint8_t)203, (uint8_t)70, (uint8_t)175, (uint8_t)13, (uint8_t)113, (uint8_t)50, (uint8_t)188, (uint8_t)227, (uint8_t)29, (uint8_t)102, (uint8_t)57, (uint8_t)61, (uint8_t)88, (uint8_t)6, (uint8_t)61, (uint8_t)222, (uint8_t)182, (uint8_t)106, (uint8_t)184, (uint8_t)240, (uint8_t)236, (uint8_t)55, (uint8_t)175, (uint8_t)88, (uint8_t)146, (uint8_t)158, (uint8_t)226, (uint8_t)111, (uint8_t)108, (uint8_t)136, (uint8_t)212, (uint8_t)99, (uint8_t)187, (uint8_t)104, (uint8_t)136, (uint8_t)232, (uint8_t)180, (uint8_t)211, (uint8_t)8, (uint8_t)65, (uint8_t)64, (uint8_t)175, (uint8_t)94, (uint8_t)165, (uint8_t)196, (uint8_t)87, (uint8_t)173, (uint8_t)117, (uint8_t)242, (uint8_t)223, (uint8_t)17, (uint8_t)140, (uint8_t)202, (uint8_t)119, (uint8_t)53, (uint8_t)103} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)132);
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)30, (uint8_t)93, (uint8_t)151, (uint8_t)6, (uint8_t)232, (uint8_t)105, (uint8_t)143, (uint8_t)227, (uint8_t)25, (uint8_t)157, (uint8_t)130, (uint8_t)169, (uint8_t)194, (uint8_t)136, (uint8_t)25, (uint8_t)226, (uint8_t)141, (uint8_t)200, (uint8_t)242, (uint8_t)147, (uint8_t)2, (uint8_t)29, (uint8_t)52, (uint8_t)39, (uint8_t)114, (uint8_t)71, (uint8_t)109, (uint8_t)6, (uint8_t)89, (uint8_t)43, (uint8_t)241, (uint8_t)11, (uint8_t)177, (uint8_t)56, (uint8_t)139, (uint8_t)173, (uint8_t)238, (uint8_t)222, (uint8_t)185, (uint8_t)184, (uint8_t)27, (uint8_t)119, (uint8_t)44, (uint8_t)227, (uint8_t)233, (uint8_t)19, (uint8_t)250, (uint8_t)199, (uint8_t)178, (uint8_t)30, (uint8_t)39, (uint8_t)100, (uint8_t)228, (uint8_t)132, (uint8_t)242, (uint8_t)229, (uint8_t)70, (uint8_t)19, (uint8_t)114, (uint8_t)29, (uint8_t)179, (uint8_t)116, (uint8_t)116, (uint8_t)71, (uint8_t)22, (uint8_t)62, (uint8_t)193, (uint8_t)102, (uint8_t)208, (uint8_t)129, (uint8_t)77, (uint8_t)26, (uint8_t)28, (uint8_t)33, (uint8_t)221, (uint8_t)243, (uint8_t)251, (uint8_t)217, (uint8_t)194, (uint8_t)144, (uint8_t)144, (uint8_t)1, (uint8_t)117, (uint8_t)253, (uint8_t)159, (uint8_t)59, (uint8_t)175, (uint8_t)138, (uint8_t)122, (uint8_t)85, (uint8_t)205, (uint8_t)161, (uint8_t)103, (uint8_t)83, (uint8_t)115, (uint8_t)104, (uint8_t)71, (uint8_t)162, (uint8_t)128, (uint8_t)143, (uint8_t)42, (uint8_t)130, (uint8_t)2, (uint8_t)135, (uint8_t)98, (uint8_t)181, (uint8_t)143, (uint8_t)254, (uint8_t)232, (uint8_t)54, (uint8_t)214, (uint8_t)204, (uint8_t)40, (uint8_t)29, (uint8_t)90, (uint8_t)26, (uint8_t)89, (uint8_t)78, (uint8_t)7} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)176643694L);
    assert(p143_press_abs_GET(pack) == (float) -2.025529E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)11114);
    assert(p143_press_diff_GET(pack) == (float)1.5244773E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.7666785E38F, 2.613078E38F, 2.9254336E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)5336908620676108773L);
    assert(p144_custom_state_GET(pack) == (uint64_t)8913478194104340227L);
    assert(p144_lat_GET(pack) == (int32_t) -93181125);
    {
        float exemplary[] =  {1.6436897E38F, 9.618559E37F, 2.0776343E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t) -1533572787);
    {
        float exemplary[] =  {-1.1398464E38F, -1.04725E38F, 5.476229E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.0884544E38F, 1.0277152E38F, -1.6944959E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {8.152563E37F, 1.2902562E38F, 2.2670308E38F, 2.5008503E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p144_alt_GET(pack) == (float)4.734789E37F);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_y_vel_GET(pack) == (float) -2.301271E38F);
    assert(p146_y_pos_GET(pack) == (float)3.2602722E38F);
    {
        float exemplary[] =  {-3.0221038E38F, 5.3883916E36F, -2.8420856E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_roll_rate_GET(pack) == (float) -2.6390595E38F);
    assert(p146_y_acc_GET(pack) == (float) -1.3680707E38F);
    {
        float exemplary[] =  {2.327465E37F, 3.0296444E37F, 1.5844813E38F, 1.9787612E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {3.7857574E37F, 5.81117E37F, 8.600664E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float) -6.01052E37F);
    assert(p146_pitch_rate_GET(pack) == (float)2.0662802E38F);
    assert(p146_yaw_rate_GET(pack) == (float)3.3781014E38F);
    assert(p146_z_vel_GET(pack) == (float)1.0011744E37F);
    assert(p146_z_pos_GET(pack) == (float) -2.123568E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)436041890818856602L);
    assert(p146_x_pos_GET(pack) == (float)7.675606E37F);
    assert(p146_airspeed_GET(pack) == (float)6.3650836E37F);
    assert(p146_z_acc_GET(pack) == (float) -1.6775835E38F);
    assert(p146_x_acc_GET(pack) == (float) -3.354663E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -18418);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)88);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1504423173);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -18599);
    assert(p147_current_consumed_GET(pack) == (int32_t)1831234869);
    {
        uint16_t exemplary[] =  {(uint16_t)58158, (uint16_t)41723, (uint16_t)57296, (uint16_t)54563, (uint16_t)26403, (uint16_t)9528, (uint16_t)30931, (uint16_t)58487, (uint16_t)3320, (uint16_t)7773} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)29248);
    {
        uint8_t exemplary[] =  {(uint8_t)209, (uint8_t)48, (uint8_t)201, (uint8_t)23, (uint8_t)213, (uint8_t)30, (uint8_t)65, (uint8_t)118} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1470095165L);
    assert(p148_board_version_GET(pack) == (uint32_t)2971220480L);
    {
        uint8_t exemplary[] =  {(uint8_t)137, (uint8_t)204, (uint8_t)13, (uint8_t)140, (uint8_t)27, (uint8_t)121, (uint8_t)194, (uint8_t)234} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
    {
        uint8_t exemplary[] =  {(uint8_t)153, (uint8_t)219, (uint8_t)97, (uint8_t)8, (uint8_t)79, (uint8_t)162, (uint8_t)69, (uint8_t)153, (uint8_t)176, (uint8_t)232, (uint8_t)21, (uint8_t)242, (uint8_t)109, (uint8_t)185, (uint8_t)39, (uint8_t)200, (uint8_t)95, (uint8_t)241} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)2822070701982720481L);
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)229, (uint8_t)131, (uint8_t)186, (uint8_t)102, (uint8_t)76, (uint8_t)18, (uint8_t)236} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)44721);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)1331408180L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3438114637L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_angle_x_GET(pack) == (float) -2.4148075E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p149_angle_y_GET(pack) == (float) -2.1669767E38F);
    assert(p149_z_TRY(ph) == (float) -7.4046975E37F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_size_y_GET(pack) == (float) -1.6460075E38F);
    {
        float exemplary[] =  {-3.9778678E37F, 2.0180464E38F, -4.9996846E36F, -3.8511377E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p149_x_TRY(ph) == (float) -1.3803584E38F);
    assert(p149_y_TRY(ph) == (float) -3.069645E38F);
    assert(p149_size_x_GET(pack) == (float) -1.3106837E38F);
    assert(p149_distance_GET(pack) == (float) -1.0075667E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)200);
    assert(p149_time_usec_GET(pack) == (uint64_t)1775487527632103847L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)1.8470552E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.917916E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.1365245E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -4.1829126E37F);
    assert(p230_hagl_ratio_GET(pack) == (float)7.7758854E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -9.557742E37F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL));
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -3.2853072E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)3.0303612E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)5287178835736150153L);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float)4.0462925E37F);
    assert(p231_var_vert_GET(pack) == (float) -3.0402711E38F);
    assert(p231_wind_z_GET(pack) == (float)1.8237474E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.982345E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -1.6230908E38F);
    assert(p231_var_horiz_GET(pack) == (float)6.7785214E36F);
    assert(p231_wind_x_GET(pack) == (float) -3.0065384E38F);
    assert(p231_wind_y_GET(pack) == (float) -2.8806591E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)8117871042252918916L);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p232_vert_accuracy_GET(pack) == (float)5.220543E37F);
    assert(p232_hdop_GET(pack) == (float)1.4930187E37F);
    assert(p232_horiz_accuracy_GET(pack) == (float)3.2609186E38F);
    assert(p232_ve_GET(pack) == (float) -1.1895552E37F);
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP));
    assert(p232_vd_GET(pack) == (float)1.6291726E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p232_vn_GET(pack) == (float) -6.5181656E37F);
    assert(p232_alt_GET(pack) == (float) -5.1515607E37F);
    assert(p232_time_usec_GET(pack) == (uint64_t)6796294275446939390L);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)55463);
    assert(p232_vdop_GET(pack) == (float)2.2910862E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2920580970L);
    assert(p232_lat_GET(pack) == (int32_t) -762120614);
    assert(p232_lon_GET(pack) == (int32_t)101951546);
    assert(p232_speed_accuracy_GET(pack) == (float)1.4037119E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)116);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)41);
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)141, (uint8_t)137, (uint8_t)255, (uint8_t)198, (uint8_t)219, (uint8_t)255, (uint8_t)68, (uint8_t)45, (uint8_t)189, (uint8_t)124, (uint8_t)0, (uint8_t)211, (uint8_t)8, (uint8_t)98, (uint8_t)244, (uint8_t)133, (uint8_t)154, (uint8_t)80, (uint8_t)89, (uint8_t)62, (uint8_t)31, (uint8_t)70, (uint8_t)2, (uint8_t)164, (uint8_t)167, (uint8_t)95, (uint8_t)107, (uint8_t)244, (uint8_t)140, (uint8_t)150, (uint8_t)152, (uint8_t)234, (uint8_t)121, (uint8_t)134, (uint8_t)252, (uint8_t)13, (uint8_t)77, (uint8_t)106, (uint8_t)208, (uint8_t)64, (uint8_t)91, (uint8_t)171, (uint8_t)207, (uint8_t)200, (uint8_t)116, (uint8_t)184, (uint8_t)222, (uint8_t)15, (uint8_t)119, (uint8_t)208, (uint8_t)87, (uint8_t)222, (uint8_t)40, (uint8_t)227, (uint8_t)13, (uint8_t)25, (uint8_t)94, (uint8_t)221, (uint8_t)21, (uint8_t)204, (uint8_t)161, (uint8_t)221, (uint8_t)253, (uint8_t)136, (uint8_t)146, (uint8_t)251, (uint8_t)92, (uint8_t)148, (uint8_t)135, (uint8_t)204, (uint8_t)79, (uint8_t)250, (uint8_t)132, (uint8_t)116, (uint8_t)101, (uint8_t)33, (uint8_t)59, (uint8_t)195, (uint8_t)205, (uint8_t)4, (uint8_t)56, (uint8_t)166, (uint8_t)139, (uint8_t)58, (uint8_t)97, (uint8_t)238, (uint8_t)211, (uint8_t)223, (uint8_t)217, (uint8_t)182, (uint8_t)74, (uint8_t)37, (uint8_t)221, (uint8_t)78, (uint8_t)50, (uint8_t)169, (uint8_t)179, (uint8_t)203, (uint8_t)181, (uint8_t)61, (uint8_t)2, (uint8_t)89, (uint8_t)7, (uint8_t)159, (uint8_t)117, (uint8_t)205, (uint8_t)237, (uint8_t)252, (uint8_t)108, (uint8_t)192, (uint8_t)114, (uint8_t)155, (uint8_t)209, (uint8_t)163, (uint8_t)101, (uint8_t)172, (uint8_t)112, (uint8_t)17, (uint8_t)221, (uint8_t)201, (uint8_t)254, (uint8_t)83, (uint8_t)75, (uint8_t)51, (uint8_t)117, (uint8_t)26, (uint8_t)89, (uint8_t)40, (uint8_t)74, (uint8_t)251, (uint8_t)26, (uint8_t)83, (uint8_t)223, (uint8_t)168, (uint8_t)116, (uint8_t)179, (uint8_t)100, (uint8_t)111, (uint8_t)62, (uint8_t)170, (uint8_t)193, (uint8_t)83, (uint8_t)153, (uint8_t)110, (uint8_t)158, (uint8_t)220, (uint8_t)146, (uint8_t)78, (uint8_t)23, (uint8_t)240, (uint8_t)85, (uint8_t)219, (uint8_t)224, (uint8_t)253, (uint8_t)238, (uint8_t)14, (uint8_t)7, (uint8_t)82, (uint8_t)109, (uint8_t)181, (uint8_t)66, (uint8_t)147, (uint8_t)198, (uint8_t)0, (uint8_t)166, (uint8_t)146, (uint8_t)156, (uint8_t)116, (uint8_t)25, (uint8_t)121, (uint8_t)88, (uint8_t)12, (uint8_t)1, (uint8_t)22, (uint8_t)169, (uint8_t)180, (uint8_t)27, (uint8_t)222, (uint8_t)32} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)28);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)59);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -82);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)6019);
    assert(p234_custom_mode_GET(pack) == (uint32_t)2843085041L);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p234_latitude_GET(pack) == (int32_t)545338634);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -16936);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)34);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -13404);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)35800);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)818);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)11793);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED));
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_longitude_GET(pack) == (int32_t) -1408138842);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)38);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -17302);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)1936171052L);
    assert(p241_vibration_z_GET(pack) == (float) -2.0747193E38F);
    assert(p241_vibration_x_GET(pack) == (float) -2.1498667E38F);
    assert(p241_vibration_y_GET(pack) == (float) -1.8880999E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)885330228L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2384852834L);
    assert(p241_time_usec_GET(pack) == (uint64_t)88758116823500243L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-3.1711195E38F, 3.1123542E38F, -2.6607583E37F, -3.2880774E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_x_GET(pack) == (float) -2.6161317E38F);
    assert(p242_y_GET(pack) == (float)1.4482789E38F);
    assert(p242_approach_z_GET(pack) == (float) -2.332299E38F);
    assert(p242_z_GET(pack) == (float)8.927314E37F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)5816692232829493303L);
    assert(p242_approach_x_GET(pack) == (float) -3.188299E38F);
    assert(p242_altitude_GET(pack) == (int32_t)1984896424);
    assert(p242_approach_y_GET(pack) == (float) -3.4078446E37F);
    assert(p242_latitude_GET(pack) == (int32_t) -362013497);
    assert(p242_longitude_GET(pack) == (int32_t)1407934925);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_longitude_GET(pack) == (int32_t) -855408304);
    assert(p243_latitude_GET(pack) == (int32_t)973801516);
    assert(p243_altitude_GET(pack) == (int32_t) -1454459877);
    assert(p243_time_usec_TRY(ph) == (uint64_t)7136118091644479318L);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p243_approach_z_GET(pack) == (float) -8.928275E37F);
    assert(p243_approach_y_GET(pack) == (float)1.7741673E38F);
    {
        float exemplary[] =  {-3.0992475E38F, 2.0203825E38F, 1.3991387E38F, 2.6735978E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_x_GET(pack) == (float) -5.102158E37F);
    assert(p243_y_GET(pack) == (float) -1.1093138E38F);
    assert(p243_x_GET(pack) == (float) -2.003202E38F);
    assert(p243_z_GET(pack) == (float)3.1830455E38F);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)13097);
    assert(p244_interval_us_GET(pack) == (int32_t) -1276593059);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)64833);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS));
    assert(p246_lat_GET(pack) == (int32_t) -1282048658);
    assert(p246_callsign_LEN(ph) == 7);
    {
        char16_t * exemplary = u"jvzKyrx";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)56653);
    assert(p246_altitude_GET(pack) == (int32_t) -607948576);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3412688908L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)10442);
    assert(p246_lon_GET(pack) == (int32_t)2076718877);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -16645);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)2121070195L);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.3423435E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -3.3542013E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.4761509E38F);
    assert(p247_threat_level_GET(pack) == (e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW));
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)18894);
    {
        uint8_t exemplary[] =  {(uint8_t)170, (uint8_t)156, (uint8_t)242, (uint8_t)136, (uint8_t)188, (uint8_t)162, (uint8_t)16, (uint8_t)200, (uint8_t)160, (uint8_t)188, (uint8_t)232, (uint8_t)75, (uint8_t)64, (uint8_t)65, (uint8_t)224, (uint8_t)195, (uint8_t)243, (uint8_t)226, (uint8_t)136, (uint8_t)83, (uint8_t)156, (uint8_t)178, (uint8_t)149, (uint8_t)53, (uint8_t)63, (uint8_t)8, (uint8_t)208, (uint8_t)74, (uint8_t)81, (uint8_t)241, (uint8_t)42, (uint8_t)217, (uint8_t)77, (uint8_t)135, (uint8_t)108, (uint8_t)94, (uint8_t)107, (uint8_t)48, (uint8_t)104, (uint8_t)142, (uint8_t)138, (uint8_t)221, (uint8_t)71, (uint8_t)33, (uint8_t)108, (uint8_t)82, (uint8_t)20, (uint8_t)10, (uint8_t)223, (uint8_t)57, (uint8_t)3, (uint8_t)94, (uint8_t)117, (uint8_t)71, (uint8_t)241, (uint8_t)136, (uint8_t)139, (uint8_t)118, (uint8_t)152, (uint8_t)74, (uint8_t)158, (uint8_t)252, (uint8_t)185, (uint8_t)20, (uint8_t)125, (uint8_t)228, (uint8_t)122, (uint8_t)129, (uint8_t)78, (uint8_t)228, (uint8_t)150, (uint8_t)83, (uint8_t)234, (uint8_t)58, (uint8_t)180, (uint8_t)80, (uint8_t)35, (uint8_t)194, (uint8_t)137, (uint8_t)240, (uint8_t)73, (uint8_t)147, (uint8_t)137, (uint8_t)54, (uint8_t)88, (uint8_t)160, (uint8_t)28, (uint8_t)250, (uint8_t)139, (uint8_t)38, (uint8_t)41, (uint8_t)123, (uint8_t)168, (uint8_t)87, (uint8_t)61, (uint8_t)50, (uint8_t)194, (uint8_t)49, (uint8_t)171, (uint8_t)100, (uint8_t)0, (uint8_t)191, (uint8_t)69, (uint8_t)48, (uint8_t)24, (uint8_t)23, (uint8_t)3, (uint8_t)91, (uint8_t)248, (uint8_t)78, (uint8_t)63, (uint8_t)181, (uint8_t)81, (uint8_t)49, (uint8_t)121, (uint8_t)173, (uint8_t)103, (uint8_t)226, (uint8_t)161, (uint8_t)169, (uint8_t)58, (uint8_t)179, (uint8_t)27, (uint8_t)133, (uint8_t)8, (uint8_t)251, (uint8_t)166, (uint8_t)252, (uint8_t)116, (uint8_t)244, (uint8_t)142, (uint8_t)96, (uint8_t)35, (uint8_t)6, (uint8_t)239, (uint8_t)63, (uint8_t)165, (uint8_t)66, (uint8_t)209, (uint8_t)47, (uint8_t)65, (uint8_t)172, (uint8_t)69, (uint8_t)230, (uint8_t)113, (uint8_t)62, (uint8_t)167, (uint8_t)252, (uint8_t)138, (uint8_t)25, (uint8_t)55, (uint8_t)236, (uint8_t)218, (uint8_t)49, (uint8_t)57, (uint8_t)180, (uint8_t)76, (uint8_t)235, (uint8_t)252, (uint8_t)250, (uint8_t)71, (uint8_t)182, (uint8_t)15, (uint8_t)98, (uint8_t)64, (uint8_t)235, (uint8_t)195, (uint8_t)219, (uint8_t)31, (uint8_t)55, (uint8_t)131, (uint8_t)7, (uint8_t)181, (uint8_t)117, (uint8_t)170, (uint8_t)26, (uint8_t)230, (uint8_t)214, (uint8_t)89, (uint8_t)159, (uint8_t)62, (uint8_t)221, (uint8_t)49, (uint8_t)231, (uint8_t)35, (uint8_t)246, (uint8_t)78, (uint8_t)74, (uint8_t)128, (uint8_t)220, (uint8_t)198, (uint8_t)132, (uint8_t)91, (uint8_t)87, (uint8_t)38, (uint8_t)101, (uint8_t)15, (uint8_t)43, (uint8_t)38, (uint8_t)130, (uint8_t)213, (uint8_t)115, (uint8_t)125, (uint8_t)141, (uint8_t)63, (uint8_t)212, (uint8_t)217, (uint8_t)244, (uint8_t)88, (uint8_t)8, (uint8_t)174, (uint8_t)62, (uint8_t)16, (uint8_t)129, (uint8_t)129, (uint8_t)164, (uint8_t)255, (uint8_t)210, (uint8_t)24, (uint8_t)35, (uint8_t)53, (uint8_t)41, (uint8_t)76, (uint8_t)84, (uint8_t)146, (uint8_t)138, (uint8_t)191, (uint8_t)76, (uint8_t)198, (uint8_t)221, (uint8_t)238, (uint8_t)75, (uint8_t)160, (uint8_t)240, (uint8_t)120, (uint8_t)56, (uint8_t)51, (uint8_t)139, (uint8_t)186, (uint8_t)82, (uint8_t)139, (uint8_t)25, (uint8_t)130, (uint8_t)106, (uint8_t)230, (uint8_t)148, (uint8_t)19, (uint8_t)178, (uint8_t)25} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)38);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -76, (int8_t)36, (int8_t)94, (int8_t) -44, (int8_t)55, (int8_t) -103, (int8_t)96, (int8_t) -40, (int8_t)11, (int8_t)88, (int8_t)118, (int8_t) -72, (int8_t) -110, (int8_t) -117, (int8_t) -103, (int8_t) -77, (int8_t) -2, (int8_t) -108, (int8_t) -38, (int8_t)23, (int8_t)0, (int8_t)84, (int8_t) -39, (int8_t)25, (int8_t) -120, (int8_t)104, (int8_t)78, (int8_t)53, (int8_t)47, (int8_t) -2, (int8_t)28, (int8_t) -53} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)51563);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_z_GET(pack) == (float)2.1132204E38F);
    assert(p250_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"Fxvy";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_y_GET(pack) == (float)1.3338664E38F);
    assert(p250_x_GET(pack) == (float)3.1479594E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)2718307025972945720L);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"pde";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float)2.190758E38F);
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)4094056152L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3058828977L);
    assert(p252_value_GET(pack) == (int32_t)325996248);
    assert(p252_name_LEN(ph) == 7);
    {
        char16_t * exemplary = u"mvNplju";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_INFO);
    assert(p253_text_LEN(ph) == 49);
    {
        char16_t * exemplary = u"IldqvsepcKonfAsgPsTreVgMxzHuygMfvwsuundkjokqfqtYt";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 98);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)1.2999273E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1064951605L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)184);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)7997221146474400308L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)209);
    {
        uint8_t exemplary[] =  {(uint8_t)136, (uint8_t)85, (uint8_t)143, (uint8_t)33, (uint8_t)139, (uint8_t)42, (uint8_t)148, (uint8_t)42, (uint8_t)142, (uint8_t)238, (uint8_t)246, (uint8_t)85, (uint8_t)19, (uint8_t)177, (uint8_t)138, (uint8_t)102, (uint8_t)195, (uint8_t)89, (uint8_t)63, (uint8_t)191, (uint8_t)129, (uint8_t)171, (uint8_t)67, (uint8_t)42, (uint8_t)4, (uint8_t)5, (uint8_t)180, (uint8_t)40, (uint8_t)107, (uint8_t)115, (uint8_t)157, (uint8_t)115} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2740310731L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1132642732L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p258_tune_LEN(ph) == 13);
    {
        char16_t * exemplary = u"ruhfpEbhdHaqP";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)10469);
    assert(p259_focal_length_GET(pack) == (float)1.8539804E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)221, (uint8_t)54, (uint8_t)68, (uint8_t)76, (uint8_t)66, (uint8_t)158, (uint8_t)88, (uint8_t)24, (uint8_t)244, (uint8_t)48, (uint8_t)97, (uint8_t)31, (uint8_t)233, (uint8_t)200, (uint8_t)83, (uint8_t)102, (uint8_t)221, (uint8_t)165, (uint8_t)173, (uint8_t)94, (uint8_t)191, (uint8_t)49, (uint8_t)206, (uint8_t)213, (uint8_t)80, (uint8_t)235, (uint8_t)215, (uint8_t)72, (uint8_t)131, (uint8_t)45, (uint8_t)98, (uint8_t)102} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_uri_LEN(ph) == 31);
    {
        char16_t * exemplary = u"pdatowkyGvnebzhqmuyuDgitzvoalgg";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 62);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1324451707L);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.9260292E38F);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)18014);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1817569644L);
    assert(p259_sensor_size_v_GET(pack) == (float)1.6835256E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)68, (uint8_t)84, (uint8_t)213, (uint8_t)9, (uint8_t)73, (uint8_t)249, (uint8_t)80, (uint8_t)97, (uint8_t)141, (uint8_t)171, (uint8_t)235, (uint8_t)186, (uint8_t)98, (uint8_t)228, (uint8_t)94, (uint8_t)69, (uint8_t)91, (uint8_t)201, (uint8_t)160, (uint8_t)128, (uint8_t)167, (uint8_t)0, (uint8_t)117, (uint8_t)204, (uint8_t)199, (uint8_t)95, (uint8_t)113, (uint8_t)58, (uint8_t)155, (uint8_t)239, (uint8_t)108, (uint8_t)176} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)17445);
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES));
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == (e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY));
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)2251935856L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2466477652L);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p261_used_capacity_GET(pack) == (float) -2.0992014E38F);
    assert(p261_available_capacity_GET(pack) == (float)1.9025022E38F);
    assert(p261_total_capacity_GET(pack) == (float)2.3419926E38F);
    assert(p261_write_speed_GET(pack) == (float)2.8579588E38F);
    assert(p261_read_speed_GET(pack) == (float) -2.972034E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)4230862204L);
    assert(p262_image_interval_GET(pack) == (float) -3.0753607E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1697281424L);
    assert(p262_available_capacity_GET(pack) == (float) -1.137561E37F);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_image_index_GET(pack) == (int32_t) -190829341);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -73);
    assert(p263_alt_GET(pack) == (int32_t) -112134685);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)2394441637L);
    assert(p263_time_utc_GET(pack) == (uint64_t)2298377708204336141L);
    {
        float exemplary[] =  {-1.3732799E38F, -3.119649E36F, 2.2551616E38F, 3.0849064E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t) -22757982);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p263_lat_GET(pack) == (int32_t) -141458465);
    assert(p263_lon_GET(pack) == (int32_t)1237042074);
    assert(p263_file_url_LEN(ph) == 57);
    {
        char16_t * exemplary = u"zbsmmxnimrtupoqvmptjlvuqtghkyqlurubavjxcnrgcsxoodnBkejazt";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 114);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2249161870L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7244699179574670238L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)7335966714418326034L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)6164808241692094862L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1470190723L);
    assert(p265_roll_GET(pack) == (float) -1.3081709E38F);
    assert(p265_pitch_GET(pack) == (float) -1.6608219E38F);
    assert(p265_yaw_GET(pack) == (float) -2.3985385E38F);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)160);
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)231, (uint8_t)51, (uint8_t)229, (uint8_t)68, (uint8_t)229, (uint8_t)65, (uint8_t)15, (uint8_t)94, (uint8_t)251, (uint8_t)132, (uint8_t)245, (uint8_t)32, (uint8_t)213, (uint8_t)156, (uint8_t)113, (uint8_t)60, (uint8_t)212, (uint8_t)196, (uint8_t)16, (uint8_t)101, (uint8_t)183, (uint8_t)249, (uint8_t)219, (uint8_t)24, (uint8_t)192, (uint8_t)60, (uint8_t)105, (uint8_t)200, (uint8_t)93, (uint8_t)25, (uint8_t)218, (uint8_t)246, (uint8_t)102, (uint8_t)212, (uint8_t)42, (uint8_t)5, (uint8_t)199, (uint8_t)152, (uint8_t)131, (uint8_t)234, (uint8_t)105, (uint8_t)8, (uint8_t)88, (uint8_t)142, (uint8_t)188, (uint8_t)244, (uint8_t)32, (uint8_t)79, (uint8_t)8, (uint8_t)12, (uint8_t)240, (uint8_t)68, (uint8_t)129, (uint8_t)85, (uint8_t)188, (uint8_t)79, (uint8_t)144, (uint8_t)145, (uint8_t)227, (uint8_t)210, (uint8_t)55, (uint8_t)53, (uint8_t)6, (uint8_t)241, (uint8_t)240, (uint8_t)253, (uint8_t)7, (uint8_t)119, (uint8_t)53, (uint8_t)236, (uint8_t)91, (uint8_t)49, (uint8_t)26, (uint8_t)125, (uint8_t)205, (uint8_t)130, (uint8_t)4, (uint8_t)20, (uint8_t)214, (uint8_t)213, (uint8_t)9, (uint8_t)166, (uint8_t)40, (uint8_t)243, (uint8_t)253, (uint8_t)9, (uint8_t)139, (uint8_t)6, (uint8_t)60, (uint8_t)75, (uint8_t)246, (uint8_t)183, (uint8_t)52, (uint8_t)168, (uint8_t)104, (uint8_t)29, (uint8_t)70, (uint8_t)94, (uint8_t)94, (uint8_t)77, (uint8_t)18, (uint8_t)243, (uint8_t)128, (uint8_t)168, (uint8_t)213, (uint8_t)236, (uint8_t)178, (uint8_t)56, (uint8_t)65, (uint8_t)241, (uint8_t)48, (uint8_t)148, (uint8_t)75, (uint8_t)6, (uint8_t)1, (uint8_t)147, (uint8_t)117, (uint8_t)207, (uint8_t)177, (uint8_t)176, (uint8_t)145, (uint8_t)95, (uint8_t)168, (uint8_t)232, (uint8_t)106, (uint8_t)3, (uint8_t)203, (uint8_t)3, (uint8_t)3, (uint8_t)43, (uint8_t)208, (uint8_t)252, (uint8_t)153, (uint8_t)215, (uint8_t)171, (uint8_t)188, (uint8_t)124, (uint8_t)217, (uint8_t)172, (uint8_t)84, (uint8_t)82, (uint8_t)147, (uint8_t)43, (uint8_t)175, (uint8_t)173, (uint8_t)8, (uint8_t)143, (uint8_t)21, (uint8_t)141, (uint8_t)83, (uint8_t)67, (uint8_t)60, (uint8_t)19, (uint8_t)181, (uint8_t)240, (uint8_t)151, (uint8_t)72, (uint8_t)247, (uint8_t)64, (uint8_t)244, (uint8_t)173, (uint8_t)5, (uint8_t)248, (uint8_t)59, (uint8_t)16, (uint8_t)142, (uint8_t)54, (uint8_t)69, (uint8_t)31, (uint8_t)33, (uint8_t)73, (uint8_t)143, (uint8_t)161, (uint8_t)211, (uint8_t)61, (uint8_t)102, (uint8_t)109, (uint8_t)0, (uint8_t)45, (uint8_t)85, (uint8_t)122, (uint8_t)243, (uint8_t)2, (uint8_t)85, (uint8_t)30, (uint8_t)55, (uint8_t)161, (uint8_t)115, (uint8_t)221, (uint8_t)114, (uint8_t)75, (uint8_t)18, (uint8_t)197, (uint8_t)248, (uint8_t)250, (uint8_t)25, (uint8_t)196, (uint8_t)93, (uint8_t)75, (uint8_t)169, (uint8_t)241, (uint8_t)87, (uint8_t)34, (uint8_t)253, (uint8_t)33, (uint8_t)242, (uint8_t)224, (uint8_t)108, (uint8_t)184, (uint8_t)170, (uint8_t)57, (uint8_t)21, (uint8_t)171, (uint8_t)138, (uint8_t)52, (uint8_t)134, (uint8_t)183, (uint8_t)141, (uint8_t)177, (uint8_t)168, (uint8_t)116, (uint8_t)21, (uint8_t)126, (uint8_t)76, (uint8_t)79, (uint8_t)104, (uint8_t)90, (uint8_t)129, (uint8_t)249, (uint8_t)25, (uint8_t)140, (uint8_t)74, (uint8_t)190, (uint8_t)176, (uint8_t)198, (uint8_t)232, (uint8_t)245, (uint8_t)129, (uint8_t)170, (uint8_t)106, (uint8_t)131, (uint8_t)44, (uint8_t)140, (uint8_t)208, (uint8_t)111, (uint8_t)151, (uint8_t)221, (uint8_t)42} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)18973);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)200);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)7, (uint8_t)125, (uint8_t)105, (uint8_t)61, (uint8_t)161, (uint8_t)244, (uint8_t)201, (uint8_t)91, (uint8_t)132, (uint8_t)182, (uint8_t)75, (uint8_t)229, (uint8_t)175, (uint8_t)216, (uint8_t)155, (uint8_t)186, (uint8_t)162, (uint8_t)128, (uint8_t)56, (uint8_t)15, (uint8_t)132, (uint8_t)134, (uint8_t)135, (uint8_t)94, (uint8_t)180, (uint8_t)82, (uint8_t)232, (uint8_t)4, (uint8_t)119, (uint8_t)180, (uint8_t)106, (uint8_t)86, (uint8_t)11, (uint8_t)41, (uint8_t)47, (uint8_t)139, (uint8_t)192, (uint8_t)178, (uint8_t)168, (uint8_t)168, (uint8_t)60, (uint8_t)197, (uint8_t)242, (uint8_t)1, (uint8_t)73, (uint8_t)43, (uint8_t)38, (uint8_t)28, (uint8_t)26, (uint8_t)251, (uint8_t)180, (uint8_t)137, (uint8_t)83, (uint8_t)141, (uint8_t)15, (uint8_t)1, (uint8_t)50, (uint8_t)110, (uint8_t)33, (uint8_t)161, (uint8_t)140, (uint8_t)116, (uint8_t)23, (uint8_t)147, (uint8_t)164, (uint8_t)138, (uint8_t)66, (uint8_t)8, (uint8_t)26, (uint8_t)161, (uint8_t)115, (uint8_t)226, (uint8_t)150, (uint8_t)49, (uint8_t)248, (uint8_t)210, (uint8_t)3, (uint8_t)205, (uint8_t)111, (uint8_t)9, (uint8_t)68, (uint8_t)101, (uint8_t)5, (uint8_t)140, (uint8_t)152, (uint8_t)156, (uint8_t)125, (uint8_t)57, (uint8_t)163, (uint8_t)250, (uint8_t)159, (uint8_t)179, (uint8_t)252, (uint8_t)114, (uint8_t)76, (uint8_t)211, (uint8_t)10, (uint8_t)185, (uint8_t)10, (uint8_t)17, (uint8_t)51, (uint8_t)70, (uint8_t)109, (uint8_t)230, (uint8_t)165, (uint8_t)84, (uint8_t)240, (uint8_t)176, (uint8_t)162, (uint8_t)77, (uint8_t)161, (uint8_t)20, (uint8_t)22, (uint8_t)246, (uint8_t)213, (uint8_t)92, (uint8_t)192, (uint8_t)128, (uint8_t)5, (uint8_t)199, (uint8_t)241, (uint8_t)237, (uint8_t)40, (uint8_t)129, (uint8_t)173, (uint8_t)17, (uint8_t)174, (uint8_t)123, (uint8_t)69, (uint8_t)236, (uint8_t)158, (uint8_t)144, (uint8_t)6, (uint8_t)15, (uint8_t)92, (uint8_t)153, (uint8_t)57, (uint8_t)135, (uint8_t)21, (uint8_t)56, (uint8_t)201, (uint8_t)65, (uint8_t)46, (uint8_t)177, (uint8_t)250, (uint8_t)56, (uint8_t)86, (uint8_t)112, (uint8_t)73, (uint8_t)83, (uint8_t)139, (uint8_t)159, (uint8_t)165, (uint8_t)254, (uint8_t)254, (uint8_t)29, (uint8_t)163, (uint8_t)229, (uint8_t)103, (uint8_t)77, (uint8_t)127, (uint8_t)253, (uint8_t)32, (uint8_t)36, (uint8_t)48, (uint8_t)193, (uint8_t)127, (uint8_t)192, (uint8_t)159, (uint8_t)136, (uint8_t)22, (uint8_t)151, (uint8_t)157, (uint8_t)162, (uint8_t)128, (uint8_t)191, (uint8_t)155, (uint8_t)2, (uint8_t)74, (uint8_t)221, (uint8_t)5, (uint8_t)65, (uint8_t)69, (uint8_t)144, (uint8_t)226, (uint8_t)165, (uint8_t)38, (uint8_t)202, (uint8_t)221, (uint8_t)136, (uint8_t)142, (uint8_t)29, (uint8_t)2, (uint8_t)30, (uint8_t)30, (uint8_t)133, (uint8_t)50, (uint8_t)82, (uint8_t)4, (uint8_t)62, (uint8_t)8, (uint8_t)91, (uint8_t)99, (uint8_t)210, (uint8_t)147, (uint8_t)48, (uint8_t)187, (uint8_t)69, (uint8_t)243, (uint8_t)199, (uint8_t)67, (uint8_t)34, (uint8_t)28, (uint8_t)99, (uint8_t)117, (uint8_t)234, (uint8_t)199, (uint8_t)38, (uint8_t)39, (uint8_t)60, (uint8_t)93, (uint8_t)134, (uint8_t)224, (uint8_t)87, (uint8_t)174, (uint8_t)138, (uint8_t)210, (uint8_t)62, (uint8_t)5, (uint8_t)31, (uint8_t)161, (uint8_t)34, (uint8_t)110, (uint8_t)167, (uint8_t)225, (uint8_t)247, (uint8_t)165, (uint8_t)35, (uint8_t)1, (uint8_t)115, (uint8_t)210, (uint8_t)167, (uint8_t)177, (uint8_t)158, (uint8_t)30, (uint8_t)118, (uint8_t)151, (uint8_t)61, (uint8_t)2} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)44013);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)216);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)56352);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_bitrate_GET(pack) == (uint32_t)1680054574L);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)56060);
    assert(p269_uri_LEN(ph) == 147);
    {
        char16_t * exemplary = u"flnjvzezuhddnyekLPyulrdqpzgupfczsDGlqzdkpdaznlqlxzjjSzYniizzgfmhfjdkypqfkbEqvamgzgabjcqwfexsngibgkeDluphXuzupqgZkhplalpwyofvganMQtbypfbimyjumqLiosp";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 294);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float)2.1730022E38F);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)63140);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)14643);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float)3.3637989E38F);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)63693);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)58839);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)44866);
    assert(p270_bitrate_GET(pack) == (uint32_t)2649034022L);
    assert(p270_uri_LEN(ph) == 71);
    {
        char16_t * exemplary = u"gjNixrhbbzuunikuzvrxjunuaQchsvrhsHumegtttngektdilbDttxndjoopwupzfvnkkhu";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 142);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 59);
    {
        char16_t * exemplary = u"krzZcihktNhrufPhItxtItbjdnnelzhxtpfftgnowotbkYinudxitogznee";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 118);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 12);
    {
        char16_t * exemplary = u"hlptjeKdxuDk";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)9, (uint8_t)69, (uint8_t)146, (uint8_t)194, (uint8_t)33, (uint8_t)127, (uint8_t)34, (uint8_t)153} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)7415);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)13503);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)32071);
    {
        uint8_t exemplary[] =  {(uint8_t)167, (uint8_t)113, (uint8_t)200, (uint8_t)202, (uint8_t)31, (uint8_t)251, (uint8_t)204, (uint8_t)132} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)8183);
    assert(p310_time_usec_GET(pack) == (uint64_t)7656805574587324619L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1729834692L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2218605585L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p311_name_LEN(ph) == 24);
    {
        char16_t * exemplary = u"xquwtltouZvdpXRywwSopogf";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)187, (uint8_t)153, (uint8_t)33, (uint8_t)244, (uint8_t)153, (uint8_t)18, (uint8_t)167, (uint8_t)90, (uint8_t)165, (uint8_t)237, (uint8_t)53, (uint8_t)14, (uint8_t)29, (uint8_t)196, (uint8_t)77, (uint8_t)193} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1864135422L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p311_time_usec_GET(pack) == (uint64_t)8998787099209742211L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p320_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"rrzhv";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)21278);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"odvtpvaimyxialA";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)39163);
    assert(p322_param_value_LEN(ph) == 80);
    {
        char16_t * exemplary = u"uAztmrhiulRuuhpsixwzwfahduxwsfwaaLodhqvjxlumrbrrsdwbmZilpdMikcyvjtomybbmwvwvdqrf";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 160);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)45208);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p323_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"meoO";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p323_param_value_LEN(ph) == 116);
    {
        char16_t * exemplary = u"lZbnwsrndvbJvtpskopUgncafqforxqpvorFHrsqarwBbnpmftankbbFiyvaijltovGqqvokJtzetpacaXEedvpplyrgnoojXyxblZirtrlfomqEZcwj";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 232);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_value_LEN(ph) == 93);
    {
        char16_t * exemplary = u"hncrjdrvnglkbpcwlYtukkfiprddBwkazngajjUocukyesirvpaoffGbfTjmmbgkpxvggctLkeyVmpejovutkgxdbfuIp";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 186);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"knyvzpgPal";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)58241);
    {
        uint16_t exemplary[] =  {(uint16_t)38336, (uint16_t)44768, (uint16_t)33457, (uint16_t)30564, (uint16_t)24839, (uint16_t)4701, (uint16_t)19218, (uint16_t)20883, (uint16_t)4387, (uint16_t)44434, (uint16_t)60078, (uint16_t)16338, (uint16_t)251, (uint16_t)10886, (uint16_t)7750, (uint16_t)58009, (uint16_t)60772, (uint16_t)43028, (uint16_t)64898, (uint16_t)55500, (uint16_t)58525, (uint16_t)21857, (uint16_t)42226, (uint16_t)65471, (uint16_t)5051, (uint16_t)26527, (uint16_t)51777, (uint16_t)20932, (uint16_t)3072, (uint16_t)38498, (uint16_t)33704, (uint16_t)26461, (uint16_t)23940, (uint16_t)61891, (uint16_t)2747, (uint16_t)61745, (uint16_t)59979, (uint16_t)47293, (uint16_t)61525, (uint16_t)15311, (uint16_t)41090, (uint16_t)28435, (uint16_t)57613, (uint16_t)37969, (uint16_t)49526, (uint16_t)39896, (uint16_t)14746, (uint16_t)35036, (uint16_t)11846, (uint16_t)30606, (uint16_t)12686, (uint16_t)57765, (uint16_t)4656, (uint16_t)15589, (uint16_t)15104, (uint16_t)41375, (uint16_t)32693, (uint16_t)41810, (uint16_t)4299, (uint16_t)28555, (uint16_t)39729, (uint16_t)21543, (uint16_t)53504, (uint16_t)54054, (uint16_t)46056, (uint16_t)21137, (uint16_t)10431, (uint16_t)41055, (uint16_t)54350, (uint16_t)63474, (uint16_t)12657, (uint16_t)18700} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p330_time_usec_GET(pack) == (uint64_t)7480347028072290336L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)4806);
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
        p0_custom_mode_SET((uint32_t)1060501840L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED), PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count4_SET((uint16_t)(uint16_t)11524, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)32220, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)27432, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)40903, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -13001, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)10059, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL), PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)105, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)33591, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)1857, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)8325, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING), PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)1105336511L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)7634622423534182884L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_vz_SET((float)3.3353678E38F, PH.base.pack) ;
        p3_z_SET((float) -1.8385365E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p3_afz_SET((float)9.214037E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -1.0090224E38F, PH.base.pack) ;
        p3_x_SET((float) -2.9445163E38F, PH.base.pack) ;
        p3_vy_SET((float)1.3642794E38F, PH.base.pack) ;
        p3_afy_SET((float) -2.8402219E38F, PH.base.pack) ;
        p3_vx_SET((float) -3.068526E38F, PH.base.pack) ;
        p3_afx_SET((float)8.729704E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1074675970L, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)22294, PH.base.pack) ;
        p3_yaw_SET((float) -1.7793626E38F, PH.base.pack) ;
        p3_y_SET((float)1.9573801E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)7694752356110336154L, PH.base.pack) ;
        p4_seq_SET((uint32_t)219616255L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        {
            char16_t* passkey = u"unsSvpetKkoNCwoqskxfbuvb";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"yrasWkmvgysvonvz";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)249607035L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"Fssxj";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t)15820, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_count_SET((uint16_t)(uint16_t)15042, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)8502, PH.base.pack) ;
        p22_param_value_SET((float) -7.9419654E37F, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        {
            char16_t* param_id = u"zvxoroblihnbzntd";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        {
            char16_t* param_id = u"rlffz";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
        p23_param_value_SET((float)1.1738127E38F, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_v_acc_SET((uint32_t)1189866364L, &PH) ;
        p24_alt_SET((int32_t)515059622, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)1049345780, &PH) ;
        p24_lon_SET((int32_t) -323317869, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)22006, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)1608262074L, &PH) ;
        p24_lat_SET((int32_t) -892740890, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)3780895093L, &PH) ;
        p24_vel_acc_SET((uint32_t)2179923220L, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)19212, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)54292, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)30746, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)8283869478514346971L, PH.base.pack) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)113, (uint8_t)47, (uint8_t)234, (uint8_t)129, (uint8_t)45, (uint8_t)189, (uint8_t)138, (uint8_t)252, (uint8_t)240, (uint8_t)149, (uint8_t)79, (uint8_t)159, (uint8_t)56, (uint8_t)68, (uint8_t)250, (uint8_t)239, (uint8_t)123, (uint8_t)148, (uint8_t)88, (uint8_t)230};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)96, (uint8_t)241, (uint8_t)155, (uint8_t)231, (uint8_t)39, (uint8_t)188, (uint8_t)69, (uint8_t)58, (uint8_t)12, (uint8_t)159, (uint8_t)183, (uint8_t)119, (uint8_t)80, (uint8_t)84, (uint8_t)133, (uint8_t)59, (uint8_t)210, (uint8_t)47, (uint8_t)225, (uint8_t)146};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)103, (uint8_t)211, (uint8_t)75, (uint8_t)182, (uint8_t)67, (uint8_t)109, (uint8_t)130, (uint8_t)110, (uint8_t)185, (uint8_t)27, (uint8_t)29, (uint8_t)56, (uint8_t)241, (uint8_t)189, (uint8_t)214, (uint8_t)227, (uint8_t)87, (uint8_t)193, (uint8_t)156, (uint8_t)186};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)73, (uint8_t)129, (uint8_t)47, (uint8_t)10, (uint8_t)40, (uint8_t)76, (uint8_t)31, (uint8_t)49, (uint8_t)126, (uint8_t)105, (uint8_t)118, (uint8_t)255, (uint8_t)49, (uint8_t)143, (uint8_t)181, (uint8_t)180, (uint8_t)134, (uint8_t)112, (uint8_t)118, (uint8_t)10};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)50, (uint8_t)9, (uint8_t)106, (uint8_t)69, (uint8_t)164, (uint8_t)226, (uint8_t)59, (uint8_t)192, (uint8_t)221, (uint8_t)165, (uint8_t)191, (uint8_t)177, (uint8_t)173, (uint8_t)112, (uint8_t)14, (uint8_t)178, (uint8_t)239, (uint8_t)41, (uint8_t)73, (uint8_t)204};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)2032376289L, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -19170, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -4552, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)11430, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)12711, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -8344, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)22378, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)11921, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -11756, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -31342, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_xacc_SET((int16_t)(int16_t)5434, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)6361320954881913915L, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -12857, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)30209, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -19606, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -8216, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)14861, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)18836, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)4416, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)14994, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -20992, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)6384161350934797425L, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)1097, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)4889, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)22159, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_time_boot_ms_SET((uint32_t)1939714557L, PH.base.pack) ;
        p29_press_diff_SET((float) -1.4760601E37F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -14326, PH.base.pack) ;
        p29_press_abs_SET((float) -1.5992514E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_time_boot_ms_SET((uint32_t)3148874044L, PH.base.pack) ;
        p30_rollspeed_SET((float)2.3539909E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)4.450594E37F, PH.base.pack) ;
        p30_pitch_SET((float) -2.0181127E38F, PH.base.pack) ;
        p30_roll_SET((float) -5.9697794E37F, PH.base.pack) ;
        p30_yaw_SET((float) -6.862133E36F, PH.base.pack) ;
        p30_yawspeed_SET((float)2.4052636E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q3_SET((float) -2.4151715E37F, PH.base.pack) ;
        p31_q2_SET((float) -1.9803148E38F, PH.base.pack) ;
        p31_q4_SET((float)2.5400593E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)8.0957517E36F, PH.base.pack) ;
        p31_rollspeed_SET((float)2.5469498E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1726364953L, PH.base.pack) ;
        p31_yawspeed_SET((float) -8.991098E37F, PH.base.pack) ;
        p31_q1_SET((float)1.2429265E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vx_SET((float) -1.8877108E38F, PH.base.pack) ;
        p32_vy_SET((float)2.0215652E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1490719189L, PH.base.pack) ;
        p32_z_SET((float) -2.1291113E38F, PH.base.pack) ;
        p32_x_SET((float)1.606349E38F, PH.base.pack) ;
        p32_y_SET((float) -7.6629773E37F, PH.base.pack) ;
        p32_vz_SET((float)2.7021467E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_alt_SET((int32_t)963642354, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)55671, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -13740, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)5634, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -32615, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1679850299L, PH.base.pack) ;
        p33_lat_SET((int32_t)1125743699, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -1260420412, PH.base.pack) ;
        p33_lon_SET((int32_t)1536839455, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan2_scaled_SET((int16_t)(int16_t)17247, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)30199, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -15855, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -27963, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)75113065L, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)14103, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)23161, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -13852, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -4473, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan1_raw_SET((uint16_t)(uint16_t)34443, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)50601, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)3889897813L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)27119, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)32706, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)64603, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)63074, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)60141, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)28564, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo5_raw_SET((uint16_t)(uint16_t)4824, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)44182, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)45342, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)63643, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)35025, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)36039, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)21393, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)24880, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)3905139285L, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)11094, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)54740, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)23341, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)57421, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)49598, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)36822, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)10501, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)37897, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -8621, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -28643, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)13894, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)30602, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_param1_SET((float) -1.2874269E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p39_z_SET((float) -1.2325662E38F, PH.base.pack) ;
        p39_param4_SET((float)2.9200435E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)12607, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p39_x_SET((float)1.169651E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p39_param2_SET((float) -5.1676233E36F, PH.base.pack) ;
        p39_param3_SET((float) -6.1445824E37F, PH.base.pack) ;
        p39_y_SET((float)1.0887765E38F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_STORAGE_FORMAT, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)12883, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)28654, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)9249, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)13073, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)40595, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)2297995389226948791L, &PH) ;
        p48_longitude_SET((int32_t)1942755452, PH.base.pack) ;
        p48_latitude_SET((int32_t)1880154801, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1897744154, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t) -2136336510, PH.base.pack) ;
        p49_longitude_SET((int32_t)1654509081, PH.base.pack) ;
        p49_altitude_SET((int32_t)1962497384, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)4614774793136427420L, &PH) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p50_scale_SET((float)2.6056047E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.294345E37F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p50_param_value_min_SET((float) -1.5213065E38F, PH.base.pack) ;
        p50_param_value0_SET((float)2.1272854E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"yxdxYfoxjpgt";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_index_SET((int16_t)(int16_t)9827, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)57906, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1z_SET((float) -2.6240566E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p54_p2z_SET((float) -3.173437E38F, PH.base.pack) ;
        p54_p1y_SET((float)1.0995837E38F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p54_p2y_SET((float) -2.6984233E38F, PH.base.pack) ;
        p54_p2x_SET((float)7.4545096E37F, PH.base.pack) ;
        p54_p1x_SET((float)2.7934573E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1z_SET((float) -2.8225478E38F, PH.base.pack) ;
        p55_p2x_SET((float) -8.367556E37F, PH.base.pack) ;
        p55_p1y_SET((float) -1.3722208E38F, PH.base.pack) ;
        p55_p2y_SET((float)3.036551E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p55_p1x_SET((float)1.2510541E38F, PH.base.pack) ;
        p55_p2z_SET((float)1.7487183E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float) -3.3065735E37F, PH.base.pack) ;
        p61_yawspeed_SET((float)1.7565254E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float)2.0389326E38F, PH.base.pack) ;
        {
            float covariance[] =  {3.2924492E38F, -2.372499E38F, -3.0619506E38F, -1.7453472E38F, 1.8144025E38F, 3.1344575E36F, 2.2472502E38F, -2.9819428E38F, 1.3755758E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        {
            float q[] =  {-2.1568913E38F, 1.4918001E38F, 2.3371337E38F, 3.3887769E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)7935005055702342539L, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_xtrack_error_SET((float) -2.086882E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)29735, PH.base.pack) ;
        p62_alt_error_SET((float) -2.8065545E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)858, PH.base.pack) ;
        p62_nav_roll_SET((float)1.3077961E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)2.8886658E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)29523, PH.base.pack) ;
        p62_aspd_error_SET((float)2.7348182E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p63_vz_SET((float)1.6707489E36F, PH.base.pack) ;
        p63_lon_SET((int32_t)168131907, PH.base.pack) ;
        p63_lat_SET((int32_t) -1350483138, PH.base.pack) ;
        p63_vx_SET((float) -7.0632046E37F, PH.base.pack) ;
        p63_alt_SET((int32_t)1451700335, PH.base.pack) ;
        {
            float covariance[] =  {-9.47555E37F, -7.827303E37F, 2.899872E38F, -3.0329425E38F, -2.3923755E38F, -1.7897152E38F, -2.8081144E38F, 1.2352173E38F, -1.0865552E38F, 3.3407814E38F, -3.1845638E38F, 1.2975235E38F, -3.1763426E38F, -1.1650083E38F, -3.1442382E38F, 1.8813612E38F, -2.2833734E38F, 3.7725054E37F, 9.340371E37F, -1.0670057E38F, -2.3884799E38F, -3.2039092E38F, -2.480092E38F, -1.6912496E38F, -1.7302471E37F, 2.9905823E38F, 1.1005789E38F, -3.3061685E38F, -2.2753556E38F, -2.6101327E38F, 2.7677346E38F, 1.3077347E38F, 1.4976541E38F, -1.3547927E38F, 2.6614951E38F, -2.3651273E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_relative_alt_SET((int32_t)125838608, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)6185445466815249261L, PH.base.pack) ;
        p63_vy_SET((float)2.0571779E38F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vz_SET((float)1.6880419E38F, PH.base.pack) ;
        p64_az_SET((float) -2.7940283E38F, PH.base.pack) ;
        p64_z_SET((float)1.2196505E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)2294485730366572808L, PH.base.pack) ;
        p64_ax_SET((float) -1.2626822E38F, PH.base.pack) ;
        p64_x_SET((float)1.3050805E38F, PH.base.pack) ;
        p64_ay_SET((float) -2.421818E38F, PH.base.pack) ;
        p64_vx_SET((float) -1.0162065E38F, PH.base.pack) ;
        {
            float covariance[] =  {-2.0377043E38F, -2.071029E38F, 3.0549292E38F, 2.6636452E38F, -1.8794192E38F, 2.8698894E38F, 1.0656874E36F, 3.0693095E38F, 1.298982E38F, -1.5649746E38F, -2.8529138E38F, 9.932878E36F, 2.9926576E37F, 2.0676016E38F, -2.1946105E38F, -2.6913827E38F, 8.669231E37F, -1.0735569E38F, -1.3451566E38F, -2.5044446E38F, 1.3240429E38F, 1.1653121E38F, -1.4157384E38F, 2.882579E38F, 1.4770544E38F, 1.8289164E38F, 1.8928488E38F, -2.660423E38F, -4.979486E37F, 1.0063902E38F, 2.7669657E38F, 5.0793715E37F, 1.896898E38F, -2.5899305E38F, 1.9656882E38F, 3.6805852E37F, 9.101184E37F, 2.7958472E38F, 2.6063777E38F, 7.551962E37F, -2.6833013E38F, -1.3314866E38F, 1.0726121E38F, -2.903881E38F, 7.7668557E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float)2.3888334E38F, PH.base.pack) ;
        p64_vy_SET((float) -2.3622876E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan9_raw_SET((uint16_t)(uint16_t)52030, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)34161, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)17732, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)63884, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)60811, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)33041, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)37565, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)27235, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)52473, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)1559, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)5632, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)17230, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)983290654L, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)10412, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)39606, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)51841, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)10682, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)17653, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)34750, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)25617, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)25103, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t)19348, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)2939, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -21068, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -7987, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)62405, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan7_raw_SET((uint16_t)(uint16_t)35545, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)53706, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)8707, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)36856, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)65515, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)29773, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)2560, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)52556, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p73_y_SET((int32_t) -666837606, PH.base.pack) ;
        p73_param3_SET((float)1.6439301E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p73_x_SET((int32_t) -1721357994, PH.base.pack) ;
        p73_z_SET((float) -2.5875047E38F, PH.base.pack) ;
        p73_param1_SET((float)1.9862022E38F, PH.base.pack) ;
        p73_param4_SET((float) -2.3288741E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)46032, PH.base.pack) ;
        p73_param2_SET((float) -2.412339E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float)3.1604836E38F, PH.base.pack) ;
        p74_airspeed_SET((float) -1.5876115E38F, PH.base.pack) ;
        p74_groundspeed_SET((float) -3.2368858E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)2897, PH.base.pack) ;
        p74_climb_SET((float) -1.6707182E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)23529, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_command_SET(e_MAV_CMD_MAV_CMD_IMAGE_START_CAPTURE, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p75_y_SET((int32_t)1702662376, PH.base.pack) ;
        p75_z_SET((float) -1.2996527E38F, PH.base.pack) ;
        p75_x_SET((int32_t) -1085923689, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p75_param2_SET((float) -6.614056E36F, PH.base.pack) ;
        p75_param4_SET((float)1.1004004E38F, PH.base.pack) ;
        p75_param1_SET((float) -3.0052724E38F, PH.base.pack) ;
        p75_param3_SET((float)2.591768E37F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_component_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p76_param1_SET((float) -2.6371179E38F, PH.base.pack) ;
        p76_param4_SET((float) -9.478394E37F, PH.base.pack) ;
        p76_param3_SET((float) -1.947417E37F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p76_param7_SET((float)1.83045E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE, PH.base.pack) ;
        p76_param2_SET((float)2.6724991E38F, PH.base.pack) ;
        p76_param5_SET((float)1.6550932E38F, PH.base.pack) ;
        p76_param6_SET((float)2.7348268E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)220, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)142, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)145, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_result_param2_SET((int32_t) -1163543969, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_CHANGE_ALT, PH.base.pack) ;
        c_CommunicationChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_time_boot_ms_SET((uint32_t)450653911L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p81_roll_SET((float) -2.050227E38F, PH.base.pack) ;
        p81_pitch_SET((float) -2.174981E38F, PH.base.pack) ;
        p81_yaw_SET((float) -2.4052832E36F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p81_thrust_SET((float) -3.1473649E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_roll_rate_SET((float) -1.6304781E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)881908173L, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)2.0367486E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        {
            float q[] =  {-1.3717455E38F, 2.3196087E38F, -1.800006E38F, -8.394787E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_thrust_SET((float) -2.03571E38F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -1.6124421E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_thrust_SET((float) -1.7353663E37F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)1.0140239E38F, PH.base.pack) ;
        {
            float q[] =  {2.7537884E38F, 1.1154149E38F, -1.7469682E38F, 1.549366E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)3.1208087E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)3731750320L, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)1.9702304E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_z_SET((float) -1.7374338E38F, PH.base.pack) ;
        p84_afy_SET((float)2.6483706E38F, PH.base.pack) ;
        p84_vy_SET((float)9.152354E37F, PH.base.pack) ;
        p84_y_SET((float) -2.1754468E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p84_x_SET((float) -3.3474953E38F, PH.base.pack) ;
        p84_vx_SET((float)1.6197272E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)292776474L, PH.base.pack) ;
        p84_yaw_SET((float) -5.50723E37F, PH.base.pack) ;
        p84_afx_SET((float)2.2206561E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)62633, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.6679453E38F, PH.base.pack) ;
        p84_vz_SET((float) -9.860154E37F, PH.base.pack) ;
        p84_afz_SET((float)9.497912E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_vy_SET((float) -5.708503E37F, PH.base.pack) ;
        p86_yaw_SET((float) -3.399446E38F, PH.base.pack) ;
        p86_alt_SET((float) -1.2333781E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p86_vx_SET((float) -2.2100614E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -1.9643246E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p86_lat_int_SET((int32_t)61107920, PH.base.pack) ;
        p86_afx_SET((float)1.6076738E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)1930383390, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)18124, PH.base.pack) ;
        p86_afy_SET((float) -1.9833387E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p86_vz_SET((float)3.3718283E38F, PH.base.pack) ;
        p86_afz_SET((float) -3.5330718E37F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2659883812L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vz_SET((float) -1.7957315E38F, PH.base.pack) ;
        p87_alt_SET((float) -1.8839837E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2906881351L, PH.base.pack) ;
        p87_yaw_SET((float) -2.57991E37F, PH.base.pack) ;
        p87_afy_SET((float) -2.272436E38F, PH.base.pack) ;
        p87_vy_SET((float) -1.6507752E38F, PH.base.pack) ;
        p87_vx_SET((float)6.403594E37F, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -880345366, PH.base.pack) ;
        p87_afx_SET((float) -2.8697687E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float) -4.518336E37F, PH.base.pack) ;
        p87_afz_SET((float) -1.388324E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)19527, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -2100545874, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_y_SET((float)7.832998E37F, PH.base.pack) ;
        p89_yaw_SET((float) -3.2715452E38F, PH.base.pack) ;
        p89_pitch_SET((float) -1.5732044E38F, PH.base.pack) ;
        p89_roll_SET((float)5.030443E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2372448754L, PH.base.pack) ;
        p89_z_SET((float)1.7574874E38F, PH.base.pack) ;
        p89_x_SET((float) -2.7154958E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_yacc_SET((int16_t)(int16_t)10214, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)5756680667953247296L, PH.base.pack) ;
        p90_alt_SET((int32_t) -784558480, PH.base.pack) ;
        p90_rollspeed_SET((float) -2.3859267E38F, PH.base.pack) ;
        p90_roll_SET((float) -2.3571089E38F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)23885, PH.base.pack) ;
        p90_lon_SET((int32_t) -529783883, PH.base.pack) ;
        p90_pitch_SET((float)5.9599597E37F, PH.base.pack) ;
        p90_yaw_SET((float)2.7065188E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)26587, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -27690, PH.base.pack) ;
        p90_yawspeed_SET((float)1.3272538E38F, PH.base.pack) ;
        p90_pitchspeed_SET((float)2.1119138E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -25168, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)8772, PH.base.pack) ;
        p90_lat_SET((int32_t)1447341091, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_throttle_SET((float) -1.2361726E38F, PH.base.pack) ;
        p91_aux3_SET((float)3.2303009E38F, PH.base.pack) ;
        p91_aux4_SET((float)9.286032E37F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.4260721E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.1384934E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -1.9582243E38F, PH.base.pack) ;
        p91_aux2_SET((float)1.3445593E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)5414419863471487181L, PH.base.pack) ;
        p91_aux1_SET((float)2.4524768E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan10_raw_SET((uint16_t)(uint16_t)43048, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)63392, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)23808, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)22742, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)31413, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)3064731715882247312L, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)10163, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)62594, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)50620, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)3362, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)23641, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)8945, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)19828, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {1.1401436E38F, 1.9270877E38F, -3.0882139E38F, 7.986186E37F, -1.47155E38F, -5.7408975E37F, -2.6365013E38F, 3.3117346E38F, 5.3224065E37F, 5.0036067E36F, -2.6785273E38F, 1.9791021E37F, -5.3181147E37F, -2.9705142E38F, -1.4168404E38F, 3.0350504E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)7259605327592818091L, PH.base.pack) ;
        p93_flags_SET((uint64_t)2598394582242494798L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -1.3724552E38F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -8.769822E37F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)9642, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -30041, PH.base.pack) ;
        p100_ground_distance_SET((float)2.2481607E38F, PH.base.pack) ;
        p100_flow_rate_x_SET((float)1.2600626E38F, &PH) ;
        p100_flow_rate_y_SET((float)1.3554497E38F, &PH) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)1355266182916426271L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_pitch_SET((float)2.6341733E38F, PH.base.pack) ;
        p101_y_SET((float)2.5222981E38F, PH.base.pack) ;
        p101_x_SET((float) -2.4737396E38F, PH.base.pack) ;
        p101_roll_SET((float)1.6704782E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)9074240654075090863L, PH.base.pack) ;
        p101_z_SET((float) -3.3105097E38F, PH.base.pack) ;
        p101_yaw_SET((float)3.0502856E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_roll_SET((float)3.204445E38F, PH.base.pack) ;
        p102_pitch_SET((float) -1.7474474E38F, PH.base.pack) ;
        p102_y_SET((float) -2.886323E38F, PH.base.pack) ;
        p102_x_SET((float)2.6751815E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)3158962391191535772L, PH.base.pack) ;
        p102_yaw_SET((float)1.3625497E37F, PH.base.pack) ;
        p102_z_SET((float)1.5985928E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float) -1.6305857E38F, PH.base.pack) ;
        p103_y_SET((float)3.2706298E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)176232725612833455L, PH.base.pack) ;
        p103_z_SET((float) -3.3275185E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float)2.091895E38F, PH.base.pack) ;
        p104_x_SET((float)1.0963697E38F, PH.base.pack) ;
        p104_yaw_SET((float)1.5150606E38F, PH.base.pack) ;
        p104_y_SET((float) -2.1407958E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)8940800004365184444L, PH.base.pack) ;
        p104_pitch_SET((float)1.4328009E38F, PH.base.pack) ;
        p104_roll_SET((float)1.5303244E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_temperature_SET((float) -1.7549057E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -2.247365E38F, PH.base.pack) ;
        p105_zacc_SET((float) -1.6470903E38F, PH.base.pack) ;
        p105_zgyro_SET((float)1.6982933E38F, PH.base.pack) ;
        p105_ymag_SET((float)3.8880685E37F, PH.base.pack) ;
        p105_pressure_alt_SET((float)2.4878684E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float)1.1688609E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -2.2073342E38F, PH.base.pack) ;
        p105_xgyro_SET((float)2.3537682E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.559563E38F, PH.base.pack) ;
        p105_xmag_SET((float) -2.5997867E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)1276669002784845366L, PH.base.pack) ;
        p105_yacc_SET((float)3.0881003E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)44005, PH.base.pack) ;
        p105_xacc_SET((float)1.51999E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_sensor_id_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -21101, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.393106E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)9156898140728557126L, PH.base.pack) ;
        p106_integrated_x_SET((float)1.6443057E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)2264475641L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p106_integrated_y_SET((float)6.6494526E37F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)2.8053854E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1452688228L, PH.base.pack) ;
        p106_distance_SET((float)2.1020458E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -1.7499715E37F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_zmag_SET((float) -2.2100229E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)1.1886964E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2770239675L, PH.base.pack) ;
        p107_temperature_SET((float) -9.36641E37F, PH.base.pack) ;
        p107_zgyro_SET((float)3.6043954E37F, PH.base.pack) ;
        p107_ygyro_SET((float) -2.0316864E37F, PH.base.pack) ;
        p107_ymag_SET((float)3.2262726E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float) -1.222173E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -2.0265759E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)264837455914001734L, PH.base.pack) ;
        p107_yacc_SET((float) -2.0126888E38F, PH.base.pack) ;
        p107_xmag_SET((float)2.1292374E38F, PH.base.pack) ;
        p107_zacc_SET((float)3.758573E37F, PH.base.pack) ;
        p107_xgyro_SET((float) -2.3846102E38F, PH.base.pack) ;
        p107_xacc_SET((float) -1.5529292E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_vn_SET((float) -2.8793454E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -1.1840725E38F, PH.base.pack) ;
        p108_lat_SET((float)1.5458847E38F, PH.base.pack) ;
        p108_zacc_SET((float) -2.3460213E38F, PH.base.pack) ;
        p108_ve_SET((float)1.705164E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -1.364973E38F, PH.base.pack) ;
        p108_q2_SET((float)2.8743123E38F, PH.base.pack) ;
        p108_alt_SET((float)9.407917E37F, PH.base.pack) ;
        p108_vd_SET((float)1.2222868E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.9987922E38F, PH.base.pack) ;
        p108_roll_SET((float)3.1525858E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -3.01782E38F, PH.base.pack) ;
        p108_q4_SET((float)9.974749E37F, PH.base.pack) ;
        p108_xacc_SET((float) -9.227157E37F, PH.base.pack) ;
        p108_q1_SET((float)7.8641276E37F, PH.base.pack) ;
        p108_q3_SET((float) -9.928618E37F, PH.base.pack) ;
        p108_yaw_SET((float)7.089557E37F, PH.base.pack) ;
        p108_yacc_SET((float)1.812067E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -9.397873E37F, PH.base.pack) ;
        p108_xgyro_SET((float)4.555993E37F, PH.base.pack) ;
        p108_lon_SET((float)1.4632474E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_txbuf_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)61437, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)10288, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)145, (uint8_t)95, (uint8_t)188, (uint8_t)53, (uint8_t)106, (uint8_t)173, (uint8_t)129, (uint8_t)239, (uint8_t)93, (uint8_t)203, (uint8_t)71, (uint8_t)59, (uint8_t)109, (uint8_t)152, (uint8_t)37, (uint8_t)117, (uint8_t)50, (uint8_t)222, (uint8_t)50, (uint8_t)158, (uint8_t)15, (uint8_t)236, (uint8_t)1, (uint8_t)103, (uint8_t)63, (uint8_t)122, (uint8_t)246, (uint8_t)219, (uint8_t)203, (uint8_t)105, (uint8_t)157, (uint8_t)11, (uint8_t)33, (uint8_t)229, (uint8_t)143, (uint8_t)85, (uint8_t)213, (uint8_t)28, (uint8_t)144, (uint8_t)76, (uint8_t)159, (uint8_t)21, (uint8_t)178, (uint8_t)46, (uint8_t)153, (uint8_t)105, (uint8_t)0, (uint8_t)195, (uint8_t)23, (uint8_t)84, (uint8_t)67, (uint8_t)116, (uint8_t)246, (uint8_t)78, (uint8_t)70, (uint8_t)239, (uint8_t)232, (uint8_t)8, (uint8_t)145, (uint8_t)87, (uint8_t)130, (uint8_t)167, (uint8_t)255, (uint8_t)76, (uint8_t)8, (uint8_t)29, (uint8_t)16, (uint8_t)177, (uint8_t)35, (uint8_t)236, (uint8_t)138, (uint8_t)112, (uint8_t)250, (uint8_t)253, (uint8_t)225, (uint8_t)178, (uint8_t)54, (uint8_t)194, (uint8_t)226, (uint8_t)207, (uint8_t)208, (uint8_t)103, (uint8_t)195, (uint8_t)203, (uint8_t)188, (uint8_t)202, (uint8_t)46, (uint8_t)98, (uint8_t)157, (uint8_t)50, (uint8_t)148, (uint8_t)214, (uint8_t)157, (uint8_t)170, (uint8_t)44, (uint8_t)243, (uint8_t)74, (uint8_t)115, (uint8_t)99, (uint8_t)148, (uint8_t)14, (uint8_t)207, (uint8_t)109, (uint8_t)116, (uint8_t)138, (uint8_t)236, (uint8_t)117, (uint8_t)204, (uint8_t)131, (uint8_t)77, (uint8_t)123, (uint8_t)175, (uint8_t)228, (uint8_t)5, (uint8_t)71, (uint8_t)122, (uint8_t)120, (uint8_t)88, (uint8_t)170, (uint8_t)198, (uint8_t)121, (uint8_t)19, (uint8_t)194, (uint8_t)130, (uint8_t)213, (uint8_t)28, (uint8_t)35, (uint8_t)122, (uint8_t)172, (uint8_t)148, (uint8_t)44, (uint8_t)143, (uint8_t)219, (uint8_t)192, (uint8_t)41, (uint8_t)82, (uint8_t)179, (uint8_t)94, (uint8_t)85, (uint8_t)193, (uint8_t)108, (uint8_t)98, (uint8_t)223, (uint8_t)0, (uint8_t)88, (uint8_t)11, (uint8_t)21, (uint8_t)46, (uint8_t)196, (uint8_t)13, (uint8_t)195, (uint8_t)201, (uint8_t)83, (uint8_t)235, (uint8_t)134, (uint8_t)254, (uint8_t)235, (uint8_t)211, (uint8_t)110, (uint8_t)213, (uint8_t)28, (uint8_t)94, (uint8_t)15, (uint8_t)32, (uint8_t)58, (uint8_t)38, (uint8_t)75, (uint8_t)14, (uint8_t)151, (uint8_t)174, (uint8_t)154, (uint8_t)190, (uint8_t)113, (uint8_t)237, (uint8_t)113, (uint8_t)205, (uint8_t)45, (uint8_t)72, (uint8_t)167, (uint8_t)20, (uint8_t)236, (uint8_t)78, (uint8_t)21, (uint8_t)34, (uint8_t)227, (uint8_t)151, (uint8_t)239, (uint8_t)180, (uint8_t)151, (uint8_t)238, (uint8_t)50, (uint8_t)202, (uint8_t)40, (uint8_t)98, (uint8_t)233, (uint8_t)17, (uint8_t)139, (uint8_t)141, (uint8_t)165, (uint8_t)202, (uint8_t)162, (uint8_t)204, (uint8_t)132, (uint8_t)124, (uint8_t)72, (uint8_t)39, (uint8_t)78, (uint8_t)158, (uint8_t)20, (uint8_t)227, (uint8_t)199, (uint8_t)86, (uint8_t)40, (uint8_t)90, (uint8_t)39, (uint8_t)85, (uint8_t)73, (uint8_t)183, (uint8_t)57, (uint8_t)57, (uint8_t)117, (uint8_t)162, (uint8_t)214, (uint8_t)249, (uint8_t)235, (uint8_t)245, (uint8_t)12, (uint8_t)29, (uint8_t)131, (uint8_t)123, (uint8_t)22, (uint8_t)158, (uint8_t)173, (uint8_t)166, (uint8_t)27, (uint8_t)162, (uint8_t)4, (uint8_t)197, (uint8_t)150, (uint8_t)25, (uint8_t)79, (uint8_t)176, (uint8_t)91, (uint8_t)91, (uint8_t)9, (uint8_t)180, (uint8_t)240, (uint8_t)86, (uint8_t)146, (uint8_t)3, (uint8_t)69};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)3280321088926868591L, PH.base.pack) ;
        p111_tc1_SET((int64_t) -4762151346494955480L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)3380251666L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)4955804822926667329L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_time_usec_SET((uint64_t)5816670287078710709L, PH.base.pack) ;
        p113_lon_SET((int32_t)1526053565, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -27480, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)53945, PH.base.pack) ;
        p113_alt_SET((int32_t) -1377047855, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)25057, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)65329, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)12735, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)12053, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -25072, PH.base.pack) ;
        p113_lat_SET((int32_t) -111109987, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_temperature_SET((int16_t)(int16_t)9600, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)6706405504069426863L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.6760674E38F, PH.base.pack) ;
        p114_distance_SET((float)1.3868266E38F, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -3.0507764E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)717073274L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3690541706L, PH.base.pack) ;
        p114_integrated_y_SET((float)2.6968279E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p114_integrated_x_SET((float)3.0666058E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)3.257685E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_lon_SET((int32_t) -1790890013, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)59408, PH.base.pack) ;
        p115_yawspeed_SET((float) -1.2281251E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)26429, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)21586, PH.base.pack) ;
        p115_pitchspeed_SET((float)1.7492317E38F, PH.base.pack) ;
        p115_alt_SET((int32_t)1170515319, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)16551, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)20137, PH.base.pack) ;
        p115_lat_SET((int32_t)1278509821, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-7.6124153E37F, -1.552223E38F, -2.6259518E38F, 2.9674731E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_xacc_SET((int16_t)(int16_t)15850, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)7409921347770046185L, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -25396, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)1023, PH.base.pack) ;
        p115_rollspeed_SET((float)2.154888E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t) -9056, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)15634, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)9903, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -30706, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)1870412910L, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)27006, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -21197, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -23421, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -32265, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -21078, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)17836, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)47350, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_id_SET((uint16_t)(uint16_t)2257, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)20670, PH.base.pack) ;
        p118_size_SET((uint32_t)244851988L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)60668, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)2895104652L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)37888, PH.base.pack) ;
        p119_count_SET((uint32_t)3621370107L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2600576083L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)7531, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)72, (uint8_t)236, (uint8_t)184, (uint8_t)145, (uint8_t)222, (uint8_t)173, (uint8_t)5, (uint8_t)243, (uint8_t)179, (uint8_t)114, (uint8_t)180, (uint8_t)3, (uint8_t)127, (uint8_t)123, (uint8_t)27, (uint8_t)186, (uint8_t)136, (uint8_t)124, (uint8_t)129, (uint8_t)62, (uint8_t)70, (uint8_t)250, (uint8_t)138, (uint8_t)254, (uint8_t)122, (uint8_t)208, (uint8_t)229, (uint8_t)48, (uint8_t)220, (uint8_t)68, (uint8_t)138, (uint8_t)226, (uint8_t)49, (uint8_t)172, (uint8_t)63, (uint8_t)172, (uint8_t)140, (uint8_t)158, (uint8_t)8, (uint8_t)154, (uint8_t)74, (uint8_t)185, (uint8_t)86, (uint8_t)110, (uint8_t)225, (uint8_t)176, (uint8_t)233, (uint8_t)43, (uint8_t)165, (uint8_t)60, (uint8_t)192, (uint8_t)77, (uint8_t)208, (uint8_t)48, (uint8_t)99, (uint8_t)11, (uint8_t)107, (uint8_t)105, (uint8_t)0, (uint8_t)17, (uint8_t)106, (uint8_t)81, (uint8_t)155, (uint8_t)138, (uint8_t)52, (uint8_t)90, (uint8_t)15, (uint8_t)109, (uint8_t)18, (uint8_t)71, (uint8_t)156, (uint8_t)84, (uint8_t)241, (uint8_t)38, (uint8_t)41, (uint8_t)13, (uint8_t)103, (uint8_t)58, (uint8_t)62, (uint8_t)173, (uint8_t)233, (uint8_t)118, (uint8_t)247, (uint8_t)98, (uint8_t)196, (uint8_t)162, (uint8_t)232, (uint8_t)165, (uint8_t)248, (uint8_t)171};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)1140444340L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)101, (uint8_t)207, (uint8_t)6, (uint8_t)18, (uint8_t)188, (uint8_t)170, (uint8_t)222, (uint8_t)213, (uint8_t)61, (uint8_t)11, (uint8_t)172, (uint8_t)120, (uint8_t)82, (uint8_t)209, (uint8_t)4, (uint8_t)125, (uint8_t)222, (uint8_t)210, (uint8_t)49, (uint8_t)136, (uint8_t)244, (uint8_t)169, (uint8_t)232, (uint8_t)8, (uint8_t)135, (uint8_t)164, (uint8_t)112, (uint8_t)5, (uint8_t)118, (uint8_t)12, (uint8_t)11, (uint8_t)60, (uint8_t)149, (uint8_t)150, (uint8_t)34, (uint8_t)248, (uint8_t)131, (uint8_t)188, (uint8_t)81, (uint8_t)36, (uint8_t)107, (uint8_t)0, (uint8_t)238, (uint8_t)205, (uint8_t)224, (uint8_t)116, (uint8_t)30, (uint8_t)7, (uint8_t)237, (uint8_t)120, (uint8_t)175, (uint8_t)93, (uint8_t)219, (uint8_t)227, (uint8_t)243, (uint8_t)207, (uint8_t)246, (uint8_t)159, (uint8_t)241, (uint8_t)201, (uint8_t)182, (uint8_t)202, (uint8_t)120, (uint8_t)93, (uint8_t)170, (uint8_t)182, (uint8_t)9, (uint8_t)154, (uint8_t)127, (uint8_t)5, (uint8_t)163, (uint8_t)236, (uint8_t)160, (uint8_t)164, (uint8_t)44, (uint8_t)16, (uint8_t)153, (uint8_t)237, (uint8_t)161, (uint8_t)67, (uint8_t)41, (uint8_t)190, (uint8_t)158, (uint8_t)44, (uint8_t)196, (uint8_t)160, (uint8_t)203, (uint8_t)137, (uint8_t)231, (uint8_t)69, (uint8_t)144, (uint8_t)74, (uint8_t)212, (uint8_t)46, (uint8_t)52, (uint8_t)227, (uint8_t)64, (uint8_t)214, (uint8_t)62, (uint8_t)31, (uint8_t)168, (uint8_t)141, (uint8_t)13, (uint8_t)23, (uint8_t)209, (uint8_t)79, (uint8_t)26, (uint8_t)30, (uint8_t)156, (uint8_t)142};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_vel_SET((uint16_t)(uint16_t)16916, PH.base.pack) ;
        p124_lon_SET((int32_t) -2132123142, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)38499, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)39860, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)4131071916201263244L, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)38232, PH.base.pack) ;
        p124_lat_SET((int32_t)1822807975, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1492970984L, PH.base.pack) ;
        p124_alt_SET((int32_t)1871906893, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)35956, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)60155, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED), PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)3605374248L, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)27450, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)214, (uint8_t)86, (uint8_t)141, (uint8_t)172, (uint8_t)156, (uint8_t)122, (uint8_t)68, (uint8_t)225, (uint8_t)151, (uint8_t)102, (uint8_t)160, (uint8_t)185, (uint8_t)192, (uint8_t)159, (uint8_t)185, (uint8_t)233, (uint8_t)183, (uint8_t)43, (uint8_t)75, (uint8_t)95, (uint8_t)233, (uint8_t)101, (uint8_t)79, (uint8_t)45, (uint8_t)164, (uint8_t)52, (uint8_t)135, (uint8_t)169, (uint8_t)186, (uint8_t)82, (uint8_t)207, (uint8_t)250, (uint8_t)122, (uint8_t)1, (uint8_t)68, (uint8_t)48, (uint8_t)205, (uint8_t)141, (uint8_t)243, (uint8_t)247, (uint8_t)218, (uint8_t)122, (uint8_t)140, (uint8_t)169, (uint8_t)236, (uint8_t)107, (uint8_t)249, (uint8_t)207, (uint8_t)88, (uint8_t)118, (uint8_t)200, (uint8_t)95, (uint8_t)241, (uint8_t)202, (uint8_t)8, (uint8_t)2, (uint8_t)143, (uint8_t)200, (uint8_t)191, (uint8_t)220, (uint8_t)234, (uint8_t)35, (uint8_t)145, (uint8_t)243, (uint8_t)98, (uint8_t)140, (uint8_t)71, (uint8_t)50, (uint8_t)245, (uint8_t)105};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_count_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_rtk_rate_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -1232844035, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)286814817, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1221428579, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2957776115L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)12878, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)2050411559L, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p127_tow_SET((uint32_t)1290372543L, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)210161443, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_c_mm_SET((int32_t) -601496323, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)107695784, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)46598, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)354316918, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)158353155L, PH.base.pack) ;
        p128_tow_SET((uint32_t)1202810911L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2197929479L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -614350915, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ygyro_SET((int16_t)(int16_t)28897, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)22595, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)17033, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)3403146973L, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -4164, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -8844, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)10427, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -29553, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)11748, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)31208, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_height_SET((uint16_t)(uint16_t)37769, PH.base.pack) ;
        p130_size_SET((uint32_t)1858162012L, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)15324, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)53860, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)57203, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)137, (uint8_t)214, (uint8_t)216, (uint8_t)170, (uint8_t)198, (uint8_t)208, (uint8_t)34, (uint8_t)156, (uint8_t)131, (uint8_t)59, (uint8_t)23, (uint8_t)246, (uint8_t)250, (uint8_t)68, (uint8_t)213, (uint8_t)192, (uint8_t)80, (uint8_t)51, (uint8_t)47, (uint8_t)122, (uint8_t)47, (uint8_t)152, (uint8_t)74, (uint8_t)101, (uint8_t)60, (uint8_t)120, (uint8_t)235, (uint8_t)24, (uint8_t)105, (uint8_t)148, (uint8_t)117, (uint8_t)176, (uint8_t)210, (uint8_t)112, (uint8_t)166, (uint8_t)173, (uint8_t)176, (uint8_t)233, (uint8_t)30, (uint8_t)187, (uint8_t)53, (uint8_t)241, (uint8_t)95, (uint8_t)33, (uint8_t)228, (uint8_t)200, (uint8_t)166, (uint8_t)206, (uint8_t)61, (uint8_t)102, (uint8_t)167, (uint8_t)36, (uint8_t)87, (uint8_t)204, (uint8_t)156, (uint8_t)244, (uint8_t)184, (uint8_t)47, (uint8_t)217, (uint8_t)1, (uint8_t)16, (uint8_t)7, (uint8_t)61, (uint8_t)152, (uint8_t)201, (uint8_t)52, (uint8_t)73, (uint8_t)174, (uint8_t)213, (uint8_t)111, (uint8_t)43, (uint8_t)132, (uint8_t)246, (uint8_t)196, (uint8_t)35, (uint8_t)79, (uint8_t)193, (uint8_t)90, (uint8_t)194, (uint8_t)128, (uint8_t)18, (uint8_t)74, (uint8_t)70, (uint8_t)83, (uint8_t)3, (uint8_t)142, (uint8_t)42, (uint8_t)196, (uint8_t)244, (uint8_t)130, (uint8_t)3, (uint8_t)42, (uint8_t)121, (uint8_t)240, (uint8_t)33, (uint8_t)234, (uint8_t)14, (uint8_t)31, (uint8_t)109, (uint8_t)227, (uint8_t)39, (uint8_t)68, (uint8_t)17, (uint8_t)206, (uint8_t)80, (uint8_t)114, (uint8_t)116, (uint8_t)36, (uint8_t)58, (uint8_t)97, (uint8_t)179, (uint8_t)163, (uint8_t)82, (uint8_t)69, (uint8_t)82, (uint8_t)149, (uint8_t)162, (uint8_t)127, (uint8_t)85, (uint8_t)20, (uint8_t)97, (uint8_t)24, (uint8_t)84, (uint8_t)98, (uint8_t)14, (uint8_t)40, (uint8_t)204, (uint8_t)76, (uint8_t)88, (uint8_t)244, (uint8_t)191, (uint8_t)84, (uint8_t)82, (uint8_t)254, (uint8_t)175, (uint8_t)141, (uint8_t)15, (uint8_t)183, (uint8_t)62, (uint8_t)180, (uint8_t)81, (uint8_t)119, (uint8_t)46, (uint8_t)145, (uint8_t)24, (uint8_t)13, (uint8_t)54, (uint8_t)175, (uint8_t)96, (uint8_t)207, (uint8_t)252, (uint8_t)112, (uint8_t)206, (uint8_t)161, (uint8_t)27, (uint8_t)137, (uint8_t)121, (uint8_t)214, (uint8_t)78, (uint8_t)156, (uint8_t)171, (uint8_t)114, (uint8_t)26, (uint8_t)140, (uint8_t)189, (uint8_t)157, (uint8_t)229, (uint8_t)200, (uint8_t)29, (uint8_t)16, (uint8_t)140, (uint8_t)105, (uint8_t)5, (uint8_t)202, (uint8_t)109, (uint8_t)104, (uint8_t)29, (uint8_t)142, (uint8_t)150, (uint8_t)255, (uint8_t)207, (uint8_t)33, (uint8_t)148, (uint8_t)253, (uint8_t)170, (uint8_t)28, (uint8_t)164, (uint8_t)149, (uint8_t)122, (uint8_t)112, (uint8_t)183, (uint8_t)211, (uint8_t)181, (uint8_t)217, (uint8_t)253, (uint8_t)138, (uint8_t)65, (uint8_t)27, (uint8_t)63, (uint8_t)51, (uint8_t)181, (uint8_t)167, (uint8_t)254, (uint8_t)55, (uint8_t)200, (uint8_t)192, (uint8_t)85, (uint8_t)250, (uint8_t)166, (uint8_t)101, (uint8_t)96, (uint8_t)172, (uint8_t)99, (uint8_t)99, (uint8_t)87, (uint8_t)228, (uint8_t)147, (uint8_t)247, (uint8_t)164, (uint8_t)92, (uint8_t)229, (uint8_t)8, (uint8_t)242, (uint8_t)247, (uint8_t)46, (uint8_t)161, (uint8_t)43, (uint8_t)51, (uint8_t)133, (uint8_t)203, (uint8_t)111, (uint8_t)48, (uint8_t)14, (uint8_t)156, (uint8_t)149, (uint8_t)183, (uint8_t)41, (uint8_t)53, (uint8_t)207, (uint8_t)111, (uint8_t)199, (uint8_t)239, (uint8_t)214, (uint8_t)13, (uint8_t)128, (uint8_t)215, (uint8_t)37, (uint8_t)160, (uint8_t)80, (uint8_t)170, (uint8_t)117, (uint8_t)145, (uint8_t)191};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_covariance_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)1845, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)4018740351L, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)5509, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)2404, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)27589, PH.base.pack) ;
        p133_lat_SET((int32_t) -336534651, PH.base.pack) ;
        p133_mask_SET((uint64_t)1264647595439572164L, PH.base.pack) ;
        p133_lon_SET((int32_t) -1259362855, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t) -1434535575, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)32174, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -7288, (int16_t)16383, (int16_t)28963, (int16_t)6616, (int16_t)8316, (int16_t) -23706, (int16_t)8363, (int16_t) -14891, (int16_t)11849, (int16_t) -27069, (int16_t) -16468, (int16_t) -22286, (int16_t) -8726, (int16_t)3437, (int16_t) -26928, (int16_t) -28349};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lat_SET((int32_t)1660288323, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)470720237, PH.base.pack) ;
        p135_lon_SET((int32_t) -374606277, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_loaded_SET((uint16_t)(uint16_t)6574, PH.base.pack) ;
        p136_lat_SET((int32_t)1805752955, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)51990, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)27745, PH.base.pack) ;
        p136_lon_SET((int32_t) -2014111959, PH.base.pack) ;
        p136_current_height_SET((float) -1.8616023E38F, PH.base.pack) ;
        p136_terrain_height_SET((float)9.793948E37F, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t) -16157, PH.base.pack) ;
        p137_press_diff_SET((float) -2.88736E38F, PH.base.pack) ;
        p137_press_abs_SET((float)1.5324353E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3972778260L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float)5.183905E37F, PH.base.pack) ;
        p138_z_SET((float) -1.2950497E38F, PH.base.pack) ;
        p138_y_SET((float) -9.013189E37F, PH.base.pack) ;
        {
            float q[] =  {-6.451227E37F, 9.145786E37F, 2.1323398E38F, -2.6745148E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_time_usec_SET((uint64_t)2069468069690135340L, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)6062249554009329196L, PH.base.pack) ;
        {
            float controls[] =  {-2.0456471E38F, -2.9213493E38F, -8.0701847E37F, -2.42146E38F, -3.361189E38F, -1.3080889E38F, 2.0437876E38F, 1.500458E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {3.2984989E38F, -1.3556889E38F, -3.3983333E38F, 3.2478156E38F, 6.5447476E36F, -9.645796E36F, -3.0551243E38F, 2.7261915E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)5041773500179476783L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ALTITUDE_141(), &PH);
        p141_altitude_amsl_SET((float) -7.6280677E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -4.650509E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)2.0565946E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -7.200015E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)7.823466E36F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.7098068E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)6240076487266775004L, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)232, (uint8_t)30, (uint8_t)93, (uint8_t)151, (uint8_t)6, (uint8_t)232, (uint8_t)105, (uint8_t)143, (uint8_t)227, (uint8_t)25, (uint8_t)157, (uint8_t)130, (uint8_t)169, (uint8_t)194, (uint8_t)136, (uint8_t)25, (uint8_t)226, (uint8_t)141, (uint8_t)200, (uint8_t)242, (uint8_t)147, (uint8_t)2, (uint8_t)29, (uint8_t)52, (uint8_t)39, (uint8_t)114, (uint8_t)71, (uint8_t)109, (uint8_t)6, (uint8_t)89, (uint8_t)43, (uint8_t)241, (uint8_t)11, (uint8_t)177, (uint8_t)56, (uint8_t)139, (uint8_t)173, (uint8_t)238, (uint8_t)222, (uint8_t)185, (uint8_t)184, (uint8_t)27, (uint8_t)119, (uint8_t)44, (uint8_t)227, (uint8_t)233, (uint8_t)19, (uint8_t)250, (uint8_t)199, (uint8_t)178, (uint8_t)30, (uint8_t)39, (uint8_t)100, (uint8_t)228, (uint8_t)132, (uint8_t)242, (uint8_t)229, (uint8_t)70, (uint8_t)19, (uint8_t)114, (uint8_t)29, (uint8_t)179, (uint8_t)116, (uint8_t)116, (uint8_t)71, (uint8_t)22, (uint8_t)62, (uint8_t)193, (uint8_t)102, (uint8_t)208, (uint8_t)129, (uint8_t)77, (uint8_t)26, (uint8_t)28, (uint8_t)33, (uint8_t)221, (uint8_t)243, (uint8_t)251, (uint8_t)217, (uint8_t)194, (uint8_t)144, (uint8_t)144, (uint8_t)1, (uint8_t)117, (uint8_t)253, (uint8_t)159, (uint8_t)59, (uint8_t)175, (uint8_t)138, (uint8_t)122, (uint8_t)85, (uint8_t)205, (uint8_t)161, (uint8_t)103, (uint8_t)83, (uint8_t)115, (uint8_t)104, (uint8_t)71, (uint8_t)162, (uint8_t)128, (uint8_t)143, (uint8_t)42, (uint8_t)130, (uint8_t)2, (uint8_t)135, (uint8_t)98, (uint8_t)181, (uint8_t)143, (uint8_t)254, (uint8_t)232, (uint8_t)54, (uint8_t)214, (uint8_t)204, (uint8_t)40, (uint8_t)29, (uint8_t)90, (uint8_t)26, (uint8_t)89, (uint8_t)78, (uint8_t)7};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)106, (uint8_t)15, (uint8_t)22, (uint8_t)42, (uint8_t)39, (uint8_t)141, (uint8_t)5, (uint8_t)23, (uint8_t)194, (uint8_t)24, (uint8_t)76, (uint8_t)114, (uint8_t)253, (uint8_t)69, (uint8_t)183, (uint8_t)212, (uint8_t)2, (uint8_t)157, (uint8_t)67, (uint8_t)141, (uint8_t)82, (uint8_t)93, (uint8_t)100, (uint8_t)198, (uint8_t)3, (uint8_t)130, (uint8_t)238, (uint8_t)186, (uint8_t)176, (uint8_t)249, (uint8_t)0, (uint8_t)106, (uint8_t)107, (uint8_t)255, (uint8_t)134, (uint8_t)211, (uint8_t)114, (uint8_t)204, (uint8_t)255, (uint8_t)197, (uint8_t)87, (uint8_t)239, (uint8_t)96, (uint8_t)61, (uint8_t)186, (uint8_t)112, (uint8_t)225, (uint8_t)84, (uint8_t)176, (uint8_t)88, (uint8_t)156, (uint8_t)226, (uint8_t)250, (uint8_t)96, (uint8_t)199, (uint8_t)111, (uint8_t)52, (uint8_t)182, (uint8_t)213, (uint8_t)11, (uint8_t)197, (uint8_t)155, (uint8_t)18, (uint8_t)210, (uint8_t)203, (uint8_t)70, (uint8_t)175, (uint8_t)13, (uint8_t)113, (uint8_t)50, (uint8_t)188, (uint8_t)227, (uint8_t)29, (uint8_t)102, (uint8_t)57, (uint8_t)61, (uint8_t)88, (uint8_t)6, (uint8_t)61, (uint8_t)222, (uint8_t)182, (uint8_t)106, (uint8_t)184, (uint8_t)240, (uint8_t)236, (uint8_t)55, (uint8_t)175, (uint8_t)88, (uint8_t)146, (uint8_t)158, (uint8_t)226, (uint8_t)111, (uint8_t)108, (uint8_t)136, (uint8_t)212, (uint8_t)99, (uint8_t)187, (uint8_t)104, (uint8_t)136, (uint8_t)232, (uint8_t)180, (uint8_t)211, (uint8_t)8, (uint8_t)65, (uint8_t)64, (uint8_t)175, (uint8_t)94, (uint8_t)165, (uint8_t)196, (uint8_t)87, (uint8_t)173, (uint8_t)117, (uint8_t)242, (uint8_t)223, (uint8_t)17, (uint8_t)140, (uint8_t)202, (uint8_t)119, (uint8_t)53, (uint8_t)103};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_diff_SET((float)1.5244773E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)176643694L, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)11114, PH.base.pack) ;
        p143_press_abs_SET((float) -2.025529E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FOLLOW_TARGET_144(), &PH);
        {
            float acc[] =  {-1.1398464E38F, -1.04725E38F, 5.476229E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-2.0884544E38F, 1.0277152E38F, -1.6944959E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {2.7666785E38F, 2.613078E38F, 2.9254336E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)8913478194104340227L, PH.base.pack) ;
        {
            float rates[] =  {1.6436897E38F, 9.618559E37F, 2.0776343E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -93181125, PH.base.pack) ;
        p144_alt_SET((float)4.734789E37F, PH.base.pack) ;
        p144_lon_SET((int32_t) -1533572787, PH.base.pack) ;
        {
            float attitude_q[] =  {8.152563E37F, 1.2902562E38F, 2.2670308E38F, 2.5008503E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)5336908620676108773L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_pitch_rate_SET((float)2.0662802E38F, PH.base.pack) ;
        p146_airspeed_SET((float)6.3650836E37F, PH.base.pack) ;
        p146_x_vel_SET((float) -6.01052E37F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)436041890818856602L, PH.base.pack) ;
        p146_z_vel_SET((float)1.0011744E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {3.7857574E37F, 5.81117E37F, 8.600664E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_pos_SET((float)7.675606E37F, PH.base.pack) ;
        p146_z_pos_SET((float) -2.123568E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -2.301271E38F, PH.base.pack) ;
        p146_roll_rate_SET((float) -2.6390595E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)3.3781014E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-3.0221038E38F, 5.3883916E36F, -2.8420856E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_y_acc_SET((float) -1.3680707E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -3.354663E38F, PH.base.pack) ;
        p146_y_pos_SET((float)3.2602722E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -1.6775835E38F, PH.base.pack) ;
        {
            float q[] =  {2.327465E37F, 3.0296444E37F, 1.5844813E38F, 1.9787612E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t)1831234869, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -18418, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)58158, (uint16_t)41723, (uint16_t)57296, (uint16_t)54563, (uint16_t)26403, (uint16_t)9528, (uint16_t)30931, (uint16_t)58487, (uint16_t)3320, (uint16_t)7773};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_energy_consumed_SET((int32_t)1504423173, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)88, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -18599, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_product_id_SET((uint16_t)(uint16_t)29248, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)3438114637L, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)153, (uint8_t)219, (uint8_t)97, (uint8_t)8, (uint8_t)79, (uint8_t)162, (uint8_t)69, (uint8_t)153, (uint8_t)176, (uint8_t)232, (uint8_t)21, (uint8_t)242, (uint8_t)109, (uint8_t)185, (uint8_t)39, (uint8_t)200, (uint8_t)95, (uint8_t)241};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_uid_SET((uint64_t)2822070701982720481L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)137, (uint8_t)204, (uint8_t)13, (uint8_t)140, (uint8_t)27, (uint8_t)121, (uint8_t)194, (uint8_t)234};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)1331408180L, PH.base.pack) ;
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION), PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)209, (uint8_t)48, (uint8_t)201, (uint8_t)23, (uint8_t)213, (uint8_t)30, (uint8_t)65, (uint8_t)118};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t flight_custom_version[] =  {(uint8_t)125, (uint8_t)229, (uint8_t)131, (uint8_t)186, (uint8_t)102, (uint8_t)76, (uint8_t)18, (uint8_t)236};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)44721, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)1470095165L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)2971220480L, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p149_angle_y_SET((float) -2.1669767E38F, PH.base.pack) ;
        p149_size_y_SET((float) -1.6460075E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_size_x_SET((float) -1.3106837E38F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)200, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p149_angle_x_SET((float) -2.4148075E38F, PH.base.pack) ;
        p149_z_SET((float) -7.4046975E37F, &PH) ;
        p149_distance_SET((float) -1.0075667E38F, PH.base.pack) ;
        {
            float q[] =  {-3.9778678E37F, 2.0180464E38F, -4.9996846E36F, -3.8511377E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_time_usec_SET((uint64_t)1775487527632103847L, PH.base.pack) ;
        p149_y_SET((float) -3.069645E38F, &PH) ;
        p149_x_SET((float) -1.3803584E38F, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_vel_ratio_SET((float) -2.1365245E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)1.8470552E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -3.2853072E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)3.0303612E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)2.917916E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -9.557742E37F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)7.7758854E37F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -4.1829126E37F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL), PH.base.pack) ;
        p230_time_usec_SET((uint64_t)5287178835736150153L, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float) -3.0402711E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)6.7785214E36F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -1.6230908E38F, PH.base.pack) ;
        p231_wind_z_SET((float)1.8237474E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.982345E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -2.8806591E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -3.0065384E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)8117871042252918916L, PH.base.pack) ;
        p231_wind_alt_SET((float)4.0462925E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_lat_SET((int32_t) -762120614, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)6796294275446939390L, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2920580970L, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP), PH.base.pack) ;
        p232_vert_accuracy_SET((float)5.220543E37F, PH.base.pack) ;
        p232_alt_SET((float) -5.1515607E37F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p232_speed_accuracy_SET((float)1.4037119E38F, PH.base.pack) ;
        p232_vn_SET((float) -6.5181656E37F, PH.base.pack) ;
        p232_ve_SET((float) -1.1895552E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)55463, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p232_hdop_SET((float)1.4930187E37F, PH.base.pack) ;
        p232_vdop_SET((float)2.2910862E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)3.2609186E38F, PH.base.pack) ;
        p232_lon_SET((int32_t)101951546, PH.base.pack) ;
        p232_vd_SET((float)1.6291726E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)40, (uint8_t)141, (uint8_t)137, (uint8_t)255, (uint8_t)198, (uint8_t)219, (uint8_t)255, (uint8_t)68, (uint8_t)45, (uint8_t)189, (uint8_t)124, (uint8_t)0, (uint8_t)211, (uint8_t)8, (uint8_t)98, (uint8_t)244, (uint8_t)133, (uint8_t)154, (uint8_t)80, (uint8_t)89, (uint8_t)62, (uint8_t)31, (uint8_t)70, (uint8_t)2, (uint8_t)164, (uint8_t)167, (uint8_t)95, (uint8_t)107, (uint8_t)244, (uint8_t)140, (uint8_t)150, (uint8_t)152, (uint8_t)234, (uint8_t)121, (uint8_t)134, (uint8_t)252, (uint8_t)13, (uint8_t)77, (uint8_t)106, (uint8_t)208, (uint8_t)64, (uint8_t)91, (uint8_t)171, (uint8_t)207, (uint8_t)200, (uint8_t)116, (uint8_t)184, (uint8_t)222, (uint8_t)15, (uint8_t)119, (uint8_t)208, (uint8_t)87, (uint8_t)222, (uint8_t)40, (uint8_t)227, (uint8_t)13, (uint8_t)25, (uint8_t)94, (uint8_t)221, (uint8_t)21, (uint8_t)204, (uint8_t)161, (uint8_t)221, (uint8_t)253, (uint8_t)136, (uint8_t)146, (uint8_t)251, (uint8_t)92, (uint8_t)148, (uint8_t)135, (uint8_t)204, (uint8_t)79, (uint8_t)250, (uint8_t)132, (uint8_t)116, (uint8_t)101, (uint8_t)33, (uint8_t)59, (uint8_t)195, (uint8_t)205, (uint8_t)4, (uint8_t)56, (uint8_t)166, (uint8_t)139, (uint8_t)58, (uint8_t)97, (uint8_t)238, (uint8_t)211, (uint8_t)223, (uint8_t)217, (uint8_t)182, (uint8_t)74, (uint8_t)37, (uint8_t)221, (uint8_t)78, (uint8_t)50, (uint8_t)169, (uint8_t)179, (uint8_t)203, (uint8_t)181, (uint8_t)61, (uint8_t)2, (uint8_t)89, (uint8_t)7, (uint8_t)159, (uint8_t)117, (uint8_t)205, (uint8_t)237, (uint8_t)252, (uint8_t)108, (uint8_t)192, (uint8_t)114, (uint8_t)155, (uint8_t)209, (uint8_t)163, (uint8_t)101, (uint8_t)172, (uint8_t)112, (uint8_t)17, (uint8_t)221, (uint8_t)201, (uint8_t)254, (uint8_t)83, (uint8_t)75, (uint8_t)51, (uint8_t)117, (uint8_t)26, (uint8_t)89, (uint8_t)40, (uint8_t)74, (uint8_t)251, (uint8_t)26, (uint8_t)83, (uint8_t)223, (uint8_t)168, (uint8_t)116, (uint8_t)179, (uint8_t)100, (uint8_t)111, (uint8_t)62, (uint8_t)170, (uint8_t)193, (uint8_t)83, (uint8_t)153, (uint8_t)110, (uint8_t)158, (uint8_t)220, (uint8_t)146, (uint8_t)78, (uint8_t)23, (uint8_t)240, (uint8_t)85, (uint8_t)219, (uint8_t)224, (uint8_t)253, (uint8_t)238, (uint8_t)14, (uint8_t)7, (uint8_t)82, (uint8_t)109, (uint8_t)181, (uint8_t)66, (uint8_t)147, (uint8_t)198, (uint8_t)0, (uint8_t)166, (uint8_t)146, (uint8_t)156, (uint8_t)116, (uint8_t)25, (uint8_t)121, (uint8_t)88, (uint8_t)12, (uint8_t)1, (uint8_t)22, (uint8_t)169, (uint8_t)180, (uint8_t)27, (uint8_t)222, (uint8_t)32};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_airspeed_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)6019, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -16936, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)34, PH.base.pack) ;
        p234_latitude_SET((int32_t)545338634, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)38, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)59, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -82, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -17302, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)35800, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)818, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)11793, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1408138842, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)2843085041L, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -13404, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_clipping_2_SET((uint32_t)2384852834L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.8880999E38F, PH.base.pack) ;
        p241_vibration_x_SET((float) -2.1498667E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1936171052L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)88758116823500243L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)885330228L, PH.base.pack) ;
        p241_vibration_z_SET((float) -2.0747193E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_approach_z_SET((float) -2.332299E38F, PH.base.pack) ;
        p242_x_SET((float) -2.6161317E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -3.4078446E37F, PH.base.pack) ;
        {
            float q[] =  {-3.1711195E38F, 3.1123542E38F, -2.6607583E37F, -3.2880774E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_time_usec_SET((uint64_t)5816692232829493303L, &PH) ;
        p242_latitude_SET((int32_t) -362013497, PH.base.pack) ;
        p242_approach_x_SET((float) -3.188299E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t)1407934925, PH.base.pack) ;
        p242_y_SET((float)1.4482789E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1984896424, PH.base.pack) ;
        p242_z_SET((float)8.927314E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_y_SET((float)1.7741673E38F, PH.base.pack) ;
        p243_approach_z_SET((float) -8.928275E37F, PH.base.pack) ;
        p243_latitude_SET((int32_t)973801516, PH.base.pack) ;
        p243_x_SET((float) -2.003202E38F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)7136118091644479318L, &PH) ;
        p243_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p243_altitude_SET((int32_t) -1454459877, PH.base.pack) ;
        p243_approach_x_SET((float) -5.102158E37F, PH.base.pack) ;
        p243_y_SET((float) -1.1093138E38F, PH.base.pack) ;
        {
            float q[] =  {-3.0992475E38F, 2.0203825E38F, 1.3991387E38F, 2.6735978E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float)3.1830455E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -855408304, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1276593059, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)13097, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_ver_velocity_SET((int16_t)(int16_t) -16645, PH.base.pack) ;
        p246_lat_SET((int32_t) -1282048658, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)64833, PH.base.pack) ;
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS), PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3412688908L, PH.base.pack) ;
        p246_lon_SET((int32_t)2076718877, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)56653, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_EMERGENCY_SURFACE, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        {
            char16_t* callsign = u"jvzKyrx";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_heading_SET((uint16_t)(uint16_t)10442, PH.base.pack) ;
        p246_altitude_SET((int32_t) -607948576, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_time_to_minimum_delta_SET((float) -3.3542013E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.4761509E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
        p247_threat_level_SET((e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW), PH.base.pack) ;
        p247_id_SET((uint32_t)2121070195L, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.3423435E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)18894, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)170, (uint8_t)156, (uint8_t)242, (uint8_t)136, (uint8_t)188, (uint8_t)162, (uint8_t)16, (uint8_t)200, (uint8_t)160, (uint8_t)188, (uint8_t)232, (uint8_t)75, (uint8_t)64, (uint8_t)65, (uint8_t)224, (uint8_t)195, (uint8_t)243, (uint8_t)226, (uint8_t)136, (uint8_t)83, (uint8_t)156, (uint8_t)178, (uint8_t)149, (uint8_t)53, (uint8_t)63, (uint8_t)8, (uint8_t)208, (uint8_t)74, (uint8_t)81, (uint8_t)241, (uint8_t)42, (uint8_t)217, (uint8_t)77, (uint8_t)135, (uint8_t)108, (uint8_t)94, (uint8_t)107, (uint8_t)48, (uint8_t)104, (uint8_t)142, (uint8_t)138, (uint8_t)221, (uint8_t)71, (uint8_t)33, (uint8_t)108, (uint8_t)82, (uint8_t)20, (uint8_t)10, (uint8_t)223, (uint8_t)57, (uint8_t)3, (uint8_t)94, (uint8_t)117, (uint8_t)71, (uint8_t)241, (uint8_t)136, (uint8_t)139, (uint8_t)118, (uint8_t)152, (uint8_t)74, (uint8_t)158, (uint8_t)252, (uint8_t)185, (uint8_t)20, (uint8_t)125, (uint8_t)228, (uint8_t)122, (uint8_t)129, (uint8_t)78, (uint8_t)228, (uint8_t)150, (uint8_t)83, (uint8_t)234, (uint8_t)58, (uint8_t)180, (uint8_t)80, (uint8_t)35, (uint8_t)194, (uint8_t)137, (uint8_t)240, (uint8_t)73, (uint8_t)147, (uint8_t)137, (uint8_t)54, (uint8_t)88, (uint8_t)160, (uint8_t)28, (uint8_t)250, (uint8_t)139, (uint8_t)38, (uint8_t)41, (uint8_t)123, (uint8_t)168, (uint8_t)87, (uint8_t)61, (uint8_t)50, (uint8_t)194, (uint8_t)49, (uint8_t)171, (uint8_t)100, (uint8_t)0, (uint8_t)191, (uint8_t)69, (uint8_t)48, (uint8_t)24, (uint8_t)23, (uint8_t)3, (uint8_t)91, (uint8_t)248, (uint8_t)78, (uint8_t)63, (uint8_t)181, (uint8_t)81, (uint8_t)49, (uint8_t)121, (uint8_t)173, (uint8_t)103, (uint8_t)226, (uint8_t)161, (uint8_t)169, (uint8_t)58, (uint8_t)179, (uint8_t)27, (uint8_t)133, (uint8_t)8, (uint8_t)251, (uint8_t)166, (uint8_t)252, (uint8_t)116, (uint8_t)244, (uint8_t)142, (uint8_t)96, (uint8_t)35, (uint8_t)6, (uint8_t)239, (uint8_t)63, (uint8_t)165, (uint8_t)66, (uint8_t)209, (uint8_t)47, (uint8_t)65, (uint8_t)172, (uint8_t)69, (uint8_t)230, (uint8_t)113, (uint8_t)62, (uint8_t)167, (uint8_t)252, (uint8_t)138, (uint8_t)25, (uint8_t)55, (uint8_t)236, (uint8_t)218, (uint8_t)49, (uint8_t)57, (uint8_t)180, (uint8_t)76, (uint8_t)235, (uint8_t)252, (uint8_t)250, (uint8_t)71, (uint8_t)182, (uint8_t)15, (uint8_t)98, (uint8_t)64, (uint8_t)235, (uint8_t)195, (uint8_t)219, (uint8_t)31, (uint8_t)55, (uint8_t)131, (uint8_t)7, (uint8_t)181, (uint8_t)117, (uint8_t)170, (uint8_t)26, (uint8_t)230, (uint8_t)214, (uint8_t)89, (uint8_t)159, (uint8_t)62, (uint8_t)221, (uint8_t)49, (uint8_t)231, (uint8_t)35, (uint8_t)246, (uint8_t)78, (uint8_t)74, (uint8_t)128, (uint8_t)220, (uint8_t)198, (uint8_t)132, (uint8_t)91, (uint8_t)87, (uint8_t)38, (uint8_t)101, (uint8_t)15, (uint8_t)43, (uint8_t)38, (uint8_t)130, (uint8_t)213, (uint8_t)115, (uint8_t)125, (uint8_t)141, (uint8_t)63, (uint8_t)212, (uint8_t)217, (uint8_t)244, (uint8_t)88, (uint8_t)8, (uint8_t)174, (uint8_t)62, (uint8_t)16, (uint8_t)129, (uint8_t)129, (uint8_t)164, (uint8_t)255, (uint8_t)210, (uint8_t)24, (uint8_t)35, (uint8_t)53, (uint8_t)41, (uint8_t)76, (uint8_t)84, (uint8_t)146, (uint8_t)138, (uint8_t)191, (uint8_t)76, (uint8_t)198, (uint8_t)221, (uint8_t)238, (uint8_t)75, (uint8_t)160, (uint8_t)240, (uint8_t)120, (uint8_t)56, (uint8_t)51, (uint8_t)139, (uint8_t)186, (uint8_t)82, (uint8_t)139, (uint8_t)25, (uint8_t)130, (uint8_t)106, (uint8_t)230, (uint8_t)148, (uint8_t)19, (uint8_t)178, (uint8_t)25};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t) -76, (int8_t)36, (int8_t)94, (int8_t) -44, (int8_t)55, (int8_t) -103, (int8_t)96, (int8_t) -40, (int8_t)11, (int8_t)88, (int8_t)118, (int8_t) -72, (int8_t) -110, (int8_t) -117, (int8_t) -103, (int8_t) -77, (int8_t) -2, (int8_t) -108, (int8_t) -38, (int8_t)23, (int8_t)0, (int8_t)84, (int8_t) -39, (int8_t)25, (int8_t) -120, (int8_t)104, (int8_t)78, (int8_t)53, (int8_t)47, (int8_t) -2, (int8_t)28, (int8_t) -53};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)51563, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        {
            char16_t* name = u"Fxvy";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)2718307025972945720L, PH.base.pack) ;
        p250_x_SET((float)3.1479594E38F, PH.base.pack) ;
        p250_z_SET((float)2.1132204E38F, PH.base.pack) ;
        p250_y_SET((float)1.3338664E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"pde";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float)2.190758E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)4094056152L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)325996248, PH.base.pack) ;
        {
            char16_t* name = u"mvNplju";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)3058828977L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
        {
            char16_t* text = u"IldqvsepcKonfAsgPsTreVgMxzHuygMfvwsuundkjokqfqtYt";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float)1.2999273E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1064951605L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        {
            uint8_t secret_key[] =  {(uint8_t)136, (uint8_t)85, (uint8_t)143, (uint8_t)33, (uint8_t)139, (uint8_t)42, (uint8_t)148, (uint8_t)42, (uint8_t)142, (uint8_t)238, (uint8_t)246, (uint8_t)85, (uint8_t)19, (uint8_t)177, (uint8_t)138, (uint8_t)102, (uint8_t)195, (uint8_t)89, (uint8_t)63, (uint8_t)191, (uint8_t)129, (uint8_t)171, (uint8_t)67, (uint8_t)42, (uint8_t)4, (uint8_t)5, (uint8_t)180, (uint8_t)40, (uint8_t)107, (uint8_t)115, (uint8_t)157, (uint8_t)115};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)7997221146474400308L, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1132642732L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)2740310731L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        {
            char16_t* tune = u"ruhfpEbhdHaqP";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES), PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)221, (uint8_t)54, (uint8_t)68, (uint8_t)76, (uint8_t)66, (uint8_t)158, (uint8_t)88, (uint8_t)24, (uint8_t)244, (uint8_t)48, (uint8_t)97, (uint8_t)31, (uint8_t)233, (uint8_t)200, (uint8_t)83, (uint8_t)102, (uint8_t)221, (uint8_t)165, (uint8_t)173, (uint8_t)94, (uint8_t)191, (uint8_t)49, (uint8_t)206, (uint8_t)213, (uint8_t)80, (uint8_t)235, (uint8_t)215, (uint8_t)72, (uint8_t)131, (uint8_t)45, (uint8_t)98, (uint8_t)102};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)1.6835256E38F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)18014, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)17445, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)1817569644L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)10469, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"pdatowkyGvnebzhqmuyuDgitzvoalgg";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_focal_length_SET((float)1.8539804E38F, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -1.9260292E38F, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1324451707L, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)68, (uint8_t)84, (uint8_t)213, (uint8_t)9, (uint8_t)73, (uint8_t)249, (uint8_t)80, (uint8_t)97, (uint8_t)141, (uint8_t)171, (uint8_t)235, (uint8_t)186, (uint8_t)98, (uint8_t)228, (uint8_t)94, (uint8_t)69, (uint8_t)91, (uint8_t)201, (uint8_t)160, (uint8_t)128, (uint8_t)167, (uint8_t)0, (uint8_t)117, (uint8_t)204, (uint8_t)199, (uint8_t)95, (uint8_t)113, (uint8_t)58, (uint8_t)155, (uint8_t)239, (uint8_t)108, (uint8_t)176};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)2251935856L, PH.base.pack) ;
        p260_mode_id_SET((e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY), PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_id_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p261_available_capacity_SET((float)1.9025022E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p261_read_speed_SET((float) -2.972034E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2466477652L, PH.base.pack) ;
        p261_total_capacity_SET((float)2.3419926E38F, PH.base.pack) ;
        p261_write_speed_SET((float)2.8579588E38F, PH.base.pack) ;
        p261_used_capacity_SET((float) -2.0992014E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)1697281424L, PH.base.pack) ;
        p262_image_interval_SET((float) -3.0753607E38F, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.137561E37F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)4230862204L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_image_index_SET((int32_t) -190829341, PH.base.pack) ;
        {
            char16_t* file_url = u"zbsmmxnimrtupoqvmptjlvuqtghkyqlurubavjxcnrgcsxoodnBkejazt";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_lon_SET((int32_t)1237042074, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -73, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)2298377708204336141L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -22757982, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)2394441637L, PH.base.pack) ;
        p263_alt_SET((int32_t) -112134685, PH.base.pack) ;
        {
            float q[] =  {-1.3732799E38F, -3.119649E36F, 2.2551616E38F, 3.0849064E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p263_lat_SET((int32_t) -141458465, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)2249161870L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)6164808241692094862L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)7335966714418326034L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)7244699179574670238L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float) -1.6608219E38F, PH.base.pack) ;
        p265_roll_SET((float) -1.3081709E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1470190723L, PH.base.pack) ;
        p265_yaw_SET((float) -2.3985385E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)18973, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)179, (uint8_t)231, (uint8_t)51, (uint8_t)229, (uint8_t)68, (uint8_t)229, (uint8_t)65, (uint8_t)15, (uint8_t)94, (uint8_t)251, (uint8_t)132, (uint8_t)245, (uint8_t)32, (uint8_t)213, (uint8_t)156, (uint8_t)113, (uint8_t)60, (uint8_t)212, (uint8_t)196, (uint8_t)16, (uint8_t)101, (uint8_t)183, (uint8_t)249, (uint8_t)219, (uint8_t)24, (uint8_t)192, (uint8_t)60, (uint8_t)105, (uint8_t)200, (uint8_t)93, (uint8_t)25, (uint8_t)218, (uint8_t)246, (uint8_t)102, (uint8_t)212, (uint8_t)42, (uint8_t)5, (uint8_t)199, (uint8_t)152, (uint8_t)131, (uint8_t)234, (uint8_t)105, (uint8_t)8, (uint8_t)88, (uint8_t)142, (uint8_t)188, (uint8_t)244, (uint8_t)32, (uint8_t)79, (uint8_t)8, (uint8_t)12, (uint8_t)240, (uint8_t)68, (uint8_t)129, (uint8_t)85, (uint8_t)188, (uint8_t)79, (uint8_t)144, (uint8_t)145, (uint8_t)227, (uint8_t)210, (uint8_t)55, (uint8_t)53, (uint8_t)6, (uint8_t)241, (uint8_t)240, (uint8_t)253, (uint8_t)7, (uint8_t)119, (uint8_t)53, (uint8_t)236, (uint8_t)91, (uint8_t)49, (uint8_t)26, (uint8_t)125, (uint8_t)205, (uint8_t)130, (uint8_t)4, (uint8_t)20, (uint8_t)214, (uint8_t)213, (uint8_t)9, (uint8_t)166, (uint8_t)40, (uint8_t)243, (uint8_t)253, (uint8_t)9, (uint8_t)139, (uint8_t)6, (uint8_t)60, (uint8_t)75, (uint8_t)246, (uint8_t)183, (uint8_t)52, (uint8_t)168, (uint8_t)104, (uint8_t)29, (uint8_t)70, (uint8_t)94, (uint8_t)94, (uint8_t)77, (uint8_t)18, (uint8_t)243, (uint8_t)128, (uint8_t)168, (uint8_t)213, (uint8_t)236, (uint8_t)178, (uint8_t)56, (uint8_t)65, (uint8_t)241, (uint8_t)48, (uint8_t)148, (uint8_t)75, (uint8_t)6, (uint8_t)1, (uint8_t)147, (uint8_t)117, (uint8_t)207, (uint8_t)177, (uint8_t)176, (uint8_t)145, (uint8_t)95, (uint8_t)168, (uint8_t)232, (uint8_t)106, (uint8_t)3, (uint8_t)203, (uint8_t)3, (uint8_t)3, (uint8_t)43, (uint8_t)208, (uint8_t)252, (uint8_t)153, (uint8_t)215, (uint8_t)171, (uint8_t)188, (uint8_t)124, (uint8_t)217, (uint8_t)172, (uint8_t)84, (uint8_t)82, (uint8_t)147, (uint8_t)43, (uint8_t)175, (uint8_t)173, (uint8_t)8, (uint8_t)143, (uint8_t)21, (uint8_t)141, (uint8_t)83, (uint8_t)67, (uint8_t)60, (uint8_t)19, (uint8_t)181, (uint8_t)240, (uint8_t)151, (uint8_t)72, (uint8_t)247, (uint8_t)64, (uint8_t)244, (uint8_t)173, (uint8_t)5, (uint8_t)248, (uint8_t)59, (uint8_t)16, (uint8_t)142, (uint8_t)54, (uint8_t)69, (uint8_t)31, (uint8_t)33, (uint8_t)73, (uint8_t)143, (uint8_t)161, (uint8_t)211, (uint8_t)61, (uint8_t)102, (uint8_t)109, (uint8_t)0, (uint8_t)45, (uint8_t)85, (uint8_t)122, (uint8_t)243, (uint8_t)2, (uint8_t)85, (uint8_t)30, (uint8_t)55, (uint8_t)161, (uint8_t)115, (uint8_t)221, (uint8_t)114, (uint8_t)75, (uint8_t)18, (uint8_t)197, (uint8_t)248, (uint8_t)250, (uint8_t)25, (uint8_t)196, (uint8_t)93, (uint8_t)75, (uint8_t)169, (uint8_t)241, (uint8_t)87, (uint8_t)34, (uint8_t)253, (uint8_t)33, (uint8_t)242, (uint8_t)224, (uint8_t)108, (uint8_t)184, (uint8_t)170, (uint8_t)57, (uint8_t)21, (uint8_t)171, (uint8_t)138, (uint8_t)52, (uint8_t)134, (uint8_t)183, (uint8_t)141, (uint8_t)177, (uint8_t)168, (uint8_t)116, (uint8_t)21, (uint8_t)126, (uint8_t)76, (uint8_t)79, (uint8_t)104, (uint8_t)90, (uint8_t)129, (uint8_t)249, (uint8_t)25, (uint8_t)140, (uint8_t)74, (uint8_t)190, (uint8_t)176, (uint8_t)198, (uint8_t)232, (uint8_t)245, (uint8_t)129, (uint8_t)170, (uint8_t)106, (uint8_t)131, (uint8_t)44, (uint8_t)140, (uint8_t)208, (uint8_t)111, (uint8_t)151, (uint8_t)221, (uint8_t)42};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)7, (uint8_t)125, (uint8_t)105, (uint8_t)61, (uint8_t)161, (uint8_t)244, (uint8_t)201, (uint8_t)91, (uint8_t)132, (uint8_t)182, (uint8_t)75, (uint8_t)229, (uint8_t)175, (uint8_t)216, (uint8_t)155, (uint8_t)186, (uint8_t)162, (uint8_t)128, (uint8_t)56, (uint8_t)15, (uint8_t)132, (uint8_t)134, (uint8_t)135, (uint8_t)94, (uint8_t)180, (uint8_t)82, (uint8_t)232, (uint8_t)4, (uint8_t)119, (uint8_t)180, (uint8_t)106, (uint8_t)86, (uint8_t)11, (uint8_t)41, (uint8_t)47, (uint8_t)139, (uint8_t)192, (uint8_t)178, (uint8_t)168, (uint8_t)168, (uint8_t)60, (uint8_t)197, (uint8_t)242, (uint8_t)1, (uint8_t)73, (uint8_t)43, (uint8_t)38, (uint8_t)28, (uint8_t)26, (uint8_t)251, (uint8_t)180, (uint8_t)137, (uint8_t)83, (uint8_t)141, (uint8_t)15, (uint8_t)1, (uint8_t)50, (uint8_t)110, (uint8_t)33, (uint8_t)161, (uint8_t)140, (uint8_t)116, (uint8_t)23, (uint8_t)147, (uint8_t)164, (uint8_t)138, (uint8_t)66, (uint8_t)8, (uint8_t)26, (uint8_t)161, (uint8_t)115, (uint8_t)226, (uint8_t)150, (uint8_t)49, (uint8_t)248, (uint8_t)210, (uint8_t)3, (uint8_t)205, (uint8_t)111, (uint8_t)9, (uint8_t)68, (uint8_t)101, (uint8_t)5, (uint8_t)140, (uint8_t)152, (uint8_t)156, (uint8_t)125, (uint8_t)57, (uint8_t)163, (uint8_t)250, (uint8_t)159, (uint8_t)179, (uint8_t)252, (uint8_t)114, (uint8_t)76, (uint8_t)211, (uint8_t)10, (uint8_t)185, (uint8_t)10, (uint8_t)17, (uint8_t)51, (uint8_t)70, (uint8_t)109, (uint8_t)230, (uint8_t)165, (uint8_t)84, (uint8_t)240, (uint8_t)176, (uint8_t)162, (uint8_t)77, (uint8_t)161, (uint8_t)20, (uint8_t)22, (uint8_t)246, (uint8_t)213, (uint8_t)92, (uint8_t)192, (uint8_t)128, (uint8_t)5, (uint8_t)199, (uint8_t)241, (uint8_t)237, (uint8_t)40, (uint8_t)129, (uint8_t)173, (uint8_t)17, (uint8_t)174, (uint8_t)123, (uint8_t)69, (uint8_t)236, (uint8_t)158, (uint8_t)144, (uint8_t)6, (uint8_t)15, (uint8_t)92, (uint8_t)153, (uint8_t)57, (uint8_t)135, (uint8_t)21, (uint8_t)56, (uint8_t)201, (uint8_t)65, (uint8_t)46, (uint8_t)177, (uint8_t)250, (uint8_t)56, (uint8_t)86, (uint8_t)112, (uint8_t)73, (uint8_t)83, (uint8_t)139, (uint8_t)159, (uint8_t)165, (uint8_t)254, (uint8_t)254, (uint8_t)29, (uint8_t)163, (uint8_t)229, (uint8_t)103, (uint8_t)77, (uint8_t)127, (uint8_t)253, (uint8_t)32, (uint8_t)36, (uint8_t)48, (uint8_t)193, (uint8_t)127, (uint8_t)192, (uint8_t)159, (uint8_t)136, (uint8_t)22, (uint8_t)151, (uint8_t)157, (uint8_t)162, (uint8_t)128, (uint8_t)191, (uint8_t)155, (uint8_t)2, (uint8_t)74, (uint8_t)221, (uint8_t)5, (uint8_t)65, (uint8_t)69, (uint8_t)144, (uint8_t)226, (uint8_t)165, (uint8_t)38, (uint8_t)202, (uint8_t)221, (uint8_t)136, (uint8_t)142, (uint8_t)29, (uint8_t)2, (uint8_t)30, (uint8_t)30, (uint8_t)133, (uint8_t)50, (uint8_t)82, (uint8_t)4, (uint8_t)62, (uint8_t)8, (uint8_t)91, (uint8_t)99, (uint8_t)210, (uint8_t)147, (uint8_t)48, (uint8_t)187, (uint8_t)69, (uint8_t)243, (uint8_t)199, (uint8_t)67, (uint8_t)34, (uint8_t)28, (uint8_t)99, (uint8_t)117, (uint8_t)234, (uint8_t)199, (uint8_t)38, (uint8_t)39, (uint8_t)60, (uint8_t)93, (uint8_t)134, (uint8_t)224, (uint8_t)87, (uint8_t)174, (uint8_t)138, (uint8_t)210, (uint8_t)62, (uint8_t)5, (uint8_t)31, (uint8_t)161, (uint8_t)34, (uint8_t)110, (uint8_t)167, (uint8_t)225, (uint8_t)247, (uint8_t)165, (uint8_t)35, (uint8_t)1, (uint8_t)115, (uint8_t)210, (uint8_t)167, (uint8_t)177, (uint8_t)158, (uint8_t)30, (uint8_t)118, (uint8_t)151, (uint8_t)61, (uint8_t)2};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)44013, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)56352, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_camera_id_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)63140, PH.base.pack) ;
        {
            char16_t* uri = u"flnjvzezuhddnyekLPyulrdqpzgupfczsDGlqzdkpdaznlqlxzjjSzYniizzgfmhfjdkypqfkbEqvamgzgabjcqwfexsngibgkeDluphXuzupqgZkhplalpwyofvganMQtbypfbimyjumqLiosp";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_status_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p269_framerate_SET((float)2.1730022E38F, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1680054574L, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)56060, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)14643, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_bitrate_SET((uint32_t)2649034022L, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)58839, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)63693, PH.base.pack) ;
        p270_framerate_SET((float)3.3637989E38F, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        {
            char16_t* uri = u"gjNixrhbbzuunikuzvrxjunuaQchsvrhsHumegtttngektdilbDttxndjoopwupzfvnkkhu";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_rotation_SET((uint16_t)(uint16_t)44866, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"krzZcihktNhrufPhItxtItbjdnnelzhxtpfftgnowotbkYinudxitogznee";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"hlptjeKdxuDk";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_version_SET((uint16_t)(uint16_t)13503, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)9, (uint8_t)69, (uint8_t)146, (uint8_t)194, (uint8_t)33, (uint8_t)127, (uint8_t)34, (uint8_t)153};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)32071, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)167, (uint8_t)113, (uint8_t)200, (uint8_t)202, (uint8_t)31, (uint8_t)251, (uint8_t)204, (uint8_t)132};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)7415, PH.base.pack) ;
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)7656805574587324619L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1729834692L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)8183, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)8998787099209742211L, PH.base.pack) ;
        {
            char16_t* name = u"xquwtltouZvdpXRywwSopogf";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_vcs_commit_SET((uint32_t)2218605585L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)187, (uint8_t)153, (uint8_t)33, (uint8_t)244, (uint8_t)153, (uint8_t)18, (uint8_t)167, (uint8_t)90, (uint8_t)165, (uint8_t)237, (uint8_t)53, (uint8_t)14, (uint8_t)29, (uint8_t)196, (uint8_t)77, (uint8_t)193};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_uptime_sec_SET((uint32_t)1864135422L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t)21278, PH.base.pack) ;
        {
            char16_t* param_id = u"rrzhv";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)45208, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        {
            char16_t* param_id = u"odvtpvaimyxialA";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)39163, PH.base.pack) ;
        {
            char16_t* param_value = u"uAztmrhiulRuuhpsixwzwfahduxwsfwaaLodhqvjxlumrbrrsdwbmZilpdMikcyvjtomybbmwvwvdqrf";
            p322_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        {
            char16_t* param_id = u"meoO";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"lZbnwsrndvbJvtpskopUgncafqforxqpvorFHrsqarwBbnpmftankbbFiyvaijltovGqqvokJtzetpacaXEedvpplyrgnoojXyxblZirtrlfomqEZcwj";
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
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        {
            char16_t* param_value = u"hncrjdrvnglkbpcwlYtukkfiprddBwkazngajjUocukyesirvpaoffGbfTjmmbgkpxvggctLkeyVmpejovutkgxdbfuIp";
            p324_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"knyvzpgPal";
            p324_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_max_distance_SET((uint16_t)(uint16_t)4806, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)58241, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)38336, (uint16_t)44768, (uint16_t)33457, (uint16_t)30564, (uint16_t)24839, (uint16_t)4701, (uint16_t)19218, (uint16_t)20883, (uint16_t)4387, (uint16_t)44434, (uint16_t)60078, (uint16_t)16338, (uint16_t)251, (uint16_t)10886, (uint16_t)7750, (uint16_t)58009, (uint16_t)60772, (uint16_t)43028, (uint16_t)64898, (uint16_t)55500, (uint16_t)58525, (uint16_t)21857, (uint16_t)42226, (uint16_t)65471, (uint16_t)5051, (uint16_t)26527, (uint16_t)51777, (uint16_t)20932, (uint16_t)3072, (uint16_t)38498, (uint16_t)33704, (uint16_t)26461, (uint16_t)23940, (uint16_t)61891, (uint16_t)2747, (uint16_t)61745, (uint16_t)59979, (uint16_t)47293, (uint16_t)61525, (uint16_t)15311, (uint16_t)41090, (uint16_t)28435, (uint16_t)57613, (uint16_t)37969, (uint16_t)49526, (uint16_t)39896, (uint16_t)14746, (uint16_t)35036, (uint16_t)11846, (uint16_t)30606, (uint16_t)12686, (uint16_t)57765, (uint16_t)4656, (uint16_t)15589, (uint16_t)15104, (uint16_t)41375, (uint16_t)32693, (uint16_t)41810, (uint16_t)4299, (uint16_t)28555, (uint16_t)39729, (uint16_t)21543, (uint16_t)53504, (uint16_t)54054, (uint16_t)46056, (uint16_t)21137, (uint16_t)10431, (uint16_t)41055, (uint16_t)54350, (uint16_t)63474, (uint16_t)12657, (uint16_t)18700};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)7480347028072290336L, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


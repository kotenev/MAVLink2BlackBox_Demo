
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
    return  _en__u(get_bits(data, 40, 4));
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
*	 0 ignore*/
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
*	 (2) Distance in cm (3) Absolute valu*/
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
*	 mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
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
*	 (W) adds to True cours*/
INLINER void p194_magDir_SET(int8_t  src, Pack * dst)
{
    uint8_t * data = dst->data;
    set_bytes((uint8_t)(src), 1, data,  9);
}
/**
*Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
*	 input; N-Data not vali*/
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB);
    assert(p0_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED));
    assert(p0_custom_mode_GET(pack) == (uint32_t)2368201087L);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_EMERGENCY);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_OCTOROTOR);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)118);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_present_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_onboard_control_sensors_enabled_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL));
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)46869);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)16154);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)60444);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)39337);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)40420);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)26520);
    assert(p1_onboard_control_sensors_health_GET(pack) == (e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE));
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)30144);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)62292);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)14);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -2043);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2413121891L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)8727966769136156977L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vz_GET(pack) == (float)2.2426547E37F);
    assert(p3_z_GET(pack) == (float)7.034167E37F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)49702);
    assert(p3_yaw_rate_GET(pack) == (float) -1.9144962E38F);
    assert(p3_yaw_GET(pack) == (float)2.6648942E38F);
    assert(p3_vy_GET(pack) == (float)6.87864E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2385793786L);
    assert(p3_afz_GET(pack) == (float) -3.2760838E38F);
    assert(p3_y_GET(pack) == (float)2.5037756E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p3_afy_GET(pack) == (float)7.7701556E37F);
    assert(p3_afx_GET(pack) == (float) -3.1166658E38F);
    assert(p3_vx_GET(pack) == (float)1.9233574E38F);
    assert(p3_x_GET(pack) == (float) -2.9073579E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p4_seq_GET(pack) == (uint32_t)4283179602L);
    assert(p4_time_usec_GET(pack) == (uint64_t)3443853506419426524L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p5_passkey_LEN(ph) == 15);
    {
        char16_t * exemplary = u"pwuquStmbPyjdDd";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)140);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 5);
    {
        char16_t * exemplary = u"cnama";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)173);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_DISARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)1945958422L);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"aiMhqxveBIq";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)4041);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)219);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)84);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)47943);
    assert(p22_param_value_GET(pack) == (float) -2.8889709E38F);
    assert(p22_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"xkvzxgfbmjrvm";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)8779);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float)1.6132299E38F);
    assert(p23_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"cjcWtrjgbued";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_time_usec_GET(pack) == (uint64_t)1178315162397519084L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1778023212L);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)3622);
    assert(p24_lon_GET(pack) == (int32_t) -28787096);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)126780592);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)63410);
    assert(p24_lat_GET(pack) == (int32_t) -1592012462);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)63893);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1631048640L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)42358);
    assert(p24_alt_GET(pack) == (int32_t)464270403);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2981447407L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)4055985723L);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)172, (uint8_t)234, (uint8_t)234, (uint8_t)201, (uint8_t)103, (uint8_t)118, (uint8_t)171, (uint8_t)210, (uint8_t)193, (uint8_t)147, (uint8_t)2, (uint8_t)73, (uint8_t)98, (uint8_t)3, (uint8_t)202, (uint8_t)65, (uint8_t)244, (uint8_t)221, (uint8_t)229} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)39, (uint8_t)113, (uint8_t)138, (uint8_t)244, (uint8_t)24, (uint8_t)158, (uint8_t)248, (uint8_t)130, (uint8_t)221, (uint8_t)140, (uint8_t)166, (uint8_t)214, (uint8_t)160, (uint8_t)102, (uint8_t)174, (uint8_t)56, (uint8_t)213, (uint8_t)155, (uint8_t)230, (uint8_t)55} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)172);
    {
        uint8_t exemplary[] =  {(uint8_t)24, (uint8_t)5, (uint8_t)219, (uint8_t)133, (uint8_t)4, (uint8_t)41, (uint8_t)177, (uint8_t)253, (uint8_t)204, (uint8_t)40, (uint8_t)77, (uint8_t)90, (uint8_t)142, (uint8_t)226, (uint8_t)201, (uint8_t)192, (uint8_t)11, (uint8_t)71, (uint8_t)54, (uint8_t)211} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)205, (uint8_t)118, (uint8_t)101, (uint8_t)29, (uint8_t)96, (uint8_t)64, (uint8_t)57, (uint8_t)212, (uint8_t)114, (uint8_t)79, (uint8_t)186, (uint8_t)28, (uint8_t)99, (uint8_t)51, (uint8_t)3, (uint8_t)168, (uint8_t)92, (uint8_t)68, (uint8_t)129} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)138, (uint8_t)162, (uint8_t)186, (uint8_t)44, (uint8_t)208, (uint8_t)192, (uint8_t)103, (uint8_t)114, (uint8_t)184, (uint8_t)127, (uint8_t)100, (uint8_t)188, (uint8_t)97, (uint8_t)3, (uint8_t)55, (uint8_t)104, (uint8_t)82, (uint8_t)201, (uint8_t)221, (uint8_t)122} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -24308);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)14481);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3909130814L);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)16967);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)3773);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)27947);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)6403);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -22715);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)23345);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)11123);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -532);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)8690);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -19465);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)7762);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)28330);
    assert(p27_time_usec_GET(pack) == (uint64_t)6014094652102244114L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -9596);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)3396);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)13889);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -112);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)29422);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)5631);
    assert(p28_time_usec_GET(pack) == (uint64_t)133784914142584080L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -7784);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -31493);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float)4.0341173E37F);
    assert(p29_press_diff_GET(pack) == (float) -1.2286197E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)15490);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3725462088L);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_roll_GET(pack) == (float) -1.142686E38F);
    assert(p30_yaw_GET(pack) == (float) -2.76753E38F);
    assert(p30_pitch_GET(pack) == (float)2.1987456E38F);
    assert(p30_rollspeed_GET(pack) == (float)2.4625742E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2254064296L);
    assert(p30_pitchspeed_GET(pack) == (float) -2.0630689E37F);
    assert(p30_yawspeed_GET(pack) == (float) -6.7971907E37F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q4_GET(pack) == (float) -1.0127154E38F);
    assert(p31_yawspeed_GET(pack) == (float)2.4085487E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1288546868L);
    assert(p31_q1_GET(pack) == (float) -1.8553529E38F);
    assert(p31_q2_GET(pack) == (float)1.9868803E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.962446E38F);
    assert(p31_rollspeed_GET(pack) == (float) -2.7907788E38F);
    assert(p31_q3_GET(pack) == (float) -7.386954E35F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vy_GET(pack) == (float) -1.0854351E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)378343899L);
    assert(p32_vx_GET(pack) == (float)1.6574914E38F);
    assert(p32_z_GET(pack) == (float) -1.6259809E38F);
    assert(p32_y_GET(pack) == (float) -3.1594891E38F);
    assert(p32_x_GET(pack) == (float)5.7077936E37F);
    assert(p32_vz_GET(pack) == (float) -1.7410668E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_alt_GET(pack) == (int32_t) -287046797);
    assert(p33_lon_GET(pack) == (int32_t) -522023683);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)16613);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1622366296L);
    assert(p33_lat_GET(pack) == (int32_t)1689833997);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)5963);
    assert(p33_relative_alt_GET(pack) == (int32_t)1117244251);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -14456);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)53299);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -13868);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -26461);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t)27713);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -15509);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -3011);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)27930);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)8942);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)2681418799L);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -32223);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)25210);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)46618);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)5092);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)43864);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)7710);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1446804616L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)180);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)56938);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)41622);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)41854);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)59187);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)27060);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)63355);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)37135);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)22619);
    assert(p36_time_usec_GET(pack) == (uint32_t)3574338594L);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)36671);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)4869);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)51248);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)43014);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)61617);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)24215);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)48168);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)25166);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)35397);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)16343);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -477);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)25638);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)38);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -14457);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -27060);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)80);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_param3_GET(pack) == (float)2.1932655E38F);
    assert(p39_y_GET(pack) == (float) -6.2332327E37F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)15645);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p39_param2_GET(pack) == (float)9.020051E37F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p39_z_GET(pack) == (float)2.4942133E38F);
    assert(p39_param1_GET(pack) == (float) -3.8313776E36F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p39_param4_GET(pack) == (float)1.6215733E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p39_x_GET(pack) == (float) -1.0234654E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)55954);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)24546);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)50014);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)32330);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)33263);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_longitude_GET(pack) == (int32_t) -1246774004);
    assert(p48_altitude_GET(pack) == (int32_t) -1793087864);
    assert(p48_latitude_GET(pack) == (int32_t) -1242466161);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1689196836295157568L);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_longitude_GET(pack) == (int32_t)2013859645);
    assert(p49_altitude_GET(pack) == (int32_t) -954100936);
    assert(p49_time_usec_TRY(ph) == (uint64_t)7834989959824505815L);
    assert(p49_latitude_GET(pack) == (int32_t)563828859);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p50_param_value0_GET(pack) == (float) -3.1648095E36F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p50_scale_GET(pack) == (float)2.1902903E38F);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"m";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p50_param_value_max_GET(pack) == (float) -3.1850624E37F);
    assert(p50_param_value_min_GET(pack) == (float) -2.2529703E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -17932);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)45564);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)110);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1y_GET(pack) == (float) -8.905445E37F);
    assert(p54_p1z_GET(pack) == (float) -2.846768E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p54_p2y_GET(pack) == (float)1.0301685E38F);
    assert(p54_p2x_GET(pack) == (float) -4.1324634E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p54_p2z_GET(pack) == (float)4.277856E37F);
    assert(p54_p1x_GET(pack) == (float)5.0113113E37F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float) -3.3592358E38F);
    assert(p55_p1x_GET(pack) == (float)1.1730949E38F);
    assert(p55_p2x_GET(pack) == (float)2.2583043E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p55_p2y_GET(pack) == (float) -1.4150567E38F);
    assert(p55_p2z_GET(pack) == (float) -1.6477947E38F);
    assert(p55_p1z_GET(pack) == (float) -5.2611797E36F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.1548286E38F, -1.5985517E38F, -8.848566E37F, -7.1824576E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)2.4001287E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)4813734787884381236L);
    {
        float exemplary[] =  {1.2124208E38F, -5.5689716E37F, 2.3494486E38F, 2.159633E38F, -3.0631629E38F, 1.1434328E38F, 2.3380826E38F, 1.366611E38F, -1.0063334E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float)7.56543E37F);
    assert(p61_pitchspeed_GET(pack) == (float) -5.529453E37F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float)1.9578851E38F);
    assert(p62_nav_pitch_GET(pack) == (float)2.6189706E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)34306);
    assert(p62_alt_error_GET(pack) == (float)2.5492253E38F);
    assert(p62_aspd_error_GET(pack) == (float) -1.9997275E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)19373);
    assert(p62_xtrack_error_GET(pack) == (float) -1.3610143E38F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)27309);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vy_GET(pack) == (float)2.9610762E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)3389730451407549780L);
    {
        float exemplary[] =  {-2.6957067E38F, -1.890076E38F, -2.761005E38F, -1.3191308E38F, 3.3401092E38F, -2.317562E38F, -1.7073996E38F, 5.1679326E37F, 3.1205631E38F, -1.7870657E38F, 2.9954099E38F, 2.590948E37F, -1.9539873E38F, -2.0769374E38F, 4.513414E36F, -2.2523906E38F, 2.8590878E38F, -1.8298435E38F, -2.7709826E38F, 1.0812835E38F, 6.3182845E37F, -2.6945915E38F, 2.5032556E38F, 1.157149E38F, 2.3735444E36F, -4.2012829E37F, -8.587707E37F, -1.3866891E38F, -1.0450886E38F, -2.6464273E38F, -2.5753717E38F, -2.0896E38F, 2.50004E38F, -9.945353E37F, -1.6900007E38F, 8.0223953E37F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p63_alt_GET(pack) == (int32_t)2005295807);
    assert(p63_relative_alt_GET(pack) == (int32_t)125563192);
    assert(p63_lon_GET(pack) == (int32_t)1977843962);
    assert(p63_lat_GET(pack) == (int32_t) -1508592133);
    assert(p63_vx_GET(pack) == (float)1.6859252E38F);
    assert(p63_vz_GET(pack) == (float)9.630344E37F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vy_GET(pack) == (float)1.0815608E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_y_GET(pack) == (float)1.9297705E38F);
    assert(p64_ay_GET(pack) == (float) -3.0294322E38F);
    assert(p64_ax_GET(pack) == (float)2.7051848E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)5434551636133513944L);
    assert(p64_vx_GET(pack) == (float) -2.9892446E38F);
    assert(p64_vz_GET(pack) == (float) -2.2206486E38F);
    assert(p64_az_GET(pack) == (float) -3.0957524E38F);
    assert(p64_x_GET(pack) == (float) -8.637642E37F);
    {
        float exemplary[] =  {-1.2083756E38F, -1.6258282E37F, 3.3505596E38F, 2.1511086E38F, -8.672114E37F, 1.894085E38F, 5.202019E36F, 3.212736E37F, -7.840233E37F, -1.9384538E38F, 8.817739E37F, -2.977143E38F, -2.9575566E38F, -8.156426E37F, 2.0958434E38F, -4.958662E36F, -2.6546934E38F, 1.9080778E38F, -5.9246E37F, 1.0891799E38F, -3.230018E38F, -1.1402053E38F, -3.1039315E38F, -1.5808921E38F, 7.351391E37F, 2.4879114E38F, 1.4305344E38F, 3.538508E37F, 1.1414882E38F, -9.200113E37F, -5.1252366E37F, -3.2844245E38F, -1.1563615E38F, -2.6421362E38F, -3.3796829E38F, -7.8338657E37F, -3.1642478E38F, -1.8173437E38F, 2.5526378E38F, 2.3198255E36F, -2.4246953E38F, 2.581964E38F, 2.8594918E38F, 1.3099504E38F, -1.7613364E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_z_GET(pack) == (float)3.3627864E38F);
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)50727);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)13705);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)49558);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)7093);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)52223);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)2485);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)5188);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)33421);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)57083);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)63460);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)10209);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)8703);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3045469240L);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)13077);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)47702);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)25461);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)41847);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)8572);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)28947);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)57);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)48386);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)22005);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)118);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)51929);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)27683);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)29212);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -5949);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)15469);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)50387);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)11618);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)32576);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)18615);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)11485);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)7286);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)23308);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)8809);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_y_GET(pack) == (int32_t) -738347609);
    assert(p73_param2_GET(pack) == (float)2.482791E38F);
    assert(p73_param1_GET(pack) == (float)2.818316E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p73_param3_GET(pack) == (float) -2.481578E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p73_x_GET(pack) == (int32_t)890077737);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p73_z_GET(pack) == (float) -1.2453007E37F);
    assert(p73_param4_GET(pack) == (float) -1.7960691E37F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)49576);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_airspeed_GET(pack) == (float) -2.8551098E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)6800);
    assert(p74_climb_GET(pack) == (float)2.1317072E38F);
    assert(p74_groundspeed_GET(pack) == (float)2.7293075E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)24211);
    assert(p74_alt_GET(pack) == (float) -1.968893E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p75_x_GET(pack) == (int32_t) -614488700);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p75_z_GET(pack) == (float) -1.24216E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p75_param3_GET(pack) == (float)1.1674076E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p75_param1_GET(pack) == (float)1.3220899E38F);
    assert(p75_y_GET(pack) == (int32_t)990872196);
    assert(p75_param2_GET(pack) == (float)3.0598193E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_PARACHUTE);
    assert(p75_param4_GET(pack) == (float)1.4005405E38F);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param4_GET(pack) == (float) -8.881973E36F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p76_param3_GET(pack) == (float)1.6890158E38F);
    assert(p76_param2_GET(pack) == (float)3.0489252E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p76_param6_GET(pack) == (float) -1.7294277E38F);
    assert(p76_param5_GET(pack) == (float)1.4031938E38F);
    assert(p76_param1_GET(pack) == (float)1.6075319E38F);
    assert(p76_param7_GET(pack) == (float) -4.5132707E37F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_LAND_START);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)3);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_DENIED);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_TURN_LIGHT);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)104);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)130);
    assert(p77_result_param2_TRY(ph) == (int32_t)7278856);
};


void c_TEST_Channel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float)1.033371E38F);
    assert(p81_pitch_GET(pack) == (float)2.7030396E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2786737918L);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p81_roll_GET(pack) == (float)1.3283419E38F);
    assert(p81_yaw_GET(pack) == (float)1.5703344E38F);
};


void c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)88);
    {
        float exemplary[] =  {5.321458E36F, -1.9728319E38F, 2.5983188E38F, -3.0468047E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.0655358E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float) -2.4044492E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)1.6367781E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3332174594L);
    assert(p82_thrust_GET(pack) == (float) -1.5606732E37F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_TEST_Channel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float)2.0427037E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)1.3225716E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)117);
    {
        float exemplary[] =  {-1.6033493E38F, -1.3538407E38F, -1.9652353E38F, 2.641194E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_pitch_rate_GET(pack) == (float)3.1069863E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)669870197L);
    assert(p83_body_roll_rate_GET(pack) == (float) -3.0019252E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afx_GET(pack) == (float) -2.723192E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p84_vx_GET(pack) == (float) -4.427907E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p84_z_GET(pack) == (float) -9.224202E37F);
    assert(p84_yaw_GET(pack) == (float) -4.487791E37F);
    assert(p84_afz_GET(pack) == (float) -3.1600019E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)18744);
    assert(p84_yaw_rate_GET(pack) == (float) -3.2551943E38F);
    assert(p84_y_GET(pack) == (float)2.8384015E38F);
    assert(p84_vz_GET(pack) == (float)7.5297163E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2673692174L);
    assert(p84_afy_GET(pack) == (float) -1.6420238E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p84_x_GET(pack) == (float) -1.4941344E38F);
    assert(p84_vy_GET(pack) == (float)1.1873839E38F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vy_GET(pack) == (float) -5.393573E37F);
    assert(p86_afy_GET(pack) == (float)2.0151509E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -2.4027719E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)36479);
    assert(p86_lat_int_GET(pack) == (int32_t)223782783);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p86_vz_GET(pack) == (float)1.0802071E38F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1546633235);
    assert(p86_afz_GET(pack) == (float)2.0019235E38F);
    assert(p86_alt_GET(pack) == (float) -2.0087066E38F);
    assert(p86_afx_GET(pack) == (float)5.2352925E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2628653426L);
    assert(p86_vx_GET(pack) == (float)9.294659E37F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p86_yaw_GET(pack) == (float)2.5803196E38F);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afx_GET(pack) == (float) -2.0472472E38F);
    assert(p87_alt_GET(pack) == (float) -1.1625311E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)815859900L);
    assert(p87_yaw_rate_GET(pack) == (float) -1.1575445E38F);
    assert(p87_lat_int_GET(pack) == (int32_t)1901309378);
    assert(p87_yaw_GET(pack) == (float)3.2926938E38F);
    assert(p87_vy_GET(pack) == (float)1.0452664E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)2106576766);
    assert(p87_afz_GET(pack) == (float) -2.8309978E38F);
    assert(p87_vx_GET(pack) == (float)2.0713715E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p87_afy_GET(pack) == (float) -2.6209252E38F);
    assert(p87_vz_GET(pack) == (float)4.9452307E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)2961);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float)1.4586736E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2200897410L);
    assert(p89_z_GET(pack) == (float) -3.9658807E37F);
    assert(p89_pitch_GET(pack) == (float) -1.7877364E38F);
    assert(p89_x_GET(pack) == (float) -3.0720539E38F);
    assert(p89_roll_GET(pack) == (float)2.2900504E38F);
    assert(p89_yaw_GET(pack) == (float) -7.909022E37F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -14485);
    assert(p90_pitchspeed_GET(pack) == (float) -1.4303324E38F);
    assert(p90_lon_GET(pack) == (int32_t) -1841375888);
    assert(p90_alt_GET(pack) == (int32_t) -409128906);
    assert(p90_pitch_GET(pack) == (float)3.0228524E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -32406);
    assert(p90_yawspeed_GET(pack) == (float)1.1262859E37F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)5110);
    assert(p90_roll_GET(pack) == (float) -3.356502E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)7293);
    assert(p90_lat_GET(pack) == (int32_t) -1305774060);
    assert(p90_time_usec_GET(pack) == (uint64_t)2832205653053135463L);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -9533);
    assert(p90_rollspeed_GET(pack) == (float) -3.0479551E38F);
    assert(p90_yaw_GET(pack) == (float)1.1386386E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)3458);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_roll_ailerons_GET(pack) == (float)3.756973E37F);
    assert(p91_throttle_GET(pack) == (float)3.8237058E37F);
    assert(p91_yaw_rudder_GET(pack) == (float) -1.7107367E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
    assert(p91_time_usec_GET(pack) == (uint64_t)1073602316216600177L);
    assert(p91_aux2_GET(pack) == (float)8.982039E36F);
    assert(p91_aux3_GET(pack) == (float) -1.45688E38F);
    assert(p91_aux1_GET(pack) == (float) -1.734185E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p91_aux4_GET(pack) == (float) -1.6101972E38F);
    assert(p91_pitch_elevator_GET(pack) == (float)6.0828336E37F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)15950);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)12787);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)10402);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)62979);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)18117);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)22824);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)47765);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)17690);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)63244);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)56971);
    assert(p92_time_usec_GET(pack) == (uint64_t)1681518652508885641L);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)25424);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)16574);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)5260745353416405871L);
    assert(p93_flags_GET(pack) == (uint64_t)2239029103081186530L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED);
    {
        float exemplary[] =  {-1.2803587E38F, -1.963452E38F, 1.3767976E38F, -5.522936E37F, 1.876611E38F, 2.8575644E38F, 1.1395382E38F, 2.5705275E38F, 2.0165666E38F, -1.3129052E37F, 2.0861424E38F, -1.1335906E38F, -2.882793E38F, -3.3205546E38F, 1.3784452E38F, 1.4077943E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -1642);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -2.5025668E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -16359);
    assert(p100_flow_comp_m_x_GET(pack) == (float)2.8694677E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.837952E38F);
    assert(p100_ground_distance_GET(pack) == (float) -1.0412376E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float) -2.5601968E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)5354725655768243690L);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_roll_GET(pack) == (float) -3.2904705E38F);
    assert(p101_y_GET(pack) == (float)2.86853E38F);
    assert(p101_yaw_GET(pack) == (float) -2.1922816E38F);
    assert(p101_usec_GET(pack) == (uint64_t)7472208955689673906L);
    assert(p101_z_GET(pack) == (float)2.0210529E38F);
    assert(p101_x_GET(pack) == (float) -1.8054657E38F);
    assert(p101_pitch_GET(pack) == (float) -7.566639E37F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float)1.3605125E38F);
    assert(p102_yaw_GET(pack) == (float)1.705023E38F);
    assert(p102_x_GET(pack) == (float) -2.650691E38F);
    assert(p102_roll_GET(pack) == (float)2.4752083E38F);
    assert(p102_pitch_GET(pack) == (float) -3.1935512E38F);
    assert(p102_z_GET(pack) == (float)1.3934371E38F);
    assert(p102_usec_GET(pack) == (uint64_t)8843999295538589401L);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_usec_GET(pack) == (uint64_t)3309575221279712701L);
    assert(p103_z_GET(pack) == (float) -1.4767835E37F);
    assert(p103_x_GET(pack) == (float) -3.3698098E38F);
    assert(p103_y_GET(pack) == (float)9.812353E37F);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float) -3.277412E38F);
    assert(p104_yaw_GET(pack) == (float)2.2341906E38F);
    assert(p104_y_GET(pack) == (float)5.775396E37F);
    assert(p104_pitch_GET(pack) == (float)2.0749785E38F);
    assert(p104_roll_GET(pack) == (float) -3.0842235E38F);
    assert(p104_z_GET(pack) == (float) -1.4682016E38F);
    assert(p104_usec_GET(pack) == (uint64_t)6662620311104895561L);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_yacc_GET(pack) == (float)2.6202707E38F);
    assert(p105_zmag_GET(pack) == (float) -1.1027461E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)56854);
    assert(p105_zacc_GET(pack) == (float) -2.3568973E38F);
    assert(p105_ygyro_GET(pack) == (float) -2.5484195E38F);
    assert(p105_pressure_alt_GET(pack) == (float)1.5032194E38F);
    assert(p105_ymag_GET(pack) == (float)2.9938204E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -1.4539798E38F);
    assert(p105_temperature_GET(pack) == (float) -3.7984326E36F);
    assert(p105_xmag_GET(pack) == (float)1.6272458E38F);
    assert(p105_diff_pressure_GET(pack) == (float)3.438E36F);
    assert(p105_time_usec_GET(pack) == (uint64_t)5278025634902290332L);
    assert(p105_xgyro_GET(pack) == (float)2.8072486E38F);
    assert(p105_zgyro_GET(pack) == (float)4.945496E37F);
    assert(p105_xacc_GET(pack) == (float) -7.4203114E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)409152074L);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)202260266L);
    assert(p106_distance_GET(pack) == (float) -2.655103E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)6316001911933209737L);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p106_integrated_y_GET(pack) == (float)1.8098801E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -1.2067296E37F);
    assert(p106_integrated_zgyro_GET(pack) == (float)3.5265817E37F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -17494);
    assert(p106_integrated_ygyro_GET(pack) == (float)1.2370526E38F);
    assert(p106_integrated_x_GET(pack) == (float)3.0263609E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)83);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_abs_pressure_GET(pack) == (float)3.355604E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)1551947457327364129L);
    assert(p107_diff_pressure_GET(pack) == (float) -3.0976827E38F);
    assert(p107_zgyro_GET(pack) == (float) -1.3857324E36F);
    assert(p107_xmag_GET(pack) == (float) -1.368391E38F);
    assert(p107_zacc_GET(pack) == (float) -1.8398498E38F);
    assert(p107_zmag_GET(pack) == (float)2.3162276E38F);
    assert(p107_ygyro_GET(pack) == (float) -1.7842622E38F);
    assert(p107_temperature_GET(pack) == (float)7.4884695E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2735626694L);
    assert(p107_ymag_GET(pack) == (float)2.3068815E38F);
    assert(p107_xacc_GET(pack) == (float) -3.4134568E37F);
    assert(p107_pressure_alt_GET(pack) == (float)1.9876873E37F);
    assert(p107_xgyro_GET(pack) == (float)5.328029E37F);
    assert(p107_yacc_GET(pack) == (float)3.797947E37F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_zgyro_GET(pack) == (float) -2.0618325E38F);
    assert(p108_yaw_GET(pack) == (float) -1.5481686E37F);
    assert(p108_std_dev_vert_GET(pack) == (float) -2.474224E38F);
    assert(p108_q2_GET(pack) == (float)2.2749716E38F);
    assert(p108_roll_GET(pack) == (float)8.314679E37F);
    assert(p108_yacc_GET(pack) == (float) -2.4727515E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)1.9597769E38F);
    assert(p108_lon_GET(pack) == (float)1.4680727E38F);
    assert(p108_lat_GET(pack) == (float) -5.67679E37F);
    assert(p108_xgyro_GET(pack) == (float) -1.6345789E38F);
    assert(p108_pitch_GET(pack) == (float)2.4637252E38F);
    assert(p108_vd_GET(pack) == (float) -3.3758887E37F);
    assert(p108_q4_GET(pack) == (float) -2.5287402E38F);
    assert(p108_q3_GET(pack) == (float)2.3235058E38F);
    assert(p108_q1_GET(pack) == (float) -1.893283E38F);
    assert(p108_vn_GET(pack) == (float)1.8232296E38F);
    assert(p108_alt_GET(pack) == (float) -1.4629266E38F);
    assert(p108_xacc_GET(pack) == (float) -1.0788892E38F);
    assert(p108_ve_GET(pack) == (float) -1.0258425E38F);
    assert(p108_ygyro_GET(pack) == (float) -1.4739588E38F);
    assert(p108_zacc_GET(pack) == (float)2.3526502E37F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)27785);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)7794);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)235);
    {
        uint8_t exemplary[] =  {(uint8_t)61, (uint8_t)118, (uint8_t)230, (uint8_t)125, (uint8_t)64, (uint8_t)195, (uint8_t)177, (uint8_t)199, (uint8_t)39, (uint8_t)219, (uint8_t)30, (uint8_t)6, (uint8_t)111, (uint8_t)181, (uint8_t)148, (uint8_t)133, (uint8_t)105, (uint8_t)32, (uint8_t)153, (uint8_t)221, (uint8_t)133, (uint8_t)67, (uint8_t)244, (uint8_t)88, (uint8_t)227, (uint8_t)120, (uint8_t)251, (uint8_t)183, (uint8_t)94, (uint8_t)38, (uint8_t)7, (uint8_t)87, (uint8_t)83, (uint8_t)178, (uint8_t)66, (uint8_t)98, (uint8_t)135, (uint8_t)134, (uint8_t)127, (uint8_t)182, (uint8_t)149, (uint8_t)97, (uint8_t)90, (uint8_t)126, (uint8_t)245, (uint8_t)225, (uint8_t)140, (uint8_t)253, (uint8_t)79, (uint8_t)77, (uint8_t)83, (uint8_t)159, (uint8_t)251, (uint8_t)59, (uint8_t)208, (uint8_t)55, (uint8_t)23, (uint8_t)57, (uint8_t)129, (uint8_t)239, (uint8_t)160, (uint8_t)228, (uint8_t)156, (uint8_t)186, (uint8_t)94, (uint8_t)165, (uint8_t)152, (uint8_t)170, (uint8_t)29, (uint8_t)72, (uint8_t)9, (uint8_t)59, (uint8_t)197, (uint8_t)56, (uint8_t)18, (uint8_t)224, (uint8_t)145, (uint8_t)189, (uint8_t)106, (uint8_t)48, (uint8_t)241, (uint8_t)212, (uint8_t)99, (uint8_t)78, (uint8_t)11, (uint8_t)14, (uint8_t)137, (uint8_t)0, (uint8_t)106, (uint8_t)88, (uint8_t)131, (uint8_t)237, (uint8_t)27, (uint8_t)241, (uint8_t)139, (uint8_t)56, (uint8_t)207, (uint8_t)240, (uint8_t)33, (uint8_t)197, (uint8_t)129, (uint8_t)200, (uint8_t)38, (uint8_t)196, (uint8_t)66, (uint8_t)202, (uint8_t)244, (uint8_t)192, (uint8_t)127, (uint8_t)140, (uint8_t)237, (uint8_t)222, (uint8_t)167, (uint8_t)67, (uint8_t)206, (uint8_t)77, (uint8_t)18, (uint8_t)153, (uint8_t)221, (uint8_t)98, (uint8_t)6, (uint8_t)147, (uint8_t)161, (uint8_t)63, (uint8_t)244, (uint8_t)91, (uint8_t)45, (uint8_t)127, (uint8_t)50, (uint8_t)249, (uint8_t)215, (uint8_t)141, (uint8_t)144, (uint8_t)35, (uint8_t)132, (uint8_t)214, (uint8_t)133, (uint8_t)114, (uint8_t)157, (uint8_t)143, (uint8_t)54, (uint8_t)223, (uint8_t)253, (uint8_t)186, (uint8_t)99, (uint8_t)211, (uint8_t)7, (uint8_t)157, (uint8_t)58, (uint8_t)96, (uint8_t)180, (uint8_t)169, (uint8_t)15, (uint8_t)213, (uint8_t)82, (uint8_t)220, (uint8_t)166, (uint8_t)175, (uint8_t)40, (uint8_t)232, (uint8_t)220, (uint8_t)222, (uint8_t)254, (uint8_t)91, (uint8_t)15, (uint8_t)181, (uint8_t)122, (uint8_t)134, (uint8_t)146, (uint8_t)40, (uint8_t)212, (uint8_t)25, (uint8_t)42, (uint8_t)18, (uint8_t)15, (uint8_t)68, (uint8_t)46, (uint8_t)0, (uint8_t)232, (uint8_t)79, (uint8_t)53, (uint8_t)24, (uint8_t)99, (uint8_t)64, (uint8_t)168, (uint8_t)201, (uint8_t)115, (uint8_t)127, (uint8_t)24, (uint8_t)63, (uint8_t)123, (uint8_t)192, (uint8_t)76, (uint8_t)91, (uint8_t)205, (uint8_t)243, (uint8_t)95, (uint8_t)211, (uint8_t)239, (uint8_t)156, (uint8_t)170, (uint8_t)100, (uint8_t)27, (uint8_t)70, (uint8_t)34, (uint8_t)83, (uint8_t)172, (uint8_t)10, (uint8_t)158, (uint8_t)23, (uint8_t)103, (uint8_t)161, (uint8_t)18, (uint8_t)180, (uint8_t)205, (uint8_t)128, (uint8_t)185, (uint8_t)162, (uint8_t)147, (uint8_t)25, (uint8_t)154, (uint8_t)20, (uint8_t)67, (uint8_t)102, (uint8_t)102, (uint8_t)75, (uint8_t)55, (uint8_t)200, (uint8_t)71, (uint8_t)200, (uint8_t)97, (uint8_t)187, (uint8_t)34, (uint8_t)15, (uint8_t)103, (uint8_t)194, (uint8_t)2, (uint8_t)207, (uint8_t)194, (uint8_t)123, (uint8_t)88, (uint8_t)31, (uint8_t)202, (uint8_t)109, (uint8_t)243, (uint8_t)200, (uint8_t)104, (uint8_t)221, (uint8_t)107, (uint8_t)128, (uint8_t)238} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)139);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -8439932114570341042L);
    assert(p111_tc1_GET(pack) == (int64_t)7517988402439648145L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)1092450327L);
    assert(p112_time_usec_GET(pack) == (uint64_t)510017626207449017L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -28158);
    assert(p113_lon_GET(pack) == (int32_t)366181130);
    assert(p113_lat_GET(pack) == (int32_t)1261803483);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p113_alt_GET(pack) == (int32_t) -1679955975);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p113_time_usec_GET(pack) == (uint64_t)8356598505193163680L);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)24770);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)36129);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)1869);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)7204);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)19506);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)204);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_y_GET(pack) == (float)1.7384074E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)2395984618L);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -54);
    assert(p114_distance_GET(pack) == (float)2.090094E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.5507503E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)1182259997L);
    assert(p114_integrated_x_GET(pack) == (float)2.087782E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -2.8414072E38F);
    assert(p114_integrated_ygyro_GET(pack) == (float) -1.423513E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)8639009172074228845L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yawspeed_GET(pack) == (float)1.7750216E38F);
    assert(p115_rollspeed_GET(pack) == (float)3.2759178E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)24338);
    assert(p115_lon_GET(pack) == (int32_t)640215503);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)6504);
    assert(p115_pitchspeed_GET(pack) == (float)2.6645121E38F);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)26518);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -4297);
    {
        float exemplary[] =  {-2.609413E38F, 3.6031221E37F, 1.2476989E38F, 3.3152702E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)22840);
    assert(p115_lat_GET(pack) == (int32_t)618862939);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)9181);
    assert(p115_time_usec_GET(pack) == (uint64_t)1130406468568807247L);
    assert(p115_alt_GET(pack) == (int32_t)241589092);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)11715);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -4358);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -4137);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2556121615L);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)3825);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t) -409);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -11885);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -10276);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -12568);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)21724);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)16219);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)26927);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)58105);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)1638);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)22);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)57323);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)64433);
    assert(p118_time_utc_GET(pack) == (uint32_t)858625074L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)16759);
    assert(p118_size_GET(pack) == (uint32_t)2502954282L);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_count_GET(pack) == (uint32_t)3959045842L);
    assert(p119_ofs_GET(pack) == (uint32_t)1458794439L);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)31397);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)489941814L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)6288);
    {
        uint8_t exemplary[] =  {(uint8_t)30, (uint8_t)54, (uint8_t)228, (uint8_t)65, (uint8_t)193, (uint8_t)51, (uint8_t)36, (uint8_t)155, (uint8_t)109, (uint8_t)44, (uint8_t)16, (uint8_t)20, (uint8_t)248, (uint8_t)37, (uint8_t)88, (uint8_t)46, (uint8_t)98, (uint8_t)89, (uint8_t)170, (uint8_t)171, (uint8_t)65, (uint8_t)78, (uint8_t)191, (uint8_t)138, (uint8_t)2, (uint8_t)210, (uint8_t)194, (uint8_t)125, (uint8_t)19, (uint8_t)61, (uint8_t)40, (uint8_t)205, (uint8_t)187, (uint8_t)35, (uint8_t)202, (uint8_t)63, (uint8_t)25, (uint8_t)37, (uint8_t)86, (uint8_t)112, (uint8_t)250, (uint8_t)236, (uint8_t)136, (uint8_t)163, (uint8_t)38, (uint8_t)12, (uint8_t)230, (uint8_t)241, (uint8_t)173, (uint8_t)23, (uint8_t)152, (uint8_t)248, (uint8_t)62, (uint8_t)100, (uint8_t)61, (uint8_t)128, (uint8_t)66, (uint8_t)94, (uint8_t)151, (uint8_t)7, (uint8_t)224, (uint8_t)226, (uint8_t)22, (uint8_t)63, (uint8_t)24, (uint8_t)42, (uint8_t)30, (uint8_t)90, (uint8_t)162, (uint8_t)211, (uint8_t)10, (uint8_t)114, (uint8_t)214, (uint8_t)26, (uint8_t)201, (uint8_t)236, (uint8_t)59, (uint8_t)0, (uint8_t)76, (uint8_t)14, (uint8_t)136, (uint8_t)20, (uint8_t)19, (uint8_t)163, (uint8_t)70, (uint8_t)159, (uint8_t)129, (uint8_t)201, (uint8_t)39, (uint8_t)80} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)87);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)101);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    {
        uint8_t exemplary[] =  {(uint8_t)173, (uint8_t)124, (uint8_t)211, (uint8_t)251, (uint8_t)158, (uint8_t)125, (uint8_t)97, (uint8_t)29, (uint8_t)25, (uint8_t)54, (uint8_t)176, (uint8_t)50, (uint8_t)132, (uint8_t)118, (uint8_t)214, (uint8_t)196, (uint8_t)39, (uint8_t)160, (uint8_t)27, (uint8_t)22, (uint8_t)213, (uint8_t)232, (uint8_t)6, (uint8_t)179, (uint8_t)159, (uint8_t)82, (uint8_t)1, (uint8_t)43, (uint8_t)172, (uint8_t)34, (uint8_t)187, (uint8_t)62, (uint8_t)26, (uint8_t)28, (uint8_t)6, (uint8_t)220, (uint8_t)243, (uint8_t)251, (uint8_t)246, (uint8_t)57, (uint8_t)10, (uint8_t)166, (uint8_t)142, (uint8_t)95, (uint8_t)111, (uint8_t)17, (uint8_t)40, (uint8_t)35, (uint8_t)161, (uint8_t)35, (uint8_t)32, (uint8_t)239, (uint8_t)115, (uint8_t)204, (uint8_t)165, (uint8_t)101, (uint8_t)212, (uint8_t)254, (uint8_t)139, (uint8_t)106, (uint8_t)63, (uint8_t)233, (uint8_t)224, (uint8_t)32, (uint8_t)179, (uint8_t)194, (uint8_t)83, (uint8_t)144, (uint8_t)4, (uint8_t)138, (uint8_t)33, (uint8_t)76, (uint8_t)126, (uint8_t)17, (uint8_t)172, (uint8_t)171, (uint8_t)77, (uint8_t)124, (uint8_t)68, (uint8_t)54, (uint8_t)28, (uint8_t)234, (uint8_t)101, (uint8_t)165, (uint8_t)237, (uint8_t)227, (uint8_t)159, (uint8_t)38, (uint8_t)192, (uint8_t)79, (uint8_t)223, (uint8_t)78, (uint8_t)21, (uint8_t)244, (uint8_t)140, (uint8_t)113, (uint8_t)104, (uint8_t)26, (uint8_t)230, (uint8_t)153, (uint8_t)28, (uint8_t)243, (uint8_t)115, (uint8_t)28, (uint8_t)69, (uint8_t)170, (uint8_t)86, (uint8_t)25, (uint8_t)236, (uint8_t)211} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)210);
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)45846);
    assert(p124_alt_GET(pack) == (int32_t)1963830972);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)59294);
    assert(p124_time_usec_GET(pack) == (uint64_t)760136646223207252L);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)12174);
    assert(p124_lat_GET(pack) == (int32_t) -989848541);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p124_lon_GET(pack) == (int32_t) -739647616);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1640087953L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)1279);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)20488);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)14702);
    assert(p125_flags_GET(pack) == (e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED));
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)182, (uint8_t)132, (uint8_t)140, (uint8_t)181, (uint8_t)3, (uint8_t)132, (uint8_t)154, (uint8_t)183, (uint8_t)114, (uint8_t)35, (uint8_t)152, (uint8_t)159, (uint8_t)251, (uint8_t)146, (uint8_t)198, (uint8_t)112, (uint8_t)162, (uint8_t)173, (uint8_t)204, (uint8_t)87, (uint8_t)23, (uint8_t)79, (uint8_t)10, (uint8_t)191, (uint8_t)40, (uint8_t)167, (uint8_t)16, (uint8_t)10, (uint8_t)138, (uint8_t)31, (uint8_t)34, (uint8_t)84, (uint8_t)58, (uint8_t)92, (uint8_t)135, (uint8_t)108, (uint8_t)103, (uint8_t)172, (uint8_t)172, (uint8_t)199, (uint8_t)36, (uint8_t)7, (uint8_t)0, (uint8_t)147, (uint8_t)47, (uint8_t)161, (uint8_t)131, (uint8_t)195, (uint8_t)209, (uint8_t)111, (uint8_t)119, (uint8_t)183, (uint8_t)209, (uint8_t)136, (uint8_t)247, (uint8_t)14, (uint8_t)232, (uint8_t)91, (uint8_t)133, (uint8_t)236, (uint8_t)149, (uint8_t)91, (uint8_t)122, (uint8_t)36, (uint8_t)82, (uint8_t)54, (uint8_t)97, (uint8_t)13, (uint8_t)69, (uint8_t)40} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)6970);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1);
    assert(p126_baudrate_GET(pack) == (uint32_t)3097330505L);
    assert(p126_flags_GET(pack) == (e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY));
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)201516214);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p127_tow_GET(pack) == (uint32_t)2743584219L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)821895812L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)354223196);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)2093463328);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p127_accuracy_GET(pack) == (uint32_t)2832729612L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -577622754);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)39221);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)1213886110);
    assert(p128_tow_GET(pack) == (uint32_t)3749454599L);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)2114971928);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p128_accuracy_GET(pack) == (uint32_t)668987585L);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)534180396L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)1525939098);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -971032171);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)45273);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)2368);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -16730);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -14898);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -5856);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)26334);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)19732);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)30970);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)966557289L);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -10678);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)11860);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)6724);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p130_size_GET(pack) == (uint32_t)501245363L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)45590);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)42846);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)50304);
    {
        uint8_t exemplary[] =  {(uint8_t)68, (uint8_t)178, (uint8_t)9, (uint8_t)149, (uint8_t)227, (uint8_t)168, (uint8_t)95, (uint8_t)60, (uint8_t)10, (uint8_t)241, (uint8_t)81, (uint8_t)185, (uint8_t)142, (uint8_t)229, (uint8_t)109, (uint8_t)96, (uint8_t)247, (uint8_t)203, (uint8_t)162, (uint8_t)253, (uint8_t)221, (uint8_t)234, (uint8_t)66, (uint8_t)179, (uint8_t)248, (uint8_t)141, (uint8_t)41, (uint8_t)67, (uint8_t)217, (uint8_t)172, (uint8_t)186, (uint8_t)52, (uint8_t)50, (uint8_t)180, (uint8_t)177, (uint8_t)246, (uint8_t)184, (uint8_t)203, (uint8_t)227, (uint8_t)229, (uint8_t)156, (uint8_t)236, (uint8_t)180, (uint8_t)134, (uint8_t)141, (uint8_t)190, (uint8_t)83, (uint8_t)186, (uint8_t)168, (uint8_t)57, (uint8_t)164, (uint8_t)220, (uint8_t)88, (uint8_t)173, (uint8_t)43, (uint8_t)148, (uint8_t)21, (uint8_t)75, (uint8_t)187, (uint8_t)199, (uint8_t)81, (uint8_t)48, (uint8_t)216, (uint8_t)50, (uint8_t)31, (uint8_t)246, (uint8_t)224, (uint8_t)45, (uint8_t)242, (uint8_t)147, (uint8_t)4, (uint8_t)16, (uint8_t)129, (uint8_t)37, (uint8_t)27, (uint8_t)45, (uint8_t)161, (uint8_t)228, (uint8_t)9, (uint8_t)63, (uint8_t)164, (uint8_t)155, (uint8_t)218, (uint8_t)7, (uint8_t)157, (uint8_t)227, (uint8_t)109, (uint8_t)147, (uint8_t)219, (uint8_t)6, (uint8_t)185, (uint8_t)185, (uint8_t)95, (uint8_t)211, (uint8_t)15, (uint8_t)129, (uint8_t)178, (uint8_t)178, (uint8_t)85, (uint8_t)139, (uint8_t)58, (uint8_t)84, (uint8_t)77, (uint8_t)31, (uint8_t)254, (uint8_t)227, (uint8_t)155, (uint8_t)169, (uint8_t)188, (uint8_t)114, (uint8_t)241, (uint8_t)41, (uint8_t)241, (uint8_t)172, (uint8_t)64, (uint8_t)71, (uint8_t)155, (uint8_t)87, (uint8_t)209, (uint8_t)58, (uint8_t)69, (uint8_t)9, (uint8_t)84, (uint8_t)31, (uint8_t)0, (uint8_t)218, (uint8_t)73, (uint8_t)183, (uint8_t)151, (uint8_t)184, (uint8_t)35, (uint8_t)174, (uint8_t)172, (uint8_t)147, (uint8_t)33, (uint8_t)184, (uint8_t)228, (uint8_t)70, (uint8_t)37, (uint8_t)43, (uint8_t)197, (uint8_t)209, (uint8_t)55, (uint8_t)72, (uint8_t)187, (uint8_t)93, (uint8_t)46, (uint8_t)47, (uint8_t)157, (uint8_t)146, (uint8_t)108, (uint8_t)108, (uint8_t)157, (uint8_t)216, (uint8_t)209, (uint8_t)253, (uint8_t)94, (uint8_t)118, (uint8_t)4, (uint8_t)143, (uint8_t)148, (uint8_t)74, (uint8_t)249, (uint8_t)91, (uint8_t)173, (uint8_t)116, (uint8_t)58, (uint8_t)196, (uint8_t)14, (uint8_t)171, (uint8_t)66, (uint8_t)159, (uint8_t)222, (uint8_t)40, (uint8_t)142, (uint8_t)96, (uint8_t)253, (uint8_t)194, (uint8_t)218, (uint8_t)8, (uint8_t)54, (uint8_t)115, (uint8_t)111, (uint8_t)240, (uint8_t)190, (uint8_t)128, (uint8_t)100, (uint8_t)193, (uint8_t)25, (uint8_t)28, (uint8_t)195, (uint8_t)109, (uint8_t)140, (uint8_t)115, (uint8_t)237, (uint8_t)142, (uint8_t)144, (uint8_t)55, (uint8_t)216, (uint8_t)129, (uint8_t)237, (uint8_t)114, (uint8_t)223, (uint8_t)167, (uint8_t)228, (uint8_t)4, (uint8_t)55, (uint8_t)245, (uint8_t)74, (uint8_t)199, (uint8_t)121, (uint8_t)169, (uint8_t)18, (uint8_t)233, (uint8_t)219, (uint8_t)132, (uint8_t)155, (uint8_t)82, (uint8_t)161, (uint8_t)5, (uint8_t)163, (uint8_t)202, (uint8_t)197, (uint8_t)175, (uint8_t)79, (uint8_t)69, (uint8_t)251, (uint8_t)198, (uint8_t)144, (uint8_t)130, (uint8_t)219, (uint8_t)124, (uint8_t)19, (uint8_t)92, (uint8_t)197, (uint8_t)129, (uint8_t)221, (uint8_t)19, (uint8_t)161, (uint8_t)50, (uint8_t)210, (uint8_t)87, (uint8_t)134, (uint8_t)89, (uint8_t)126, (uint8_t)161, (uint8_t)79, (uint8_t)79, (uint8_t)168, (uint8_t)102, (uint8_t)39, (uint8_t)28, (uint8_t)41} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2454020105L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)64445);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)29768);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)26051);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)218);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t) -1468581729);
    assert(p133_lon_GET(pack) == (int32_t)884446308);
    assert(p133_mask_GET(pack) == (uint64_t)9121066560038311676L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)37700);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)42);
    {
        int16_t exemplary[] =  {(int16_t) -28575, (int16_t)11511, (int16_t)3118, (int16_t) -7877, (int16_t) -14561, (int16_t)29769, (int16_t)13778, (int16_t)16503, (int16_t) -11041, (int16_t)4307, (int16_t) -15659, (int16_t)11932, (int16_t)19535, (int16_t) -32581, (int16_t)31572, (int16_t) -17178} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lat_GET(pack) == (int32_t) -755014592);
    assert(p134_lon_GET(pack) == (int32_t) -819922380);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)52153);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t) -730051938);
    assert(p135_lat_GET(pack) == (int32_t)2041419854);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)49411);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)3363);
    assert(p136_lon_GET(pack) == (int32_t) -1395858177);
    assert(p136_lat_GET(pack) == (int32_t) -670223865);
    assert(p136_current_height_GET(pack) == (float)1.1898775E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)8060);
    assert(p136_terrain_height_GET(pack) == (float) -5.334361E37F);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)2.9691862E38F);
    assert(p137_press_diff_GET(pack) == (float)2.701547E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)4177462740L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)2329);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float) -1.7049639E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)6510877867391602605L);
    assert(p138_z_GET(pack) == (float)1.0896863E38F);
    assert(p138_y_GET(pack) == (float)2.990864E38F);
    {
        float exemplary[] =  {-3.20737E37F, 1.5144625E38F, -2.6561913E38F, -2.6428126E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_time_usec_GET(pack) == (uint64_t)1597536677507813078L);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)170);
    {
        float exemplary[] =  {2.6567122E37F, -1.738392E38F, -1.2494255E38F, -2.685112E38F, 1.5473886E37F, 1.4867653E38F, -3.0646017E38F, 7.3236365E36F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)1221740477354467599L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)249);
    {
        float exemplary[] =  {1.7506742E38F, 2.8661289E37F, 1.8561244E38F, 2.4901434E38F, -3.3080714E37F, 2.1715289E38F, 2.0233821E38F, 2.4651658E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_time_usec_GET(pack) == (uint64_t)8601352168081623731L);
    assert(p141_altitude_relative_GET(pack) == (float) -7.6910274E37F);
    assert(p141_altitude_local_GET(pack) == (float) -1.6517595E38F);
    assert(p141_bottom_clearance_GET(pack) == (float)2.7745892E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -2.9129976E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)2.6466644E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -3.0527416E38F);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)3);
    {
        uint8_t exemplary[] =  {(uint8_t)182, (uint8_t)206, (uint8_t)107, (uint8_t)56, (uint8_t)72, (uint8_t)100, (uint8_t)216, (uint8_t)201, (uint8_t)208, (uint8_t)8, (uint8_t)62, (uint8_t)181, (uint8_t)209, (uint8_t)242, (uint8_t)120, (uint8_t)28, (uint8_t)78, (uint8_t)38, (uint8_t)72, (uint8_t)135, (uint8_t)240, (uint8_t)33, (uint8_t)41, (uint8_t)60, (uint8_t)99, (uint8_t)208, (uint8_t)133, (uint8_t)193, (uint8_t)90, (uint8_t)211, (uint8_t)161, (uint8_t)43, (uint8_t)207, (uint8_t)56, (uint8_t)227, (uint8_t)163, (uint8_t)52, (uint8_t)48, (uint8_t)89, (uint8_t)44, (uint8_t)232, (uint8_t)33, (uint8_t)203, (uint8_t)165, (uint8_t)62, (uint8_t)158, (uint8_t)70, (uint8_t)208, (uint8_t)15, (uint8_t)134, (uint8_t)1, (uint8_t)164, (uint8_t)121, (uint8_t)186, (uint8_t)240, (uint8_t)12, (uint8_t)8, (uint8_t)102, (uint8_t)247, (uint8_t)45, (uint8_t)130, (uint8_t)219, (uint8_t)51, (uint8_t)237, (uint8_t)33, (uint8_t)253, (uint8_t)27, (uint8_t)41, (uint8_t)249, (uint8_t)107, (uint8_t)161, (uint8_t)219, (uint8_t)201, (uint8_t)88, (uint8_t)11, (uint8_t)188, (uint8_t)166, (uint8_t)187, (uint8_t)208, (uint8_t)0, (uint8_t)117, (uint8_t)123, (uint8_t)99, (uint8_t)245, (uint8_t)174, (uint8_t)245, (uint8_t)79, (uint8_t)37, (uint8_t)29, (uint8_t)61, (uint8_t)204, (uint8_t)9, (uint8_t)226, (uint8_t)131, (uint8_t)74, (uint8_t)202, (uint8_t)169, (uint8_t)51, (uint8_t)3, (uint8_t)140, (uint8_t)3, (uint8_t)91, (uint8_t)236, (uint8_t)192, (uint8_t)19, (uint8_t)21, (uint8_t)224, (uint8_t)205, (uint8_t)59, (uint8_t)144, (uint8_t)112, (uint8_t)157, (uint8_t)78, (uint8_t)169, (uint8_t)45, (uint8_t)156, (uint8_t)201, (uint8_t)208, (uint8_t)247, (uint8_t)7} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)78);
    {
        uint8_t exemplary[] =  {(uint8_t)130, (uint8_t)108, (uint8_t)180, (uint8_t)204, (uint8_t)14, (uint8_t)213, (uint8_t)63, (uint8_t)70, (uint8_t)75, (uint8_t)168, (uint8_t)169, (uint8_t)1, (uint8_t)46, (uint8_t)215, (uint8_t)131, (uint8_t)70, (uint8_t)63, (uint8_t)144, (uint8_t)76, (uint8_t)78, (uint8_t)178, (uint8_t)149, (uint8_t)201, (uint8_t)225, (uint8_t)87, (uint8_t)240, (uint8_t)99, (uint8_t)39, (uint8_t)188, (uint8_t)223, (uint8_t)108, (uint8_t)140, (uint8_t)145, (uint8_t)95, (uint8_t)131, (uint8_t)106, (uint8_t)108, (uint8_t)16, (uint8_t)115, (uint8_t)164, (uint8_t)88, (uint8_t)68, (uint8_t)16, (uint8_t)112, (uint8_t)208, (uint8_t)216, (uint8_t)179, (uint8_t)211, (uint8_t)140, (uint8_t)223, (uint8_t)112, (uint8_t)230, (uint8_t)228, (uint8_t)192, (uint8_t)93, (uint8_t)107, (uint8_t)192, (uint8_t)156, (uint8_t)4, (uint8_t)79, (uint8_t)43, (uint8_t)252, (uint8_t)191, (uint8_t)68, (uint8_t)136, (uint8_t)121, (uint8_t)243, (uint8_t)246, (uint8_t)81, (uint8_t)200, (uint8_t)109, (uint8_t)95, (uint8_t)71, (uint8_t)199, (uint8_t)19, (uint8_t)120, (uint8_t)54, (uint8_t)100, (uint8_t)201, (uint8_t)0, (uint8_t)121, (uint8_t)86, (uint8_t)247, (uint8_t)193, (uint8_t)69, (uint8_t)92, (uint8_t)80, (uint8_t)11, (uint8_t)108, (uint8_t)153, (uint8_t)213, (uint8_t)24, (uint8_t)18, (uint8_t)148, (uint8_t)220, (uint8_t)55, (uint8_t)112, (uint8_t)215, (uint8_t)90, (uint8_t)246, (uint8_t)237, (uint8_t)93, (uint8_t)114, (uint8_t)228, (uint8_t)139, (uint8_t)47, (uint8_t)142, (uint8_t)80, (uint8_t)200, (uint8_t)115, (uint8_t)157, (uint8_t)33, (uint8_t)251, (uint8_t)112, (uint8_t)178, (uint8_t)112, (uint8_t)45, (uint8_t)85, (uint8_t)111, (uint8_t)120} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float) -1.1983486E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)27338);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)4175788980L);
    assert(p143_press_diff_GET(pack) == (float)2.0969235E38F);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p144_custom_state_GET(pack) == (uint64_t)627703799685721512L);
    {
        float exemplary[] =  {2.4460973E38F, 5.3681114E37F, -2.4519705E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.856969E38F, 4.317438E37F, -2.5425582E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.9995608E38F, 9.143456E37F, 1.4209028E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.7470009E38F, -2.6165886E38F, 3.2525385E38F, -3.2264085E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.523387E38F, -2.181224E37F, 8.302649E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)3.166199E38F);
    assert(p144_lat_GET(pack) == (int32_t) -592812632);
    assert(p144_lon_GET(pack) == (int32_t)754066331);
    assert(p144_timestamp_GET(pack) == (uint64_t)2355961586016763788L);
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_z_vel_GET(pack) == (float)2.8887506E38F);
    assert(p146_z_acc_GET(pack) == (float)1.3378068E38F);
    assert(p146_airspeed_GET(pack) == (float)3.313076E38F);
    assert(p146_y_acc_GET(pack) == (float)1.1610373E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)276873188634234135L);
    assert(p146_y_pos_GET(pack) == (float)3.8577992E37F);
    {
        float exemplary[] =  {-7.403642E37F, -3.3075905E38F, -6.895184E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_vel_GET(pack) == (float)2.8727403E38F);
    {
        float exemplary[] =  {6.22984E37F, -1.5724663E38F, 1.9821358E38F, -9.892832E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_pos_GET(pack) == (float) -1.3636583E38F);
    assert(p146_yaw_rate_GET(pack) == (float)1.4470273E38F);
    assert(p146_z_pos_GET(pack) == (float)2.3418026E38F);
    assert(p146_roll_rate_GET(pack) == (float)2.1447538E37F);
    assert(p146_x_acc_GET(pack) == (float)2.7588225E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -9.945516E37F);
    {
        float exemplary[] =  {2.2258663E38F, -2.4024944E38F, 2.6694326E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float) -1.9777915E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)31084);
    assert(p147_energy_consumed_GET(pack) == (int32_t)55248091);
    assert(p147_current_consumed_GET(pack) == (int32_t)751416974);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD);
    {
        uint16_t exemplary[] =  {(uint16_t)13384, (uint16_t)45569, (uint16_t)15615, (uint16_t)55444, (uint16_t)8628, (uint16_t)9982, (uint16_t)45475, (uint16_t)42987, (uint16_t)18725, (uint16_t)37811} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -16344);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -93);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)170, (uint8_t)82, (uint8_t)230, (uint8_t)56, (uint8_t)233, (uint8_t)230, (uint8_t)118, (uint8_t)75, (uint8_t)173, (uint8_t)158, (uint8_t)58, (uint8_t)104, (uint8_t)201, (uint8_t)193, (uint8_t)26, (uint8_t)230, (uint8_t)137, (uint8_t)27} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)1228316375L);
    assert(p148_uid_GET(pack) == (uint64_t)6771966564808546969L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)2340035441L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)41553);
    assert(p148_capabilities_GET(pack) == (e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1025672981L);
    {
        uint8_t exemplary[] =  {(uint8_t)149, (uint8_t)23, (uint8_t)163, (uint8_t)61, (uint8_t)117, (uint8_t)128, (uint8_t)104, (uint8_t)116} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)108, (uint8_t)233, (uint8_t)20, (uint8_t)100, (uint8_t)3, (uint8_t)124, (uint8_t)182} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)2649506992L);
    {
        uint8_t exemplary[] =  {(uint8_t)6, (uint8_t)168, (uint8_t)249, (uint8_t)21, (uint8_t)78, (uint8_t)211, (uint8_t)5, (uint8_t)43} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)17162);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_y_TRY(ph) == (float)2.416411E38F);
    assert(p149_x_TRY(ph) == (float) -7.170414E37F);
    assert(p149_angle_x_GET(pack) == (float) -2.366068E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_time_usec_GET(pack) == (uint64_t)7171653044739120128L);
    {
        float exemplary[] =  {3.236762E38F, -2.2086911E38F, 9.720465E37F, 3.2245752E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)78);
    assert(p149_z_TRY(ph) == (float)2.8451686E38F);
    assert(p149_size_y_GET(pack) == (float)3.2916353E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p149_size_x_GET(pack) == (float) -1.5521891E38F);
    assert(p149_distance_GET(pack) == (float) -9.04913E37F);
    assert(p149_angle_y_GET(pack) == (float) -1.6752369E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_CommunicationChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_batVolt_GET(pack) == (uint16_t)(uint16_t)17941);
    assert(p170_sensLoad_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p170_ctrlLoad_GET(pack) == (uint8_t)(uint8_t)105);
};


void c_CommunicationChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_axBias_GET(pack) == (float)1.3166431E38F);
    assert(p172_gzBias_GET(pack) == (float)7.341513E37F);
    assert(p172_ayBias_GET(pack) == (float) -3.2870851E38F);
    assert(p172_azBias_GET(pack) == (float)2.3712946E38F);
    assert(p172_gyBias_GET(pack) == (float) -1.1456557E38F);
    assert(p172_gxBias_GET(pack) == (float)1.0470745E38F);
};


void c_CommunicationChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_diagFl1_GET(pack) == (float) -2.8129893E38F);
    assert(p173_diagSh1_GET(pack) == (int16_t)(int16_t)22251);
    assert(p173_diagFl3_GET(pack) == (float) -8.804417E37F);
    assert(p173_diagSh2_GET(pack) == (int16_t)(int16_t) -14224);
    assert(p173_diagFl2_GET(pack) == (float)1.3951366E38F);
    assert(p173_diagSh3_GET(pack) == (int16_t)(int16_t)8482);
};


void c_CommunicationChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_u_m_GET(pack) == (float) -1.95483E38F);
    assert(p176_phi_c_GET(pack) == (float)3.2726863E38F);
    assert(p176_toWP_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p176_h_c_GET(pack) == (uint16_t)(uint16_t)25870);
    assert(p176_dist2Go_GET(pack) == (float)2.1877445E38F);
    assert(p176_ay_body_GET(pack) == (float)3.3508044E38F);
    assert(p176_theta_c_GET(pack) == (float) -2.163432E38F);
    assert(p176_totalDist_GET(pack) == (float) -2.83225E38F);
    assert(p176_psiDot_c_GET(pack) == (float)7.2772124E37F);
    assert(p176_fromWP_GET(pack) == (uint8_t)(uint8_t)201);
};


void c_CommunicationChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_fl_3_GET(pack) == (float)3.2931802E38F);
    assert(p177_fl_5_GET(pack) == (float)1.4994007E38F);
    assert(p177_fl_1_GET(pack) == (float)1.2769154E38F);
    assert(p177_fl_6_GET(pack) == (float) -1.9883373E38F);
    assert(p177_fl_2_GET(pack) == (float)1.2383974E38F);
    assert(p177_fl_4_GET(pack) == (float) -2.7703782E38F);
};


void c_CommunicationChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_day_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p179_percentUsed_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p179_clockStat_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p179_useSat_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p179_sigUsedMask_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p179_visSat_GET(pack) == (uint8_t)(uint8_t)131);
    assert(p179_month_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p179_GppGl_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p179_min_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p179_year_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p179_hour_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p179_sec_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_CommunicationChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_uCommand_GET(pack) == (float) -1.1837362E38F);
    assert(p180_hCommand_GET(pack) == (float)2.2638E37F);
    assert(p180_target_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p180_rCommand_GET(pack) == (float)2.4772779E38F);
};


void c_CommunicationChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_bitfieldPt_GET(pack) == (uint16_t)(uint16_t)46960);
    assert(p181_target_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_zoom_GET(pack) == (int8_t)(int8_t)14);
    assert(p184_target_GET(pack) == (uint8_t)(uint8_t)127);
    assert(p184_tilt_GET(pack) == (int8_t)(int8_t) -47);
    assert(p184_pan_GET(pack) == (int8_t)(int8_t)31);
    assert(p184_moveHome_GET(pack) == (int8_t)(int8_t) -104);
};


void c_CommunicationChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_mControl_GET(pack) == (float) -1.4701675E38F);
    assert(p185_target_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p185_bControl_GET(pack) == (float) -2.4601088E38F);
    assert(p185_idSurface_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_longitude_GET(pack) == (float)1.5686384E37F);
    assert(p186_target_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p186_latitude_GET(pack) == (float) -1.6373453E38F);
};


void c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_target_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p188_idOrder_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p188_order_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_CommunicationChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    assert(p189_option3_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p189_height_GET(pack) == (float)2.0739581E38F);
    assert(p189_longitude_GET(pack) == (float) -6.074059E37F);
    assert(p189_option2_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p189_target_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p189_option1_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p189_latitude_GET(pack) == (float) -1.5179171E38F);
};


void c_CommunicationChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_reading2_GET(pack) == (uint16_t)(uint16_t)5562);
    assert(p191_voltage_GET(pack) == (uint16_t)(uint16_t)11692);
    assert(p191_r2Type_GET(pack) == (uint8_t)(uint8_t)197);
};


void c_CommunicationChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_tilt_GET(pack) == (int16_t)(int16_t)17176);
    assert(p192_zoom_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p192_pan_GET(pack) == (int16_t)(int16_t)495);
};


void c_CommunicationChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_target_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p193_course_GET(pack) == (float) -4.364703E36F);
    assert(p193_longitude_GET(pack) == (float) -8.1235494E36F);
    assert(p193_latitude_GET(pack) == (float)1.3339058E38F);
    assert(p193_speed_GET(pack) == (float)1.7351549E38F);
    assert(p193_altitude_GET(pack) == (float) -2.1225223E38F);
};


void c_CommunicationChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_magVar_GET(pack) == (float)2.4075656E38F);
    assert(p194_posStatus_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p194_modeInd_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p194_gpsQuality_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p194_magDir_GET(pack) == (int8_t)(int8_t)50);
    assert(p194_msgsType_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p194_csFails_GET(pack) == (uint16_t)(uint16_t)172);
};


void c_CommunicationChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    assert(p195_timeStatus_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p195_solStatus_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p195_receiverStatus_GET(pack) == (uint32_t)496074806L);
    assert(p195_posSolAge_GET(pack) == (float)4.6098895E37F);
    assert(p195_posType_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p195_velType_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p195_csFails_GET(pack) == (uint16_t)(uint16_t)29890);
};


void c_CommunicationChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    assert(p196_float1_GET(pack) == (float)1.9607516E38F);
    assert(p196_int1_GET(pack) == (int16_t)(int16_t)5253);
    assert(p196_char1_GET(pack) == (int8_t)(int8_t) -44);
    assert(p196_float2_GET(pack) == (float) -2.6506899E38F);
};


void c_CommunicationChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    assert(p197_version_GET(pack) == (uint32_t)257663186L);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_ratio_GET(pack) == (float)5.8033283E37F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)8.529798E37F);
    assert(p230_mag_ratio_GET(pack) == (float) -4.8233984E37F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)1.6140117E37F);
    assert(p230_flags_GET(pack) == (e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS));
    assert(p230_vel_ratio_GET(pack) == (float)2.0955638E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.0908332E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -6.3105477E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)9208457423408500L);
    assert(p230_hagl_ratio_GET(pack) == (float) -2.4584455E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_vert_GET(pack) == (float)3.8379952E37F);
    assert(p231_wind_z_GET(pack) == (float)1.3281879E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -1.7053923E38F);
    assert(p231_wind_x_GET(pack) == (float) -3.166215E37F);
    assert(p231_wind_alt_GET(pack) == (float) -8.894556E37F);
    assert(p231_wind_y_GET(pack) == (float) -1.088157E38F);
    assert(p231_var_horiz_GET(pack) == (float) -2.2498153E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)2839261002752190233L);
    assert(p231_vert_accuracy_GET(pack) == (float) -1.5604916E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_ignore_flags_GET(pack) == (e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)4177);
    assert(p232_vn_GET(pack) == (float) -1.398853E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)8.520085E37F);
    assert(p232_lat_GET(pack) == (int32_t)2029335380);
    assert(p232_hdop_GET(pack) == (float) -1.8329702E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p232_horiz_accuracy_GET(pack) == (float)3.0317978E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2686258151L);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p232_ve_GET(pack) == (float)2.7527102E38F);
    assert(p232_speed_accuracy_GET(pack) == (float)8.651104E36F);
    assert(p232_alt_GET(pack) == (float) -3.0147989E38F);
    assert(p232_vd_GET(pack) == (float) -1.306838E38F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p232_lon_GET(pack) == (int32_t)308225254);
    assert(p232_time_usec_GET(pack) == (uint64_t)6205240408801939510L);
    assert(p232_vdop_GET(pack) == (float) -2.3996587E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)157, (uint8_t)87, (uint8_t)0, (uint8_t)7, (uint8_t)49, (uint8_t)108, (uint8_t)134, (uint8_t)174, (uint8_t)159, (uint8_t)156, (uint8_t)56, (uint8_t)105, (uint8_t)127, (uint8_t)106, (uint8_t)80, (uint8_t)170, (uint8_t)211, (uint8_t)109, (uint8_t)33, (uint8_t)200, (uint8_t)139, (uint8_t)49, (uint8_t)145, (uint8_t)176, (uint8_t)167, (uint8_t)229, (uint8_t)224, (uint8_t)221, (uint8_t)6, (uint8_t)37, (uint8_t)113, (uint8_t)141, (uint8_t)250, (uint8_t)33, (uint8_t)93, (uint8_t)144, (uint8_t)173, (uint8_t)141, (uint8_t)63, (uint8_t)59, (uint8_t)164, (uint8_t)178, (uint8_t)254, (uint8_t)71, (uint8_t)228, (uint8_t)246, (uint8_t)107, (uint8_t)140, (uint8_t)230, (uint8_t)204, (uint8_t)168, (uint8_t)226, (uint8_t)24, (uint8_t)44, (uint8_t)234, (uint8_t)238, (uint8_t)152, (uint8_t)85, (uint8_t)39, (uint8_t)208, (uint8_t)172, (uint8_t)112, (uint8_t)59, (uint8_t)94, (uint8_t)114, (uint8_t)156, (uint8_t)24, (uint8_t)145, (uint8_t)145, (uint8_t)38, (uint8_t)138, (uint8_t)152, (uint8_t)77, (uint8_t)129, (uint8_t)55, (uint8_t)252, (uint8_t)235, (uint8_t)43, (uint8_t)36, (uint8_t)94, (uint8_t)169, (uint8_t)23, (uint8_t)12, (uint8_t)22, (uint8_t)166, (uint8_t)214, (uint8_t)112, (uint8_t)20, (uint8_t)175, (uint8_t)203, (uint8_t)244, (uint8_t)56, (uint8_t)178, (uint8_t)40, (uint8_t)156, (uint8_t)213, (uint8_t)179, (uint8_t)222, (uint8_t)68, (uint8_t)22, (uint8_t)219, (uint8_t)65, (uint8_t)237, (uint8_t)127, (uint8_t)206, (uint8_t)101, (uint8_t)50, (uint8_t)254, (uint8_t)209, (uint8_t)179, (uint8_t)162, (uint8_t)154, (uint8_t)148, (uint8_t)145, (uint8_t)251, (uint8_t)41, (uint8_t)15, (uint8_t)54, (uint8_t)99, (uint8_t)246, (uint8_t)250, (uint8_t)97, (uint8_t)213, (uint8_t)67, (uint8_t)242, (uint8_t)26, (uint8_t)107, (uint8_t)152, (uint8_t)207, (uint8_t)19, (uint8_t)84, (uint8_t)132, (uint8_t)59, (uint8_t)245, (uint8_t)98, (uint8_t)188, (uint8_t)152, (uint8_t)252, (uint8_t)78, (uint8_t)150, (uint8_t)178, (uint8_t)143, (uint8_t)129, (uint8_t)129, (uint8_t)50, (uint8_t)69, (uint8_t)48, (uint8_t)9, (uint8_t)184, (uint8_t)138, (uint8_t)38, (uint8_t)44, (uint8_t)73, (uint8_t)90, (uint8_t)164, (uint8_t)155, (uint8_t)131, (uint8_t)233, (uint8_t)193, (uint8_t)254, (uint8_t)131, (uint8_t)96, (uint8_t)226, (uint8_t)217, (uint8_t)133, (uint8_t)117, (uint8_t)126, (uint8_t)155, (uint8_t)222, (uint8_t)16, (uint8_t)120, (uint8_t)100, (uint8_t)145, (uint8_t)18, (uint8_t)137, (uint8_t)176, (uint8_t)187, (uint8_t)187, (uint8_t)116, (uint8_t)217} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)15328);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)114);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p234_base_mode_GET(pack) == (e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED));
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)34769);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)12319);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -6604);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -29078);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1295893122L);
    assert(p234_longitude_GET(pack) == (int32_t)2061642597);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)57105);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -26780);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -53);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -80);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p234_latitude_GET(pack) == (int32_t) -1845363456);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)87);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_vibration_y_GET(pack) == (float) -1.9035678E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)3898909041L);
    assert(p241_vibration_x_GET(pack) == (float)8.88594E37F);
    assert(p241_time_usec_GET(pack) == (uint64_t)7753734045770659766L);
    assert(p241_vibration_z_GET(pack) == (float)1.6957717E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)1628964820L);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3396370242L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_latitude_GET(pack) == (int32_t) -726004260);
    assert(p242_altitude_GET(pack) == (int32_t) -895546505);
    assert(p242_approach_z_GET(pack) == (float)1.9080141E38F);
    assert(p242_approach_y_GET(pack) == (float)7.62881E36F);
    assert(p242_z_GET(pack) == (float) -8.632061E37F);
    assert(p242_longitude_GET(pack) == (int32_t)1021630051);
    {
        float exemplary[] =  {-1.6978115E38F, 2.8245178E38F, -8.317952E37F, 3.503049E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_x_GET(pack) == (float)1.2591977E38F);
    assert(p242_y_GET(pack) == (float)2.3926528E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)3655407604944890551L);
    assert(p242_approach_x_GET(pack) == (float)2.5482511E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_approach_y_GET(pack) == (float) -1.7105381E38F);
    assert(p243_z_GET(pack) == (float)3.1667113E38F);
    assert(p243_approach_x_GET(pack) == (float) -1.2765116E38F);
    assert(p243_x_GET(pack) == (float) -1.9187488E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.5212334E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p243_latitude_GET(pack) == (int32_t)1824677614);
    {
        float exemplary[] =  {1.9325936E38F, -1.4690068E38F, 3.0035275E38F, -2.1767672E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_time_usec_TRY(ph) == (uint64_t)3455242827807158572L);
    assert(p243_y_GET(pack) == (float)1.5637175E37F);
    assert(p243_longitude_GET(pack) == (int32_t)1836327592);
    assert(p243_altitude_GET(pack) == (int32_t)1094736259);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)20543);
    assert(p244_interval_us_GET(pack) == (int32_t) -854985435);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)25749);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)46487);
    assert(p246_flags_GET(pack) == (e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                                    e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN));
    assert(p246_lon_GET(pack) == (int32_t) -1118690624);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT);
    assert(p246_callsign_LEN(ph) == 4);
    {
        char16_t * exemplary = u"upjj";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_altitude_GET(pack) == (int32_t)1496757153);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)29947);
    assert(p246_lat_GET(pack) == (int32_t)375246231);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)63451827L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)16201);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_altitude_minimum_delta_GET(pack) == (float)2.816482E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_threat_level_GET(pack) == (e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW |
                                           e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH));
    assert(p247_id_GET(pack) == (uint32_t)586448800L);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.3550403E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.6636986E38F);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)60);
    {
        uint8_t exemplary[] =  {(uint8_t)227, (uint8_t)125, (uint8_t)176, (uint8_t)21, (uint8_t)69, (uint8_t)233, (uint8_t)105, (uint8_t)156, (uint8_t)206, (uint8_t)179, (uint8_t)89, (uint8_t)57, (uint8_t)16, (uint8_t)226, (uint8_t)173, (uint8_t)90, (uint8_t)183, (uint8_t)112, (uint8_t)185, (uint8_t)9, (uint8_t)248, (uint8_t)171, (uint8_t)42, (uint8_t)117, (uint8_t)105, (uint8_t)170, (uint8_t)176, (uint8_t)60, (uint8_t)246, (uint8_t)32, (uint8_t)12, (uint8_t)2, (uint8_t)107, (uint8_t)92, (uint8_t)171, (uint8_t)213, (uint8_t)20, (uint8_t)34, (uint8_t)84, (uint8_t)208, (uint8_t)91, (uint8_t)157, (uint8_t)31, (uint8_t)194, (uint8_t)99, (uint8_t)144, (uint8_t)108, (uint8_t)185, (uint8_t)55, (uint8_t)182, (uint8_t)21, (uint8_t)252, (uint8_t)34, (uint8_t)222, (uint8_t)169, (uint8_t)254, (uint8_t)37, (uint8_t)208, (uint8_t)72, (uint8_t)204, (uint8_t)104, (uint8_t)49, (uint8_t)84, (uint8_t)159, (uint8_t)249, (uint8_t)110, (uint8_t)159, (uint8_t)160, (uint8_t)102, (uint8_t)117, (uint8_t)50, (uint8_t)212, (uint8_t)26, (uint8_t)191, (uint8_t)38, (uint8_t)15, (uint8_t)26, (uint8_t)171, (uint8_t)217, (uint8_t)70, (uint8_t)102, (uint8_t)135, (uint8_t)208, (uint8_t)214, (uint8_t)14, (uint8_t)187, (uint8_t)200, (uint8_t)43, (uint8_t)23, (uint8_t)32, (uint8_t)101, (uint8_t)9, (uint8_t)51, (uint8_t)136, (uint8_t)210, (uint8_t)107, (uint8_t)214, (uint8_t)255, (uint8_t)31, (uint8_t)107, (uint8_t)59, (uint8_t)16, (uint8_t)52, (uint8_t)144, (uint8_t)218, (uint8_t)95, (uint8_t)31, (uint8_t)1, (uint8_t)15, (uint8_t)231, (uint8_t)209, (uint8_t)211, (uint8_t)90, (uint8_t)236, (uint8_t)61, (uint8_t)18, (uint8_t)140, (uint8_t)200, (uint8_t)29, (uint8_t)223, (uint8_t)120, (uint8_t)230, (uint8_t)210, (uint8_t)49, (uint8_t)101, (uint8_t)144, (uint8_t)86, (uint8_t)61, (uint8_t)228, (uint8_t)212, (uint8_t)131, (uint8_t)4, (uint8_t)120, (uint8_t)183, (uint8_t)208, (uint8_t)82, (uint8_t)38, (uint8_t)233, (uint8_t)96, (uint8_t)250, (uint8_t)84, (uint8_t)157, (uint8_t)208, (uint8_t)61, (uint8_t)220, (uint8_t)255, (uint8_t)214, (uint8_t)107, (uint8_t)64, (uint8_t)0, (uint8_t)5, (uint8_t)243, (uint8_t)80, (uint8_t)32, (uint8_t)57, (uint8_t)161, (uint8_t)79, (uint8_t)28, (uint8_t)222, (uint8_t)129, (uint8_t)193, (uint8_t)57, (uint8_t)110, (uint8_t)223, (uint8_t)51, (uint8_t)5, (uint8_t)10, (uint8_t)146, (uint8_t)24, (uint8_t)75, (uint8_t)216, (uint8_t)140, (uint8_t)230, (uint8_t)85, (uint8_t)185, (uint8_t)100, (uint8_t)175, (uint8_t)104, (uint8_t)210, (uint8_t)195, (uint8_t)245, (uint8_t)130, (uint8_t)67, (uint8_t)154, (uint8_t)201, (uint8_t)130, (uint8_t)122, (uint8_t)62, (uint8_t)146, (uint8_t)110, (uint8_t)249, (uint8_t)53, (uint8_t)102, (uint8_t)176, (uint8_t)157, (uint8_t)79, (uint8_t)239, (uint8_t)152, (uint8_t)92, (uint8_t)26, (uint8_t)0, (uint8_t)93, (uint8_t)186, (uint8_t)188, (uint8_t)9, (uint8_t)11, (uint8_t)127, (uint8_t)32, (uint8_t)168, (uint8_t)17, (uint8_t)38, (uint8_t)223, (uint8_t)232, (uint8_t)45, (uint8_t)65, (uint8_t)2, (uint8_t)189, (uint8_t)100, (uint8_t)46, (uint8_t)240, (uint8_t)139, (uint8_t)165, (uint8_t)228, (uint8_t)37, (uint8_t)176, (uint8_t)169, (uint8_t)116, (uint8_t)140, (uint8_t)21, (uint8_t)253, (uint8_t)157, (uint8_t)123, (uint8_t)200, (uint8_t)120, (uint8_t)207, (uint8_t)14, (uint8_t)147, (uint8_t)15, (uint8_t)159, (uint8_t)227, (uint8_t)47, (uint8_t)141, (uint8_t)188, (uint8_t)168, (uint8_t)71, (uint8_t)159, (uint8_t)59, (uint8_t)25, (uint8_t)197} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)11916);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)113);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)16627);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)95);
    {
        int8_t exemplary[] =  {(int8_t) -57, (int8_t)112, (int8_t)106, (int8_t)65, (int8_t) -36, (int8_t) -89, (int8_t) -26, (int8_t)8, (int8_t) -13, (int8_t)47, (int8_t)54, (int8_t) -94, (int8_t) -14, (int8_t) -62, (int8_t)60, (int8_t) -120, (int8_t) -18, (int8_t)50, (int8_t) -2, (int8_t) -74, (int8_t)35, (int8_t) -100, (int8_t) -24, (int8_t) -124, (int8_t)8, (int8_t) -86, (int8_t)25, (int8_t)32, (int8_t) -105, (int8_t)84, (int8_t)106, (int8_t)7} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float)7.739887E36F);
    assert(p250_x_GET(pack) == (float) -1.1351017E38F);
    assert(p250_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"nptunwuW";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)3889429453381407108L);
    assert(p250_z_GET(pack) == (float) -2.971191E38F);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float) -2.4793783E38F);
    assert(p251_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"ryxf";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2837358595L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -1615440492);
    assert(p252_name_LEN(ph) == 1);
    {
        char16_t * exemplary = u"a";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3272189847L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 19);
    {
        char16_t * exemplary = u"bkfxmhufvFffvxiKjTq";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_NOTICE);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p254_value_GET(pack) == (float)1.1443905E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1288113467L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)3804130512939255754L);
    {
        uint8_t exemplary[] =  {(uint8_t)198, (uint8_t)116, (uint8_t)45, (uint8_t)247, (uint8_t)180, (uint8_t)55, (uint8_t)164, (uint8_t)240, (uint8_t)29, (uint8_t)53, (uint8_t)133, (uint8_t)40, (uint8_t)37, (uint8_t)90, (uint8_t)4, (uint8_t)135, (uint8_t)182, (uint8_t)9, (uint8_t)32, (uint8_t)116, (uint8_t)86, (uint8_t)110, (uint8_t)192, (uint8_t)76, (uint8_t)225, (uint8_t)138, (uint8_t)167, (uint8_t)174, (uint8_t)80, (uint8_t)194, (uint8_t)27, (uint8_t)123} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)159);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)24836270L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)547027225L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)170);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p258_tune_LEN(ph) == 6);
    {
        char16_t * exemplary = u"xVtRxr";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_cam_definition_uri_LEN(ph) == 130);
    {
        char16_t * exemplary = u"nDoBdhaxjrizhmeyxUUkCfZzzgukrguoQutoskkjfxopkkbfdlqknbxhlwxluyzhxryygxnkWxtxiflgxvzdpnynhgGTaLeNdcgcletqhHwirdmkhkzcadgvjuhfrvvaEq";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 260);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_firmware_version_GET(pack) == (uint32_t)311101875L);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p259_sensor_size_h_GET(pack) == (float) -3.0575876E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)139, (uint8_t)68, (uint8_t)125, (uint8_t)149, (uint8_t)150, (uint8_t)169, (uint8_t)146, (uint8_t)188, (uint8_t)160, (uint8_t)122, (uint8_t)118, (uint8_t)101, (uint8_t)241, (uint8_t)84, (uint8_t)105, (uint8_t)230, (uint8_t)97, (uint8_t)55, (uint8_t)213, (uint8_t)45, (uint8_t)105, (uint8_t)46, (uint8_t)34, (uint8_t)208, (uint8_t)166, (uint8_t)193, (uint8_t)2, (uint8_t)170, (uint8_t)20, (uint8_t)83, (uint8_t)190, (uint8_t)20} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)786836195L);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)51716);
    assert(p259_focal_length_GET(pack) == (float)2.9310163E38F);
    assert(p259_sensor_size_v_GET(pack) == (float) -1.4299655E38F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)34083);
    {
        uint8_t exemplary[] =  {(uint8_t)56, (uint8_t)125, (uint8_t)28, (uint8_t)218, (uint8_t)62, (uint8_t)17, (uint8_t)212, (uint8_t)14, (uint8_t)119, (uint8_t)238, (uint8_t)198, (uint8_t)18, (uint8_t)138, (uint8_t)143, (uint8_t)189, (uint8_t)251, (uint8_t)123, (uint8_t)43, (uint8_t)190, (uint8_t)65, (uint8_t)115, (uint8_t)92, (uint8_t)254, (uint8_t)254, (uint8_t)75, (uint8_t)206, (uint8_t)163, (uint8_t)108, (uint8_t)129, (uint8_t)184, (uint8_t)147, (uint8_t)181} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_flags_GET(pack) == (e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)33931);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3254092763L);
    assert(p260_mode_id_GET(pack) == (e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY));
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2832701868L);
    assert(p261_write_speed_GET(pack) == (float) -1.7756785E38F);
    assert(p261_used_capacity_GET(pack) == (float)2.5781693E38F);
    assert(p261_total_capacity_GET(pack) == (float) -3.0473856E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p261_available_capacity_GET(pack) == (float) -1.3462915E38F);
    assert(p261_read_speed_GET(pack) == (float)2.9386455E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_image_interval_GET(pack) == (float)4.453051E36F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)1432752928L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p262_available_capacity_GET(pack) == (float)7.27429E37F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)1780001994L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p263_lat_GET(pack) == (int32_t) -988653941);
    assert(p263_lon_GET(pack) == (int32_t)1751530392);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1120284865L);
    {
        float exemplary[] =  {-3.1696979E38F, 3.3108026E38F, -2.474192E38F, 1.1462996E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_file_url_LEN(ph) == 91);
    {
        char16_t * exemplary = u"djzvulsxrSwfblsyieuiraoyrrxzEuzbmcypvTmvjhqoybxTvPjZggvvkqapdSmjeEtjwlhgrhlvztxiezfrwbklxjy";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 182);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_image_index_GET(pack) == (int32_t) -1145425973);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)98);
    assert(p263_time_utc_GET(pack) == (uint64_t)703317381551829819L);
    assert(p263_alt_GET(pack) == (int32_t)1104459243);
    assert(p263_relative_alt_GET(pack) == (int32_t) -1452437474);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)2094253198347421623L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)1691260246187801663L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)5446581557599514829L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3489916229L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float) -6.358186E37F);
    assert(p265_yaw_GET(pack) == (float)2.726115E38F);
    assert(p265_roll_GET(pack) == (float) -3.3782696E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)452821304L);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)23762);
    {
        uint8_t exemplary[] =  {(uint8_t)197, (uint8_t)190, (uint8_t)221, (uint8_t)188, (uint8_t)7, (uint8_t)172, (uint8_t)231, (uint8_t)31, (uint8_t)208, (uint8_t)216, (uint8_t)36, (uint8_t)46, (uint8_t)254, (uint8_t)205, (uint8_t)224, (uint8_t)217, (uint8_t)7, (uint8_t)16, (uint8_t)96, (uint8_t)8, (uint8_t)207, (uint8_t)120, (uint8_t)33, (uint8_t)243, (uint8_t)186, (uint8_t)61, (uint8_t)230, (uint8_t)154, (uint8_t)46, (uint8_t)41, (uint8_t)173, (uint8_t)15, (uint8_t)146, (uint8_t)19, (uint8_t)2, (uint8_t)127, (uint8_t)131, (uint8_t)114, (uint8_t)162, (uint8_t)3, (uint8_t)171, (uint8_t)86, (uint8_t)19, (uint8_t)83, (uint8_t)160, (uint8_t)105, (uint8_t)208, (uint8_t)62, (uint8_t)177, (uint8_t)222, (uint8_t)35, (uint8_t)175, (uint8_t)48, (uint8_t)169, (uint8_t)112, (uint8_t)161, (uint8_t)218, (uint8_t)12, (uint8_t)25, (uint8_t)34, (uint8_t)63, (uint8_t)60, (uint8_t)47, (uint8_t)133, (uint8_t)240, (uint8_t)45, (uint8_t)229, (uint8_t)156, (uint8_t)145, (uint8_t)197, (uint8_t)97, (uint8_t)138, (uint8_t)26, (uint8_t)30, (uint8_t)132, (uint8_t)57, (uint8_t)122, (uint8_t)7, (uint8_t)28, (uint8_t)109, (uint8_t)248, (uint8_t)217, (uint8_t)191, (uint8_t)75, (uint8_t)248, (uint8_t)103, (uint8_t)140, (uint8_t)103, (uint8_t)234, (uint8_t)58, (uint8_t)176, (uint8_t)176, (uint8_t)195, (uint8_t)150, (uint8_t)120, (uint8_t)53, (uint8_t)213, (uint8_t)158, (uint8_t)45, (uint8_t)147, (uint8_t)88, (uint8_t)73, (uint8_t)59, (uint8_t)232, (uint8_t)227, (uint8_t)49, (uint8_t)62, (uint8_t)91, (uint8_t)155, (uint8_t)149, (uint8_t)68, (uint8_t)138, (uint8_t)94, (uint8_t)102, (uint8_t)204, (uint8_t)52, (uint8_t)23, (uint8_t)0, (uint8_t)235, (uint8_t)223, (uint8_t)150, (uint8_t)136, (uint8_t)192, (uint8_t)21, (uint8_t)134, (uint8_t)213, (uint8_t)134, (uint8_t)90, (uint8_t)60, (uint8_t)204, (uint8_t)201, (uint8_t)25, (uint8_t)177, (uint8_t)219, (uint8_t)165, (uint8_t)186, (uint8_t)138, (uint8_t)209, (uint8_t)64, (uint8_t)89, (uint8_t)127, (uint8_t)55, (uint8_t)245, (uint8_t)56, (uint8_t)201, (uint8_t)148, (uint8_t)141, (uint8_t)31, (uint8_t)125, (uint8_t)26, (uint8_t)213, (uint8_t)158, (uint8_t)60, (uint8_t)22, (uint8_t)87, (uint8_t)162, (uint8_t)145, (uint8_t)110, (uint8_t)84, (uint8_t)114, (uint8_t)154, (uint8_t)136, (uint8_t)230, (uint8_t)24, (uint8_t)104, (uint8_t)19, (uint8_t)0, (uint8_t)164, (uint8_t)161, (uint8_t)164, (uint8_t)191, (uint8_t)101, (uint8_t)147, (uint8_t)116, (uint8_t)131, (uint8_t)228, (uint8_t)37, (uint8_t)33, (uint8_t)160, (uint8_t)244, (uint8_t)86, (uint8_t)254, (uint8_t)228, (uint8_t)2, (uint8_t)124, (uint8_t)207, (uint8_t)233, (uint8_t)130, (uint8_t)66, (uint8_t)26, (uint8_t)232, (uint8_t)7, (uint8_t)189, (uint8_t)47, (uint8_t)119, (uint8_t)163, (uint8_t)116, (uint8_t)163, (uint8_t)70, (uint8_t)149, (uint8_t)6, (uint8_t)129, (uint8_t)254, (uint8_t)238, (uint8_t)112, (uint8_t)140, (uint8_t)116, (uint8_t)206, (uint8_t)227, (uint8_t)150, (uint8_t)206, (uint8_t)39, (uint8_t)240, (uint8_t)59, (uint8_t)16, (uint8_t)59, (uint8_t)38, (uint8_t)128, (uint8_t)40, (uint8_t)64, (uint8_t)174, (uint8_t)46, (uint8_t)55, (uint8_t)190, (uint8_t)1, (uint8_t)122, (uint8_t)120, (uint8_t)236, (uint8_t)73, (uint8_t)50, (uint8_t)255, (uint8_t)244, (uint8_t)93, (uint8_t)133, (uint8_t)110, (uint8_t)45, (uint8_t)181, (uint8_t)189, (uint8_t)133, (uint8_t)228, (uint8_t)58, (uint8_t)51, (uint8_t)59, (uint8_t)238, (uint8_t)158, (uint8_t)205, (uint8_t)122, (uint8_t)152, (uint8_t)161} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)6);
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)3596);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)165);
    {
        uint8_t exemplary[] =  {(uint8_t)142, (uint8_t)151, (uint8_t)113, (uint8_t)173, (uint8_t)95, (uint8_t)75, (uint8_t)207, (uint8_t)141, (uint8_t)222, (uint8_t)63, (uint8_t)46, (uint8_t)217, (uint8_t)22, (uint8_t)23, (uint8_t)93, (uint8_t)185, (uint8_t)111, (uint8_t)194, (uint8_t)118, (uint8_t)0, (uint8_t)50, (uint8_t)232, (uint8_t)250, (uint8_t)251, (uint8_t)217, (uint8_t)40, (uint8_t)42, (uint8_t)134, (uint8_t)31, (uint8_t)161, (uint8_t)250, (uint8_t)156, (uint8_t)38, (uint8_t)65, (uint8_t)43, (uint8_t)43, (uint8_t)71, (uint8_t)162, (uint8_t)189, (uint8_t)75, (uint8_t)46, (uint8_t)100, (uint8_t)13, (uint8_t)127, (uint8_t)7, (uint8_t)176, (uint8_t)220, (uint8_t)101, (uint8_t)232, (uint8_t)128, (uint8_t)220, (uint8_t)171, (uint8_t)0, (uint8_t)37, (uint8_t)112, (uint8_t)4, (uint8_t)204, (uint8_t)25, (uint8_t)174, (uint8_t)19, (uint8_t)239, (uint8_t)215, (uint8_t)94, (uint8_t)194, (uint8_t)185, (uint8_t)71, (uint8_t)88, (uint8_t)80, (uint8_t)180, (uint8_t)61, (uint8_t)86, (uint8_t)171, (uint8_t)5, (uint8_t)108, (uint8_t)159, (uint8_t)97, (uint8_t)21, (uint8_t)247, (uint8_t)149, (uint8_t)24, (uint8_t)207, (uint8_t)13, (uint8_t)237, (uint8_t)24, (uint8_t)3, (uint8_t)38, (uint8_t)71, (uint8_t)184, (uint8_t)33, (uint8_t)63, (uint8_t)224, (uint8_t)128, (uint8_t)235, (uint8_t)154, (uint8_t)171, (uint8_t)196, (uint8_t)243, (uint8_t)97, (uint8_t)70, (uint8_t)67, (uint8_t)211, (uint8_t)15, (uint8_t)167, (uint8_t)74, (uint8_t)108, (uint8_t)75, (uint8_t)170, (uint8_t)146, (uint8_t)166, (uint8_t)87, (uint8_t)2, (uint8_t)140, (uint8_t)71, (uint8_t)56, (uint8_t)56, (uint8_t)129, (uint8_t)30, (uint8_t)76, (uint8_t)128, (uint8_t)104, (uint8_t)31, (uint8_t)72, (uint8_t)117, (uint8_t)46, (uint8_t)155, (uint8_t)150, (uint8_t)199, (uint8_t)197, (uint8_t)76, (uint8_t)67, (uint8_t)240, (uint8_t)62, (uint8_t)224, (uint8_t)207, (uint8_t)31, (uint8_t)134, (uint8_t)184, (uint8_t)46, (uint8_t)130, (uint8_t)202, (uint8_t)102, (uint8_t)223, (uint8_t)121, (uint8_t)172, (uint8_t)8, (uint8_t)23, (uint8_t)159, (uint8_t)26, (uint8_t)10, (uint8_t)96, (uint8_t)219, (uint8_t)51, (uint8_t)25, (uint8_t)238, (uint8_t)171, (uint8_t)200, (uint8_t)67, (uint8_t)209, (uint8_t)90, (uint8_t)135, (uint8_t)254, (uint8_t)251, (uint8_t)192, (uint8_t)143, (uint8_t)254, (uint8_t)107, (uint8_t)7, (uint8_t)246, (uint8_t)129, (uint8_t)59, (uint8_t)249, (uint8_t)107, (uint8_t)152, (uint8_t)225, (uint8_t)64, (uint8_t)53, (uint8_t)27, (uint8_t)205, (uint8_t)17, (uint8_t)150, (uint8_t)172, (uint8_t)139, (uint8_t)179, (uint8_t)218, (uint8_t)41, (uint8_t)113, (uint8_t)131, (uint8_t)28, (uint8_t)237, (uint8_t)235, (uint8_t)107, (uint8_t)228, (uint8_t)72, (uint8_t)93, (uint8_t)203, (uint8_t)61, (uint8_t)137, (uint8_t)42, (uint8_t)252, (uint8_t)84, (uint8_t)44, (uint8_t)20, (uint8_t)6, (uint8_t)169, (uint8_t)52, (uint8_t)181, (uint8_t)127, (uint8_t)14, (uint8_t)55, (uint8_t)199, (uint8_t)58, (uint8_t)135, (uint8_t)5, (uint8_t)44, (uint8_t)47, (uint8_t)40, (uint8_t)20, (uint8_t)143, (uint8_t)9, (uint8_t)27, (uint8_t)41, (uint8_t)228, (uint8_t)59, (uint8_t)195, (uint8_t)215, (uint8_t)199, (uint8_t)214, (uint8_t)231, (uint8_t)107, (uint8_t)227, (uint8_t)173, (uint8_t)185, (uint8_t)120, (uint8_t)33, (uint8_t)107, (uint8_t)215, (uint8_t)98, (uint8_t)237, (uint8_t)104, (uint8_t)132, (uint8_t)81, (uint8_t)113, (uint8_t)36, (uint8_t)234, (uint8_t)174, (uint8_t)5, (uint8_t)64, (uint8_t)158, (uint8_t)88} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)10035);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)63929);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)49655);
    assert(p269_uri_LEN(ph) == 101);
    {
        char16_t * exemplary = u"iwruYwrdxjxgjilspdchxidbnaVggcdybmymFrPiwjsliwMNvqcQmcacvskpimkpipmwjqirtiqzcfsljmgnmtdobvxdgzKwxxlqt";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 202);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)2961);
    assert(p269_framerate_GET(pack) == (float)1.0979723E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)1986013198L);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)54538);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)54549);
    assert(p270_framerate_GET(pack) == (float)3.1184466E37F);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)38961);
    assert(p270_uri_LEN(ph) == 168);
    {
        char16_t * exemplary = u"mdzwnkevulkesqUwbzzrkJylageyenkgilmqlkbmWyerwFaeayeaqMtziejbpNgbhxhmwcwuxfuBmohrsiojjvjzefywczwZxwzowirVblsezlrtfepHblkprsuajjwegaLahwtgndfqpxeitdumbnBsybbPuhvgfmekhmqw";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 336);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p270_bitrate_GET(pack) == (uint32_t)393694024L);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)125);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 14);
    {
        char16_t * exemplary = u"qybxddouWfhzdi";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 61);
    {
        char16_t * exemplary = u"phmmxpujvpbKhouaflqjwlxsjsiImuavZsxpxkvjlsxQirEdxEhrflkvhsBkj";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 122);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)22489);
    {
        uint8_t exemplary[] =  {(uint8_t)104, (uint8_t)108, (uint8_t)62, (uint8_t)244, (uint8_t)180, (uint8_t)192, (uint8_t)185, (uint8_t)198} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)175, (uint8_t)235, (uint8_t)74, (uint8_t)68, (uint8_t)90, (uint8_t)32, (uint8_t)207, (uint8_t)215} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)13571);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)47312);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_time_usec_GET(pack) == (uint64_t)2157251526487921455L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)40763);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)2380015174L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p311_name_LEN(ph) == 72);
    {
        char16_t * exemplary = u"uergcnzjkqvffxiEwwqdhyiupbjagvsrqrsvrgwdzpqknqXTogeoDvqpphjlyqkqTgqPtdhh";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)844082885L);
    {
        uint8_t exemplary[] =  {(uint8_t)144, (uint8_t)159, (uint8_t)67, (uint8_t)153, (uint8_t)47, (uint8_t)78, (uint8_t)40, (uint8_t)204, (uint8_t)109, (uint8_t)47, (uint8_t)125, (uint8_t)230, (uint8_t)20, (uint8_t)20, (uint8_t)149, (uint8_t)216} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1305355254L);
    assert(p311_time_usec_GET(pack) == (uint64_t)4857865205420038110L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -8842);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p320_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"kwCaqqnffwfeoiFc";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)50);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)10654);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)45024);
    assert(p322_param_value_LEN(ph) == 33);
    {
        char16_t * exemplary = u"jvfxsygkxaohqmGrdxbhYjxreeajtoyun";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 66);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p322_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"Ztlngijtmylq";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 100);
    {
        char16_t * exemplary = u"zzjAwcnilzcjyzsdwLPgxshrltrmvAhjmnhcjyyeangwbznphzdlxviqrdyfbqFnobezsSNSDfaqlaenlstfdNGLdnsuktyffahq";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 200);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"grezlmeszkmnmi";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)110);
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_value_LEN(ph) == 83);
    {
        char16_t * exemplary = u"adntptzbjnbrnlVvlzwzAfzubevamqpqkviPxhvbosCicfghzNowxadqcoOSbkjpaaQcswjymzXXthEwzUz";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 166);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM);
    assert(p324_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"Oupjvslhcflte";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_IN_PROGRESS);
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    {
        uint16_t exemplary[] =  {(uint16_t)59295, (uint16_t)62395, (uint16_t)17591, (uint16_t)23234, (uint16_t)61648, (uint16_t)30936, (uint16_t)30154, (uint16_t)11086, (uint16_t)43930, (uint16_t)15156, (uint16_t)12306, (uint16_t)23896, (uint16_t)19772, (uint16_t)34804, (uint16_t)58249, (uint16_t)48037, (uint16_t)14730, (uint16_t)817, (uint16_t)36370, (uint16_t)4240, (uint16_t)366, (uint16_t)15960, (uint16_t)32124, (uint16_t)27882, (uint16_t)65249, (uint16_t)16514, (uint16_t)61893, (uint16_t)31158, (uint16_t)40034, (uint16_t)17464, (uint16_t)6395, (uint16_t)49209, (uint16_t)40212, (uint16_t)36767, (uint16_t)47158, (uint16_t)27870, (uint16_t)8482, (uint16_t)29651, (uint16_t)6502, (uint16_t)19108, (uint16_t)27331, (uint16_t)56813, (uint16_t)29770, (uint16_t)62859, (uint16_t)7754, (uint16_t)58721, (uint16_t)37193, (uint16_t)6772, (uint16_t)39793, (uint16_t)49631, (uint16_t)64536, (uint16_t)19869, (uint16_t)33570, (uint16_t)29953, (uint16_t)6635, (uint16_t)9051, (uint16_t)60912, (uint16_t)24246, (uint16_t)53898, (uint16_t)34722, (uint16_t)10384, (uint16_t)33768, (uint16_t)9276, (uint16_t)39570, (uint16_t)50555, (uint16_t)28696, (uint16_t)25685, (uint16_t)8396, (uint16_t)50970, (uint16_t)13050, (uint16_t)40167, (uint16_t)35461} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)1364087422723474166L);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)12842);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)50346);
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
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_OCTOROTOR, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2368201087L, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_EMERGENCY, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB, PH.base.pack) ;
        p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                          e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED), PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_errors_count2_SET((uint16_t)(uint16_t)30144, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)40420, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)39337, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                               e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)62292, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)60444, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)14, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)26520, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -2043, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)16154, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)46869, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)8727966769136156977L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)2413121891L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_rate_SET((float) -1.9144962E38F, PH.base.pack) ;
        p3_vy_SET((float)6.87864E37F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)49702, PH.base.pack) ;
        p3_y_SET((float)2.5037756E38F, PH.base.pack) ;
        p3_x_SET((float) -2.9073579E38F, PH.base.pack) ;
        p3_vz_SET((float)2.2426547E37F, PH.base.pack) ;
        p3_yaw_SET((float)2.6648942E38F, PH.base.pack) ;
        p3_afx_SET((float) -3.1166658E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2385793786L, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p3_afy_SET((float)7.7701556E37F, PH.base.pack) ;
        p3_vx_SET((float)1.9233574E38F, PH.base.pack) ;
        p3_afz_SET((float) -3.2760838E38F, PH.base.pack) ;
        p3_z_SET((float)7.034167E37F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p4_seq_SET((uint32_t)4283179602L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)3443853506419426524L, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_version_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        {
            char16_t* passkey = u"pwuquStmbPyjdDd";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_gcs_system_id_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"cnama";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)1945958422L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"aiMhqxveBIq";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)4041, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_index_SET((uint16_t)(uint16_t)8779, PH.base.pack) ;
        p22_param_value_SET((float) -2.8889709E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"xkvzxgfbmjrvm";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_count_SET((uint16_t)(uint16_t)47943, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p23_param_value_SET((float)1.6132299E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"cjcWtrjgbued";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_v_acc_SET((uint32_t)1631048640L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)3622, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)63410, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)1178315162397519084L, PH.base.pack) ;
        p24_lat_SET((int32_t) -1592012462, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)42358, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1778023212L, &PH) ;
        p24_hdg_acc_SET((uint32_t)2981447407L, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t)126780592, &PH) ;
        p24_alt_SET((int32_t)464270403, PH.base.pack) ;
        p24_lon_SET((int32_t) -28787096, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)63893, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)4055985723L, &PH) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_snr[] =  {(uint8_t)138, (uint8_t)162, (uint8_t)186, (uint8_t)44, (uint8_t)208, (uint8_t)192, (uint8_t)103, (uint8_t)114, (uint8_t)184, (uint8_t)127, (uint8_t)100, (uint8_t)188, (uint8_t)97, (uint8_t)3, (uint8_t)55, (uint8_t)104, (uint8_t)82, (uint8_t)201, (uint8_t)221, (uint8_t)122};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)39, (uint8_t)113, (uint8_t)138, (uint8_t)244, (uint8_t)24, (uint8_t)158, (uint8_t)248, (uint8_t)130, (uint8_t)221, (uint8_t)140, (uint8_t)166, (uint8_t)214, (uint8_t)160, (uint8_t)102, (uint8_t)174, (uint8_t)56, (uint8_t)213, (uint8_t)155, (uint8_t)230, (uint8_t)55};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)232, (uint8_t)205, (uint8_t)118, (uint8_t)101, (uint8_t)29, (uint8_t)96, (uint8_t)64, (uint8_t)57, (uint8_t)212, (uint8_t)114, (uint8_t)79, (uint8_t)186, (uint8_t)28, (uint8_t)99, (uint8_t)51, (uint8_t)3, (uint8_t)168, (uint8_t)92, (uint8_t)68, (uint8_t)129};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)24, (uint8_t)5, (uint8_t)219, (uint8_t)133, (uint8_t)4, (uint8_t)41, (uint8_t)177, (uint8_t)253, (uint8_t)204, (uint8_t)40, (uint8_t)77, (uint8_t)90, (uint8_t)142, (uint8_t)226, (uint8_t)201, (uint8_t)192, (uint8_t)11, (uint8_t)71, (uint8_t)54, (uint8_t)211};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)232, (uint8_t)172, (uint8_t)234, (uint8_t)234, (uint8_t)201, (uint8_t)103, (uint8_t)118, (uint8_t)171, (uint8_t)210, (uint8_t)193, (uint8_t)147, (uint8_t)2, (uint8_t)73, (uint8_t)98, (uint8_t)3, (uint8_t)202, (uint8_t)65, (uint8_t)244, (uint8_t)221, (uint8_t)229};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_xmag_SET((int16_t)(int16_t)14481, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)23345, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)16967, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)11123, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)3773, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -24308, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)6403, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -22715, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3909130814L, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)27947, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_ymag_SET((int16_t)(int16_t) -19465, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)7762, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -532, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)3396, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)6014094652102244114L, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -112, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)13889, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -9596, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)28330, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)8690, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -7784, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)5631, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)29422, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -31493, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)133784914142584080L, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)15490, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3725462088L, PH.base.pack) ;
        p29_press_diff_SET((float) -1.2286197E38F, PH.base.pack) ;
        p29_press_abs_SET((float)4.0341173E37F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float) -2.76753E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)2.4625742E38F, PH.base.pack) ;
        p30_roll_SET((float) -1.142686E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -2.0630689E37F, PH.base.pack) ;
        p30_pitch_SET((float)2.1987456E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2254064296L, PH.base.pack) ;
        p30_yawspeed_SET((float) -6.7971907E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float)1.962446E38F, PH.base.pack) ;
        p31_q1_SET((float) -1.8553529E38F, PH.base.pack) ;
        p31_q3_SET((float) -7.386954E35F, PH.base.pack) ;
        p31_q2_SET((float)1.9868803E38F, PH.base.pack) ;
        p31_yawspeed_SET((float)2.4085487E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1288546868L, PH.base.pack) ;
        p31_q4_SET((float) -1.0127154E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -2.7907788E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vx_SET((float)1.6574914E38F, PH.base.pack) ;
        p32_y_SET((float) -3.1594891E38F, PH.base.pack) ;
        p32_x_SET((float)5.7077936E37F, PH.base.pack) ;
        p32_vz_SET((float) -1.7410668E38F, PH.base.pack) ;
        p32_z_SET((float) -1.6259809E38F, PH.base.pack) ;
        p32_vy_SET((float) -1.0854351E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)378343899L, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lon_SET((int32_t) -522023683, PH.base.pack) ;
        p33_lat_SET((int32_t)1689833997, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)1117244251, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)16613, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)5963, PH.base.pack) ;
        p33_alt_SET((int32_t) -287046797, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1622366296L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)53299, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -14456, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan6_scaled_SET((int16_t)(int16_t)8942, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -26461, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -3011, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)2681418799L, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)27930, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -15509, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t)27713, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -13868, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -32223, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan1_raw_SET((uint16_t)(uint16_t)25210, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)43864, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)7710, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1446804616L, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)180, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)41622, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)56938, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)46618, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)5092, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo15_raw_SET((uint16_t)(uint16_t)37135, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)27060, &PH) ;
        p36_time_usec_SET((uint32_t)3574338594L, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)25166, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)4869, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)22619, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)24215, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)61617, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)51248, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)48168, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)35397, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)59187, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)43014, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)16343, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)36671, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)63355, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)41854, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)25638, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -477, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -27060, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -14457, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p39_param3_SET((float)2.1932655E38F, PH.base.pack) ;
        p39_z_SET((float)2.4942133E38F, PH.base.pack) ;
        p39_param1_SET((float) -3.8313776E36F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_UAVCAN, PH.base.pack) ;
        p39_param2_SET((float)9.020051E37F, PH.base.pack) ;
        p39_x_SET((float) -1.0234654E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)15645, PH.base.pack) ;
        p39_y_SET((float) -6.2332327E37F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p39_param4_SET((float)1.6215733E38F, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)55954, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_seq_SET((uint16_t)(uint16_t)24546, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)50014, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)32330, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)33263, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t) -1246774004, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1793087864, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1689196836295157568L, &PH) ;
        p48_latitude_SET((int32_t) -1242466161, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)7834989959824505815L, &PH) ;
        p49_longitude_SET((int32_t)2013859645, PH.base.pack) ;
        p49_latitude_SET((int32_t)563828859, PH.base.pack) ;
        p49_altitude_SET((int32_t) -954100936, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_min_SET((float) -2.2529703E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -17932, PH.base.pack) ;
        p50_param_value0_SET((float) -3.1648095E36F, PH.base.pack) ;
        {
            char16_t* param_id = u"m";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_scale_SET((float)2.1902903E38F, PH.base.pack) ;
        p50_param_value_max_SET((float) -3.1850624E37F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)45564, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p1x_SET((float)5.0113113E37F, PH.base.pack) ;
        p54_p2y_SET((float)1.0301685E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p54_p1y_SET((float) -8.905445E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p54_p2z_SET((float)4.277856E37F, PH.base.pack) ;
        p54_p2x_SET((float) -4.1324634E37F, PH.base.pack) ;
        p54_p1z_SET((float) -2.846768E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2z_SET((float) -1.6477947E38F, PH.base.pack) ;
        p55_p2x_SET((float)2.2583043E38F, PH.base.pack) ;
        p55_p1y_SET((float) -3.3592358E38F, PH.base.pack) ;
        p55_p1z_SET((float) -5.2611797E36F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p55_p2y_SET((float) -1.4150567E38F, PH.base.pack) ;
        p55_p1x_SET((float)1.1730949E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_rollspeed_SET((float)2.4001287E38F, PH.base.pack) ;
        {
            float q[] =  {3.1548286E38F, -1.5985517E38F, -8.848566E37F, -7.1824576E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float) -5.529453E37F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)4813734787884381236L, PH.base.pack) ;
        p61_yawspeed_SET((float)7.56543E37F, PH.base.pack) ;
        {
            float covariance[] =  {1.2124208E38F, -5.5689716E37F, 2.3494486E38F, 2.159633E38F, -3.0631629E38F, 1.1434328E38F, 2.3380826E38F, 1.366611E38F, -1.0063334E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_bearing_SET((int16_t)(int16_t)19373, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)34306, PH.base.pack) ;
        p62_alt_error_SET((float)2.5492253E38F, PH.base.pack) ;
        p62_aspd_error_SET((float) -1.9997275E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)27309, PH.base.pack) ;
        p62_xtrack_error_SET((float) -1.3610143E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)2.6189706E38F, PH.base.pack) ;
        p62_nav_roll_SET((float)1.9578851E38F, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_relative_alt_SET((int32_t)125563192, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)3389730451407549780L, PH.base.pack) ;
        {
            float covariance[] =  {-2.6957067E38F, -1.890076E38F, -2.761005E38F, -1.3191308E38F, 3.3401092E38F, -2.317562E38F, -1.7073996E38F, 5.1679326E37F, 3.1205631E38F, -1.7870657E38F, 2.9954099E38F, 2.590948E37F, -1.9539873E38F, -2.0769374E38F, 4.513414E36F, -2.2523906E38F, 2.8590878E38F, -1.8298435E38F, -2.7709826E38F, 1.0812835E38F, 6.3182845E37F, -2.6945915E38F, 2.5032556E38F, 1.157149E38F, 2.3735444E36F, -4.2012829E37F, -8.587707E37F, -1.3866891E38F, -1.0450886E38F, -2.6464273E38F, -2.5753717E38F, -2.0896E38F, 2.50004E38F, -9.945353E37F, -1.6900007E38F, 8.0223953E37F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vz_SET((float)9.630344E37F, PH.base.pack) ;
        p63_vy_SET((float)2.9610762E38F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p63_alt_SET((int32_t)2005295807, PH.base.pack) ;
        p63_vx_SET((float)1.6859252E38F, PH.base.pack) ;
        p63_lat_SET((int32_t) -1508592133, PH.base.pack) ;
        p63_lon_SET((int32_t)1977843962, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vy_SET((float)1.0815608E38F, PH.base.pack) ;
        p64_ay_SET((float) -3.0294322E38F, PH.base.pack) ;
        p64_z_SET((float)3.3627864E38F, PH.base.pack) ;
        p64_y_SET((float)1.9297705E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)5434551636133513944L, PH.base.pack) ;
        p64_vx_SET((float) -2.9892446E38F, PH.base.pack) ;
        p64_az_SET((float) -3.0957524E38F, PH.base.pack) ;
        p64_vz_SET((float) -2.2206486E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.2083756E38F, -1.6258282E37F, 3.3505596E38F, 2.1511086E38F, -8.672114E37F, 1.894085E38F, 5.202019E36F, 3.212736E37F, -7.840233E37F, -1.9384538E38F, 8.817739E37F, -2.977143E38F, -2.9575566E38F, -8.156426E37F, 2.0958434E38F, -4.958662E36F, -2.6546934E38F, 1.9080778E38F, -5.9246E37F, 1.0891799E38F, -3.230018E38F, -1.1402053E38F, -3.1039315E38F, -1.5808921E38F, 7.351391E37F, 2.4879114E38F, 1.4305344E38F, 3.538508E37F, 1.1414882E38F, -9.200113E37F, -5.1252366E37F, -3.2844245E38F, -1.1563615E38F, -2.6421362E38F, -3.3796829E38F, -7.8338657E37F, -3.1642478E38F, -1.8173437E38F, 2.5526378E38F, 2.3198255E36F, -2.4246953E38F, 2.581964E38F, 2.8594918E38F, 1.3099504E38F, -1.7613364E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_x_SET((float) -8.637642E37F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_ax_SET((float)2.7051848E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan7_raw_SET((uint16_t)(uint16_t)52223, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)2485, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)13705, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)8572, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)25461, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)50727, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)5188, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)33421, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)8703, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)47702, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)41847, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)28947, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)63460, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3045469240L, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)7093, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)10209, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)57083, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)13077, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)49558, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)48386, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)22005, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t)15469, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)29212, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -5949, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t)27683, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)51929, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan1_raw_SET((uint16_t)(uint16_t)11485, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)18615, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)32576, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)23308, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)7286, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)11618, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)8809, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)50387, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)49576, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p73_param4_SET((float) -1.7960691E37F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p73_param2_SET((float)2.482791E38F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p73_param1_SET((float)2.818316E38F, PH.base.pack) ;
        p73_x_SET((int32_t)890077737, PH.base.pack) ;
        p73_z_SET((float) -1.2453007E37F, PH.base.pack) ;
        p73_param3_SET((float) -2.481578E38F, PH.base.pack) ;
        p73_y_SET((int32_t) -738347609, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float)2.7293075E38F, PH.base.pack) ;
        p74_climb_SET((float)2.1317072E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)6800, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)24211, PH.base.pack) ;
        p74_airspeed_SET((float) -2.8551098E38F, PH.base.pack) ;
        p74_alt_SET((float) -1.968893E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param3_SET((float)1.1674076E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p75_param4_SET((float)1.4005405E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p75_param2_SET((float)3.0598193E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p75_y_SET((int32_t)990872196, PH.base.pack) ;
        p75_z_SET((float) -1.24216E38F, PH.base.pack) ;
        p75_param1_SET((float)1.3220899E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
        p75_x_SET((int32_t) -614488700, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_confirmation_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p76_param5_SET((float)1.4031938E38F, PH.base.pack) ;
        p76_param2_SET((float)3.0489252E38F, PH.base.pack) ;
        p76_param1_SET((float)1.6075319E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_LAND_START, PH.base.pack) ;
        p76_param3_SET((float)1.6890158E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p76_param4_SET((float) -8.881973E36F, PH.base.pack) ;
        p76_param7_SET((float) -4.5132707E37F, PH.base.pack) ;
        p76_param6_SET((float) -1.7294277E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_param2_SET((int32_t)7278856, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)3, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)104, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)130, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_TURN_LIGHT, PH.base.pack) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float)2.7030396E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2786737918L, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p81_thrust_SET((float)1.033371E38F, PH.base.pack) ;
        p81_yaw_SET((float)1.5703344E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p81_roll_SET((float)1.3283419E38F, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_type_mask_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        {
            float q[] =  {5.321458E36F, -1.9728319E38F, 2.5983188E38F, -3.0468047E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float) -2.4044492E38F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -2.0655358E38F, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.6367781E38F, PH.base.pack) ;
        p82_thrust_SET((float) -1.5606732E37F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)3332174594L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        c_TEST_Channel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_yaw_rate_SET((float)1.3225716E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)669870197L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)3.1069863E38F, PH.base.pack) ;
        p83_thrust_SET((float)2.0427037E38F, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -3.0019252E38F, PH.base.pack) ;
        {
            float q[] =  {-1.6033493E38F, -1.3538407E38F, -1.9652353E38F, 2.641194E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_type_mask_SET((uint16_t)(uint16_t)18744, PH.base.pack) ;
        p84_yaw_SET((float) -4.487791E37F, PH.base.pack) ;
        p84_vy_SET((float)1.1873839E38F, PH.base.pack) ;
        p84_y_SET((float)2.8384015E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2673692174L, PH.base.pack) ;
        p84_yaw_rate_SET((float) -3.2551943E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p84_afy_SET((float) -1.6420238E38F, PH.base.pack) ;
        p84_vx_SET((float) -4.427907E37F, PH.base.pack) ;
        p84_z_SET((float) -9.224202E37F, PH.base.pack) ;
        p84_afz_SET((float) -3.1600019E38F, PH.base.pack) ;
        p84_vz_SET((float)7.5297163E37F, PH.base.pack) ;
        p84_x_SET((float) -1.4941344E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p84_afx_SET((float) -2.723192E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_vz_SET((float)1.0802071E38F, PH.base.pack) ;
        p86_afy_SET((float)2.0151509E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)223782783, PH.base.pack) ;
        p86_yaw_rate_SET((float) -2.4027719E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2628653426L, PH.base.pack) ;
        p86_vy_SET((float) -5.393573E37F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)36479, PH.base.pack) ;
        p86_afx_SET((float)5.2352925E37F, PH.base.pack) ;
        p86_alt_SET((float) -2.0087066E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p86_vx_SET((float)9.294659E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -1546633235, PH.base.pack) ;
        p86_yaw_SET((float)2.5803196E38F, PH.base.pack) ;
        p86_afz_SET((float)2.0019235E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_afx_SET((float) -2.0472472E38F, PH.base.pack) ;
        p87_lat_int_SET((int32_t)1901309378, PH.base.pack) ;
        p87_lon_int_SET((int32_t)2106576766, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.1575445E38F, PH.base.pack) ;
        p87_vy_SET((float)1.0452664E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)2961, PH.base.pack) ;
        p87_alt_SET((float) -1.1625311E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)815859900L, PH.base.pack) ;
        p87_afy_SET((float) -2.6209252E38F, PH.base.pack) ;
        p87_afz_SET((float) -2.8309978E38F, PH.base.pack) ;
        p87_vx_SET((float)2.0713715E38F, PH.base.pack) ;
        p87_vz_SET((float)4.9452307E37F, PH.base.pack) ;
        p87_yaw_SET((float)3.2926938E38F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_time_boot_ms_SET((uint32_t)2200897410L, PH.base.pack) ;
        p89_roll_SET((float)2.2900504E38F, PH.base.pack) ;
        p89_y_SET((float)1.4586736E38F, PH.base.pack) ;
        p89_yaw_SET((float) -7.909022E37F, PH.base.pack) ;
        p89_pitch_SET((float) -1.7877364E38F, PH.base.pack) ;
        p89_z_SET((float) -3.9658807E37F, PH.base.pack) ;
        p89_x_SET((float) -3.0720539E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_alt_SET((int32_t) -409128906, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -9533, PH.base.pack) ;
        p90_lat_SET((int32_t) -1305774060, PH.base.pack) ;
        p90_lon_SET((int32_t) -1841375888, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)5110, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)3458, PH.base.pack) ;
        p90_pitch_SET((float)3.0228524E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -32406, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -14485, PH.base.pack) ;
        p90_rollspeed_SET((float) -3.0479551E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)7293, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2832205653053135463L, PH.base.pack) ;
        p90_roll_SET((float) -3.356502E38F, PH.base.pack) ;
        p90_pitchspeed_SET((float) -1.4303324E38F, PH.base.pack) ;
        p90_yaw_SET((float)1.1386386E38F, PH.base.pack) ;
        p90_yawspeed_SET((float)1.1262859E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux2_SET((float)8.982039E36F, PH.base.pack) ;
        p91_throttle_SET((float)3.8237058E37F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)6.0828336E37F, PH.base.pack) ;
        p91_aux4_SET((float) -1.6101972E38F, PH.base.pack) ;
        p91_aux3_SET((float) -1.45688E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p91_roll_ailerons_SET((float)3.756973E37F, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -1.7107367E38F, PH.base.pack) ;
        p91_aux1_SET((float) -1.734185E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)1073602316216600177L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan2_raw_SET((uint16_t)(uint16_t)18117, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)10402, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)1681518652508885641L, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)56971, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)62979, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)17690, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)63244, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)15950, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)12787, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)47765, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)22824, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)25424, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)16574, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)5260745353416405871L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)2239029103081186530L, PH.base.pack) ;
        {
            float controls[] =  {-1.2803587E38F, -1.963452E38F, 1.3767976E38F, -5.522936E37F, 1.876611E38F, 2.8575644E38F, 1.1395382E38F, 2.5705275E38F, 2.0165666E38F, -1.3129052E37F, 2.0861424E38F, -1.1335906E38F, -2.882793E38F, -3.3205546E38F, 1.3784452E38F, 1.4077943E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_rate_x_SET((float)1.837952E38F, &PH) ;
        p100_ground_distance_SET((float) -1.0412376E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -16359, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -2.5601968E37F, &PH) ;
        p100_flow_comp_m_y_SET((float) -2.5025668E38F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)2.8694677E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -1642, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)5354725655768243690L, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_usec_SET((uint64_t)7472208955689673906L, PH.base.pack) ;
        p101_yaw_SET((float) -2.1922816E38F, PH.base.pack) ;
        p101_pitch_SET((float) -7.566639E37F, PH.base.pack) ;
        p101_y_SET((float)2.86853E38F, PH.base.pack) ;
        p101_z_SET((float)2.0210529E38F, PH.base.pack) ;
        p101_x_SET((float) -1.8054657E38F, PH.base.pack) ;
        p101_roll_SET((float) -3.2904705E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_x_SET((float) -2.650691E38F, PH.base.pack) ;
        p102_z_SET((float)1.3934371E38F, PH.base.pack) ;
        p102_yaw_SET((float)1.705023E38F, PH.base.pack) ;
        p102_pitch_SET((float) -3.1935512E38F, PH.base.pack) ;
        p102_roll_SET((float)2.4752083E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)8843999295538589401L, PH.base.pack) ;
        p102_y_SET((float)1.3605125E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float) -1.4767835E37F, PH.base.pack) ;
        p103_y_SET((float)9.812353E37F, PH.base.pack) ;
        p103_x_SET((float) -3.3698098E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)3309575221279712701L, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_x_SET((float) -3.277412E38F, PH.base.pack) ;
        p104_y_SET((float)5.775396E37F, PH.base.pack) ;
        p104_pitch_SET((float)2.0749785E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)6662620311104895561L, PH.base.pack) ;
        p104_z_SET((float) -1.4682016E38F, PH.base.pack) ;
        p104_roll_SET((float) -3.0842235E38F, PH.base.pack) ;
        p104_yaw_SET((float)2.2341906E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_temperature_SET((float) -3.7984326E36F, PH.base.pack) ;
        p105_xmag_SET((float)1.6272458E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)5278025634902290332L, PH.base.pack) ;
        p105_yacc_SET((float)2.6202707E38F, PH.base.pack) ;
        p105_ymag_SET((float)2.9938204E38F, PH.base.pack) ;
        p105_zgyro_SET((float)4.945496E37F, PH.base.pack) ;
        p105_xgyro_SET((float)2.8072486E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)3.438E36F, PH.base.pack) ;
        p105_xacc_SET((float) -7.4203114E37F, PH.base.pack) ;
        p105_zacc_SET((float) -2.3568973E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -1.4539798E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.5032194E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)56854, PH.base.pack) ;
        p105_ygyro_SET((float) -2.5484195E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.1027461E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_temperature_SET((int16_t)(int16_t) -17494, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)409152074L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)3.5265817E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6316001911933209737L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)1.2370526E38F, PH.base.pack) ;
        p106_integrated_x_SET((float)3.0263609E38F, PH.base.pack) ;
        p106_integrated_y_SET((float)1.8098801E38F, PH.base.pack) ;
        p106_distance_SET((float) -2.655103E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)202260266L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -1.2067296E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)2735626694L, PH.base.pack) ;
        p107_pressure_alt_SET((float)1.9876873E37F, PH.base.pack) ;
        p107_xmag_SET((float) -1.368391E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)3.355604E38F, PH.base.pack) ;
        p107_ygyro_SET((float) -1.7842622E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -1.3857324E36F, PH.base.pack) ;
        p107_zmag_SET((float)2.3162276E38F, PH.base.pack) ;
        p107_yacc_SET((float)3.797947E37F, PH.base.pack) ;
        p107_temperature_SET((float)7.4884695E37F, PH.base.pack) ;
        p107_zacc_SET((float) -1.8398498E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.3068815E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)1551947457327364129L, PH.base.pack) ;
        p107_diff_pressure_SET((float) -3.0976827E38F, PH.base.pack) ;
        p107_xgyro_SET((float)5.328029E37F, PH.base.pack) ;
        p107_xacc_SET((float) -3.4134568E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_q2_SET((float)2.2749716E38F, PH.base.pack) ;
        p108_xacc_SET((float) -1.0788892E38F, PH.base.pack) ;
        p108_zacc_SET((float)2.3526502E37F, PH.base.pack) ;
        p108_yaw_SET((float) -1.5481686E37F, PH.base.pack) ;
        p108_xgyro_SET((float) -1.6345789E38F, PH.base.pack) ;
        p108_q3_SET((float)2.3235058E38F, PH.base.pack) ;
        p108_vn_SET((float)1.8232296E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -2.0618325E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -2.474224E38F, PH.base.pack) ;
        p108_lon_SET((float)1.4680727E38F, PH.base.pack) ;
        p108_ve_SET((float) -1.0258425E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)1.9597769E38F, PH.base.pack) ;
        p108_vd_SET((float) -3.3758887E37F, PH.base.pack) ;
        p108_lat_SET((float) -5.67679E37F, PH.base.pack) ;
        p108_pitch_SET((float)2.4637252E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -1.4739588E38F, PH.base.pack) ;
        p108_alt_SET((float) -1.4629266E38F, PH.base.pack) ;
        p108_q1_SET((float) -1.893283E38F, PH.base.pack) ;
        p108_yacc_SET((float) -2.4727515E38F, PH.base.pack) ;
        p108_roll_SET((float)8.314679E37F, PH.base.pack) ;
        p108_q4_SET((float) -2.5287402E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_fixed__SET((uint16_t)(uint16_t)7794, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)27785, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_component_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)61, (uint8_t)118, (uint8_t)230, (uint8_t)125, (uint8_t)64, (uint8_t)195, (uint8_t)177, (uint8_t)199, (uint8_t)39, (uint8_t)219, (uint8_t)30, (uint8_t)6, (uint8_t)111, (uint8_t)181, (uint8_t)148, (uint8_t)133, (uint8_t)105, (uint8_t)32, (uint8_t)153, (uint8_t)221, (uint8_t)133, (uint8_t)67, (uint8_t)244, (uint8_t)88, (uint8_t)227, (uint8_t)120, (uint8_t)251, (uint8_t)183, (uint8_t)94, (uint8_t)38, (uint8_t)7, (uint8_t)87, (uint8_t)83, (uint8_t)178, (uint8_t)66, (uint8_t)98, (uint8_t)135, (uint8_t)134, (uint8_t)127, (uint8_t)182, (uint8_t)149, (uint8_t)97, (uint8_t)90, (uint8_t)126, (uint8_t)245, (uint8_t)225, (uint8_t)140, (uint8_t)253, (uint8_t)79, (uint8_t)77, (uint8_t)83, (uint8_t)159, (uint8_t)251, (uint8_t)59, (uint8_t)208, (uint8_t)55, (uint8_t)23, (uint8_t)57, (uint8_t)129, (uint8_t)239, (uint8_t)160, (uint8_t)228, (uint8_t)156, (uint8_t)186, (uint8_t)94, (uint8_t)165, (uint8_t)152, (uint8_t)170, (uint8_t)29, (uint8_t)72, (uint8_t)9, (uint8_t)59, (uint8_t)197, (uint8_t)56, (uint8_t)18, (uint8_t)224, (uint8_t)145, (uint8_t)189, (uint8_t)106, (uint8_t)48, (uint8_t)241, (uint8_t)212, (uint8_t)99, (uint8_t)78, (uint8_t)11, (uint8_t)14, (uint8_t)137, (uint8_t)0, (uint8_t)106, (uint8_t)88, (uint8_t)131, (uint8_t)237, (uint8_t)27, (uint8_t)241, (uint8_t)139, (uint8_t)56, (uint8_t)207, (uint8_t)240, (uint8_t)33, (uint8_t)197, (uint8_t)129, (uint8_t)200, (uint8_t)38, (uint8_t)196, (uint8_t)66, (uint8_t)202, (uint8_t)244, (uint8_t)192, (uint8_t)127, (uint8_t)140, (uint8_t)237, (uint8_t)222, (uint8_t)167, (uint8_t)67, (uint8_t)206, (uint8_t)77, (uint8_t)18, (uint8_t)153, (uint8_t)221, (uint8_t)98, (uint8_t)6, (uint8_t)147, (uint8_t)161, (uint8_t)63, (uint8_t)244, (uint8_t)91, (uint8_t)45, (uint8_t)127, (uint8_t)50, (uint8_t)249, (uint8_t)215, (uint8_t)141, (uint8_t)144, (uint8_t)35, (uint8_t)132, (uint8_t)214, (uint8_t)133, (uint8_t)114, (uint8_t)157, (uint8_t)143, (uint8_t)54, (uint8_t)223, (uint8_t)253, (uint8_t)186, (uint8_t)99, (uint8_t)211, (uint8_t)7, (uint8_t)157, (uint8_t)58, (uint8_t)96, (uint8_t)180, (uint8_t)169, (uint8_t)15, (uint8_t)213, (uint8_t)82, (uint8_t)220, (uint8_t)166, (uint8_t)175, (uint8_t)40, (uint8_t)232, (uint8_t)220, (uint8_t)222, (uint8_t)254, (uint8_t)91, (uint8_t)15, (uint8_t)181, (uint8_t)122, (uint8_t)134, (uint8_t)146, (uint8_t)40, (uint8_t)212, (uint8_t)25, (uint8_t)42, (uint8_t)18, (uint8_t)15, (uint8_t)68, (uint8_t)46, (uint8_t)0, (uint8_t)232, (uint8_t)79, (uint8_t)53, (uint8_t)24, (uint8_t)99, (uint8_t)64, (uint8_t)168, (uint8_t)201, (uint8_t)115, (uint8_t)127, (uint8_t)24, (uint8_t)63, (uint8_t)123, (uint8_t)192, (uint8_t)76, (uint8_t)91, (uint8_t)205, (uint8_t)243, (uint8_t)95, (uint8_t)211, (uint8_t)239, (uint8_t)156, (uint8_t)170, (uint8_t)100, (uint8_t)27, (uint8_t)70, (uint8_t)34, (uint8_t)83, (uint8_t)172, (uint8_t)10, (uint8_t)158, (uint8_t)23, (uint8_t)103, (uint8_t)161, (uint8_t)18, (uint8_t)180, (uint8_t)205, (uint8_t)128, (uint8_t)185, (uint8_t)162, (uint8_t)147, (uint8_t)25, (uint8_t)154, (uint8_t)20, (uint8_t)67, (uint8_t)102, (uint8_t)102, (uint8_t)75, (uint8_t)55, (uint8_t)200, (uint8_t)71, (uint8_t)200, (uint8_t)97, (uint8_t)187, (uint8_t)34, (uint8_t)15, (uint8_t)103, (uint8_t)194, (uint8_t)2, (uint8_t)207, (uint8_t)194, (uint8_t)123, (uint8_t)88, (uint8_t)31, (uint8_t)202, (uint8_t)109, (uint8_t)243, (uint8_t)200, (uint8_t)104, (uint8_t)221, (uint8_t)107, (uint8_t)128, (uint8_t)238};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t) -8439932114570341042L, PH.base.pack) ;
        p111_tc1_SET((int64_t)7517988402439648145L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)1092450327L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)510017626207449017L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_vel_SET((uint16_t)(uint16_t)36129, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -28158, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)24770, PH.base.pack) ;
        p113_lon_SET((int32_t)366181130, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)19506, PH.base.pack) ;
        p113_alt_SET((int32_t) -1679955975, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)1869, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)7204, PH.base.pack) ;
        p113_lat_SET((int32_t)1261803483, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)204, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)8356598505193163680L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_temperature_SET((int16_t)(int16_t) -54, PH.base.pack) ;
        p114_integrated_y_SET((float)1.7384074E38F, PH.base.pack) ;
        p114_integrated_x_SET((float)2.087782E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -2.8414072E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -2.5507503E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -1.423513E38F, PH.base.pack) ;
        p114_distance_SET((float)2.090094E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)8639009172074228845L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)1182259997L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)2395984618L, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_lat_SET((int32_t)618862939, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)24338, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)11715, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)26518, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.609413E38F, 3.6031221E37F, 1.2476989E38F, 3.3152702E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_vz_SET((int16_t)(int16_t) -4297, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)1130406468568807247L, PH.base.pack) ;
        p115_lon_SET((int32_t)640215503, PH.base.pack) ;
        p115_alt_SET((int32_t)241589092, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)6504, PH.base.pack) ;
        p115_rollspeed_SET((float)3.2759178E38F, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.6645121E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)22840, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)9181, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -4358, PH.base.pack) ;
        p115_yawspeed_SET((float)1.7750216E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_yacc_SET((int16_t)(int16_t)16219, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -11885, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -10276, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)26927, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -4137, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2556121615L, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t) -409, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -12568, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)21724, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)3825, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)58105, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)1638, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_num_logs_SET((uint16_t)(uint16_t)57323, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)858625074L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)16759, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)64433, PH.base.pack) ;
        p118_size_SET((uint32_t)2502954282L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)31397, PH.base.pack) ;
        p119_count_SET((uint32_t)3959045842L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1458794439L, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)6288, PH.base.pack) ;
        p120_ofs_SET((uint32_t)489941814L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)30, (uint8_t)54, (uint8_t)228, (uint8_t)65, (uint8_t)193, (uint8_t)51, (uint8_t)36, (uint8_t)155, (uint8_t)109, (uint8_t)44, (uint8_t)16, (uint8_t)20, (uint8_t)248, (uint8_t)37, (uint8_t)88, (uint8_t)46, (uint8_t)98, (uint8_t)89, (uint8_t)170, (uint8_t)171, (uint8_t)65, (uint8_t)78, (uint8_t)191, (uint8_t)138, (uint8_t)2, (uint8_t)210, (uint8_t)194, (uint8_t)125, (uint8_t)19, (uint8_t)61, (uint8_t)40, (uint8_t)205, (uint8_t)187, (uint8_t)35, (uint8_t)202, (uint8_t)63, (uint8_t)25, (uint8_t)37, (uint8_t)86, (uint8_t)112, (uint8_t)250, (uint8_t)236, (uint8_t)136, (uint8_t)163, (uint8_t)38, (uint8_t)12, (uint8_t)230, (uint8_t)241, (uint8_t)173, (uint8_t)23, (uint8_t)152, (uint8_t)248, (uint8_t)62, (uint8_t)100, (uint8_t)61, (uint8_t)128, (uint8_t)66, (uint8_t)94, (uint8_t)151, (uint8_t)7, (uint8_t)224, (uint8_t)226, (uint8_t)22, (uint8_t)63, (uint8_t)24, (uint8_t)42, (uint8_t)30, (uint8_t)90, (uint8_t)162, (uint8_t)211, (uint8_t)10, (uint8_t)114, (uint8_t)214, (uint8_t)26, (uint8_t)201, (uint8_t)236, (uint8_t)59, (uint8_t)0, (uint8_t)76, (uint8_t)14, (uint8_t)136, (uint8_t)20, (uint8_t)19, (uint8_t)163, (uint8_t)70, (uint8_t)159, (uint8_t)129, (uint8_t)201, (uint8_t)39, (uint8_t)80};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)173, (uint8_t)124, (uint8_t)211, (uint8_t)251, (uint8_t)158, (uint8_t)125, (uint8_t)97, (uint8_t)29, (uint8_t)25, (uint8_t)54, (uint8_t)176, (uint8_t)50, (uint8_t)132, (uint8_t)118, (uint8_t)214, (uint8_t)196, (uint8_t)39, (uint8_t)160, (uint8_t)27, (uint8_t)22, (uint8_t)213, (uint8_t)232, (uint8_t)6, (uint8_t)179, (uint8_t)159, (uint8_t)82, (uint8_t)1, (uint8_t)43, (uint8_t)172, (uint8_t)34, (uint8_t)187, (uint8_t)62, (uint8_t)26, (uint8_t)28, (uint8_t)6, (uint8_t)220, (uint8_t)243, (uint8_t)251, (uint8_t)246, (uint8_t)57, (uint8_t)10, (uint8_t)166, (uint8_t)142, (uint8_t)95, (uint8_t)111, (uint8_t)17, (uint8_t)40, (uint8_t)35, (uint8_t)161, (uint8_t)35, (uint8_t)32, (uint8_t)239, (uint8_t)115, (uint8_t)204, (uint8_t)165, (uint8_t)101, (uint8_t)212, (uint8_t)254, (uint8_t)139, (uint8_t)106, (uint8_t)63, (uint8_t)233, (uint8_t)224, (uint8_t)32, (uint8_t)179, (uint8_t)194, (uint8_t)83, (uint8_t)144, (uint8_t)4, (uint8_t)138, (uint8_t)33, (uint8_t)76, (uint8_t)126, (uint8_t)17, (uint8_t)172, (uint8_t)171, (uint8_t)77, (uint8_t)124, (uint8_t)68, (uint8_t)54, (uint8_t)28, (uint8_t)234, (uint8_t)101, (uint8_t)165, (uint8_t)237, (uint8_t)227, (uint8_t)159, (uint8_t)38, (uint8_t)192, (uint8_t)79, (uint8_t)223, (uint8_t)78, (uint8_t)21, (uint8_t)244, (uint8_t)140, (uint8_t)113, (uint8_t)104, (uint8_t)26, (uint8_t)230, (uint8_t)153, (uint8_t)28, (uint8_t)243, (uint8_t)115, (uint8_t)28, (uint8_t)69, (uint8_t)170, (uint8_t)86, (uint8_t)25, (uint8_t)236, (uint8_t)211};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_eph_SET((uint16_t)(uint16_t)12174, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)45846, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1640087953L, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)59294, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)760136646223207252L, PH.base.pack) ;
        p124_lat_SET((int32_t) -989848541, PH.base.pack) ;
        p124_alt_SET((int32_t)1963830972, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)1279, PH.base.pack) ;
        p124_lon_SET((int32_t) -739647616, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)20488, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)14702, PH.base.pack) ;
        p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                        e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED), PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)182, (uint8_t)132, (uint8_t)140, (uint8_t)181, (uint8_t)3, (uint8_t)132, (uint8_t)154, (uint8_t)183, (uint8_t)114, (uint8_t)35, (uint8_t)152, (uint8_t)159, (uint8_t)251, (uint8_t)146, (uint8_t)198, (uint8_t)112, (uint8_t)162, (uint8_t)173, (uint8_t)204, (uint8_t)87, (uint8_t)23, (uint8_t)79, (uint8_t)10, (uint8_t)191, (uint8_t)40, (uint8_t)167, (uint8_t)16, (uint8_t)10, (uint8_t)138, (uint8_t)31, (uint8_t)34, (uint8_t)84, (uint8_t)58, (uint8_t)92, (uint8_t)135, (uint8_t)108, (uint8_t)103, (uint8_t)172, (uint8_t)172, (uint8_t)199, (uint8_t)36, (uint8_t)7, (uint8_t)0, (uint8_t)147, (uint8_t)47, (uint8_t)161, (uint8_t)131, (uint8_t)195, (uint8_t)209, (uint8_t)111, (uint8_t)119, (uint8_t)183, (uint8_t)209, (uint8_t)136, (uint8_t)247, (uint8_t)14, (uint8_t)232, (uint8_t)91, (uint8_t)133, (uint8_t)236, (uint8_t)149, (uint8_t)91, (uint8_t)122, (uint8_t)36, (uint8_t)82, (uint8_t)54, (uint8_t)97, (uint8_t)13, (uint8_t)69, (uint8_t)40};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                        e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY), PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)6970, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)3097330505L, PH.base.pack) ;
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_time_last_baseline_ms_SET((uint32_t)821895812L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)39221, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2832729612L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p127_tow_SET((uint32_t)2743584219L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)354223196, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)2093463328, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)201516214, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -577622754, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_health_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)668987585L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)534180396L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -971032171, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)1213886110, PH.base.pack) ;
        p128_tow_SET((uint32_t)3749454599L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)45273, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)2114971928, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)1525939098, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ygyro_SET((int16_t)(int16_t)11860, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)2368, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)966557289L, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -5856, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -16730, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)30970, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -10678, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)26334, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)19732, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -14898, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_packets_SET((uint16_t)(uint16_t)6724, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)45590, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)42846, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p130_size_SET((uint32_t)501245363L, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)68, (uint8_t)178, (uint8_t)9, (uint8_t)149, (uint8_t)227, (uint8_t)168, (uint8_t)95, (uint8_t)60, (uint8_t)10, (uint8_t)241, (uint8_t)81, (uint8_t)185, (uint8_t)142, (uint8_t)229, (uint8_t)109, (uint8_t)96, (uint8_t)247, (uint8_t)203, (uint8_t)162, (uint8_t)253, (uint8_t)221, (uint8_t)234, (uint8_t)66, (uint8_t)179, (uint8_t)248, (uint8_t)141, (uint8_t)41, (uint8_t)67, (uint8_t)217, (uint8_t)172, (uint8_t)186, (uint8_t)52, (uint8_t)50, (uint8_t)180, (uint8_t)177, (uint8_t)246, (uint8_t)184, (uint8_t)203, (uint8_t)227, (uint8_t)229, (uint8_t)156, (uint8_t)236, (uint8_t)180, (uint8_t)134, (uint8_t)141, (uint8_t)190, (uint8_t)83, (uint8_t)186, (uint8_t)168, (uint8_t)57, (uint8_t)164, (uint8_t)220, (uint8_t)88, (uint8_t)173, (uint8_t)43, (uint8_t)148, (uint8_t)21, (uint8_t)75, (uint8_t)187, (uint8_t)199, (uint8_t)81, (uint8_t)48, (uint8_t)216, (uint8_t)50, (uint8_t)31, (uint8_t)246, (uint8_t)224, (uint8_t)45, (uint8_t)242, (uint8_t)147, (uint8_t)4, (uint8_t)16, (uint8_t)129, (uint8_t)37, (uint8_t)27, (uint8_t)45, (uint8_t)161, (uint8_t)228, (uint8_t)9, (uint8_t)63, (uint8_t)164, (uint8_t)155, (uint8_t)218, (uint8_t)7, (uint8_t)157, (uint8_t)227, (uint8_t)109, (uint8_t)147, (uint8_t)219, (uint8_t)6, (uint8_t)185, (uint8_t)185, (uint8_t)95, (uint8_t)211, (uint8_t)15, (uint8_t)129, (uint8_t)178, (uint8_t)178, (uint8_t)85, (uint8_t)139, (uint8_t)58, (uint8_t)84, (uint8_t)77, (uint8_t)31, (uint8_t)254, (uint8_t)227, (uint8_t)155, (uint8_t)169, (uint8_t)188, (uint8_t)114, (uint8_t)241, (uint8_t)41, (uint8_t)241, (uint8_t)172, (uint8_t)64, (uint8_t)71, (uint8_t)155, (uint8_t)87, (uint8_t)209, (uint8_t)58, (uint8_t)69, (uint8_t)9, (uint8_t)84, (uint8_t)31, (uint8_t)0, (uint8_t)218, (uint8_t)73, (uint8_t)183, (uint8_t)151, (uint8_t)184, (uint8_t)35, (uint8_t)174, (uint8_t)172, (uint8_t)147, (uint8_t)33, (uint8_t)184, (uint8_t)228, (uint8_t)70, (uint8_t)37, (uint8_t)43, (uint8_t)197, (uint8_t)209, (uint8_t)55, (uint8_t)72, (uint8_t)187, (uint8_t)93, (uint8_t)46, (uint8_t)47, (uint8_t)157, (uint8_t)146, (uint8_t)108, (uint8_t)108, (uint8_t)157, (uint8_t)216, (uint8_t)209, (uint8_t)253, (uint8_t)94, (uint8_t)118, (uint8_t)4, (uint8_t)143, (uint8_t)148, (uint8_t)74, (uint8_t)249, (uint8_t)91, (uint8_t)173, (uint8_t)116, (uint8_t)58, (uint8_t)196, (uint8_t)14, (uint8_t)171, (uint8_t)66, (uint8_t)159, (uint8_t)222, (uint8_t)40, (uint8_t)142, (uint8_t)96, (uint8_t)253, (uint8_t)194, (uint8_t)218, (uint8_t)8, (uint8_t)54, (uint8_t)115, (uint8_t)111, (uint8_t)240, (uint8_t)190, (uint8_t)128, (uint8_t)100, (uint8_t)193, (uint8_t)25, (uint8_t)28, (uint8_t)195, (uint8_t)109, (uint8_t)140, (uint8_t)115, (uint8_t)237, (uint8_t)142, (uint8_t)144, (uint8_t)55, (uint8_t)216, (uint8_t)129, (uint8_t)237, (uint8_t)114, (uint8_t)223, (uint8_t)167, (uint8_t)228, (uint8_t)4, (uint8_t)55, (uint8_t)245, (uint8_t)74, (uint8_t)199, (uint8_t)121, (uint8_t)169, (uint8_t)18, (uint8_t)233, (uint8_t)219, (uint8_t)132, (uint8_t)155, (uint8_t)82, (uint8_t)161, (uint8_t)5, (uint8_t)163, (uint8_t)202, (uint8_t)197, (uint8_t)175, (uint8_t)79, (uint8_t)69, (uint8_t)251, (uint8_t)198, (uint8_t)144, (uint8_t)130, (uint8_t)219, (uint8_t)124, (uint8_t)19, (uint8_t)92, (uint8_t)197, (uint8_t)129, (uint8_t)221, (uint8_t)19, (uint8_t)161, (uint8_t)50, (uint8_t)210, (uint8_t)87, (uint8_t)134, (uint8_t)89, (uint8_t)126, (uint8_t)161, (uint8_t)79, (uint8_t)79, (uint8_t)168, (uint8_t)102, (uint8_t)39, (uint8_t)28, (uint8_t)41};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)50304, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_id_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_180, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)64445, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2454020105L, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)26051, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)29768, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_grid_spacing_SET((uint16_t)(uint16_t)37700, PH.base.pack) ;
        p133_lon_SET((int32_t)884446308, PH.base.pack) ;
        p133_lat_SET((int32_t) -1468581729, PH.base.pack) ;
        p133_mask_SET((uint64_t)9121066560038311676L, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t) -755014592, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -28575, (int16_t)11511, (int16_t)3118, (int16_t) -7877, (int16_t) -14561, (int16_t)29769, (int16_t)13778, (int16_t)16503, (int16_t) -11041, (int16_t)4307, (int16_t) -15659, (int16_t)11932, (int16_t)19535, (int16_t) -32581, (int16_t)31572, (int16_t) -17178};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)52153, PH.base.pack) ;
        p134_lon_SET((int32_t) -819922380, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -730051938, PH.base.pack) ;
        p135_lat_SET((int32_t)2041419854, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float)1.1898775E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)49411, PH.base.pack) ;
        p136_terrain_height_SET((float) -5.334361E37F, PH.base.pack) ;
        p136_lon_SET((int32_t) -1395858177, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)3363, PH.base.pack) ;
        p136_lat_SET((int32_t) -670223865, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)8060, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float)2.701547E38F, PH.base.pack) ;
        p137_press_abs_SET((float)2.9691862E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)2329, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)4177462740L, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)6510877867391602605L, PH.base.pack) ;
        p138_z_SET((float)1.0896863E38F, PH.base.pack) ;
        p138_y_SET((float)2.990864E38F, PH.base.pack) ;
        p138_x_SET((float) -1.7049639E38F, PH.base.pack) ;
        {
            float q[] =  {-3.20737E37F, 1.5144625E38F, -2.6561913E38F, -2.6428126E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        {
            float controls[] =  {2.6567122E37F, -1.738392E38F, -1.2494255E38F, -2.685112E38F, 1.5473886E37F, 1.4867653E38F, -3.0646017E38F, 7.3236365E36F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)1597536677507813078L, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)1221740477354467599L, PH.base.pack) ;
        {
            float controls[] =  {1.7506742E38F, 2.8661289E37F, 1.8561244E38F, 2.4901434E38F, -3.3080714E37F, 2.1715289E38F, 2.0233821E38F, 2.4651658E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_amsl_SET((float)2.6466644E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -3.0527416E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.6517595E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -7.6910274E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -2.9129976E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)2.7745892E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)8601352168081623731L, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t uri[] =  {(uint8_t)130, (uint8_t)108, (uint8_t)180, (uint8_t)204, (uint8_t)14, (uint8_t)213, (uint8_t)63, (uint8_t)70, (uint8_t)75, (uint8_t)168, (uint8_t)169, (uint8_t)1, (uint8_t)46, (uint8_t)215, (uint8_t)131, (uint8_t)70, (uint8_t)63, (uint8_t)144, (uint8_t)76, (uint8_t)78, (uint8_t)178, (uint8_t)149, (uint8_t)201, (uint8_t)225, (uint8_t)87, (uint8_t)240, (uint8_t)99, (uint8_t)39, (uint8_t)188, (uint8_t)223, (uint8_t)108, (uint8_t)140, (uint8_t)145, (uint8_t)95, (uint8_t)131, (uint8_t)106, (uint8_t)108, (uint8_t)16, (uint8_t)115, (uint8_t)164, (uint8_t)88, (uint8_t)68, (uint8_t)16, (uint8_t)112, (uint8_t)208, (uint8_t)216, (uint8_t)179, (uint8_t)211, (uint8_t)140, (uint8_t)223, (uint8_t)112, (uint8_t)230, (uint8_t)228, (uint8_t)192, (uint8_t)93, (uint8_t)107, (uint8_t)192, (uint8_t)156, (uint8_t)4, (uint8_t)79, (uint8_t)43, (uint8_t)252, (uint8_t)191, (uint8_t)68, (uint8_t)136, (uint8_t)121, (uint8_t)243, (uint8_t)246, (uint8_t)81, (uint8_t)200, (uint8_t)109, (uint8_t)95, (uint8_t)71, (uint8_t)199, (uint8_t)19, (uint8_t)120, (uint8_t)54, (uint8_t)100, (uint8_t)201, (uint8_t)0, (uint8_t)121, (uint8_t)86, (uint8_t)247, (uint8_t)193, (uint8_t)69, (uint8_t)92, (uint8_t)80, (uint8_t)11, (uint8_t)108, (uint8_t)153, (uint8_t)213, (uint8_t)24, (uint8_t)18, (uint8_t)148, (uint8_t)220, (uint8_t)55, (uint8_t)112, (uint8_t)215, (uint8_t)90, (uint8_t)246, (uint8_t)237, (uint8_t)93, (uint8_t)114, (uint8_t)228, (uint8_t)139, (uint8_t)47, (uint8_t)142, (uint8_t)80, (uint8_t)200, (uint8_t)115, (uint8_t)157, (uint8_t)33, (uint8_t)251, (uint8_t)112, (uint8_t)178, (uint8_t)112, (uint8_t)45, (uint8_t)85, (uint8_t)111, (uint8_t)120};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p142_transfer_type_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)182, (uint8_t)206, (uint8_t)107, (uint8_t)56, (uint8_t)72, (uint8_t)100, (uint8_t)216, (uint8_t)201, (uint8_t)208, (uint8_t)8, (uint8_t)62, (uint8_t)181, (uint8_t)209, (uint8_t)242, (uint8_t)120, (uint8_t)28, (uint8_t)78, (uint8_t)38, (uint8_t)72, (uint8_t)135, (uint8_t)240, (uint8_t)33, (uint8_t)41, (uint8_t)60, (uint8_t)99, (uint8_t)208, (uint8_t)133, (uint8_t)193, (uint8_t)90, (uint8_t)211, (uint8_t)161, (uint8_t)43, (uint8_t)207, (uint8_t)56, (uint8_t)227, (uint8_t)163, (uint8_t)52, (uint8_t)48, (uint8_t)89, (uint8_t)44, (uint8_t)232, (uint8_t)33, (uint8_t)203, (uint8_t)165, (uint8_t)62, (uint8_t)158, (uint8_t)70, (uint8_t)208, (uint8_t)15, (uint8_t)134, (uint8_t)1, (uint8_t)164, (uint8_t)121, (uint8_t)186, (uint8_t)240, (uint8_t)12, (uint8_t)8, (uint8_t)102, (uint8_t)247, (uint8_t)45, (uint8_t)130, (uint8_t)219, (uint8_t)51, (uint8_t)237, (uint8_t)33, (uint8_t)253, (uint8_t)27, (uint8_t)41, (uint8_t)249, (uint8_t)107, (uint8_t)161, (uint8_t)219, (uint8_t)201, (uint8_t)88, (uint8_t)11, (uint8_t)188, (uint8_t)166, (uint8_t)187, (uint8_t)208, (uint8_t)0, (uint8_t)117, (uint8_t)123, (uint8_t)99, (uint8_t)245, (uint8_t)174, (uint8_t)245, (uint8_t)79, (uint8_t)37, (uint8_t)29, (uint8_t)61, (uint8_t)204, (uint8_t)9, (uint8_t)226, (uint8_t)131, (uint8_t)74, (uint8_t)202, (uint8_t)169, (uint8_t)51, (uint8_t)3, (uint8_t)140, (uint8_t)3, (uint8_t)91, (uint8_t)236, (uint8_t)192, (uint8_t)19, (uint8_t)21, (uint8_t)224, (uint8_t)205, (uint8_t)59, (uint8_t)144, (uint8_t)112, (uint8_t)157, (uint8_t)78, (uint8_t)169, (uint8_t)45, (uint8_t)156, (uint8_t)201, (uint8_t)208, (uint8_t)247, (uint8_t)7};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_abs_SET((float) -1.1983486E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)4175788980L, PH.base.pack) ;
        p143_press_diff_SET((float)2.0969235E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t)27338, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_lat_SET((int32_t) -592812632, PH.base.pack) ;
        {
            float vel[] =  {2.4460973E38F, 5.3681114E37F, -2.4519705E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)3.166199E38F, PH.base.pack) ;
        p144_lon_SET((int32_t)754066331, PH.base.pack) ;
        {
            float acc[] =  {-2.523387E38F, -2.181224E37F, 8.302649E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)627703799685721512L, PH.base.pack) ;
        {
            float position_cov[] =  {1.9995608E38F, 9.143456E37F, 1.4209028E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-2.856969E38F, 4.317438E37F, -2.5425582E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)2355961586016763788L, PH.base.pack) ;
        {
            float attitude_q[] =  {2.7470009E38F, -2.6165886E38F, 3.2525385E38F, -3.2264085E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_pos_SET((float)3.8577992E37F, PH.base.pack) ;
        p146_z_vel_SET((float)2.8887506E38F, PH.base.pack) ;
        p146_z_acc_SET((float)1.3378068E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -1.9777915E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)276873188634234135L, PH.base.pack) ;
        p146_z_pos_SET((float)2.3418026E38F, PH.base.pack) ;
        {
            float q[] =  {6.22984E37F, -1.5724663E38F, 1.9821358E38F, -9.892832E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float vel_variance[] =  {2.2258663E38F, -2.4024944E38F, 2.6694326E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        {
            float pos_variance[] =  {-7.403642E37F, -3.3075905E38F, -6.895184E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_pitch_rate_SET((float) -9.945516E37F, PH.base.pack) ;
        p146_x_acc_SET((float)2.7588225E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -1.3636583E38F, PH.base.pack) ;
        p146_roll_rate_SET((float)2.1447538E37F, PH.base.pack) ;
        p146_y_vel_SET((float)2.8727403E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)1.4470273E38F, PH.base.pack) ;
        p146_airspeed_SET((float)3.313076E38F, PH.base.pack) ;
        p146_y_acc_SET((float)1.1610373E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_TYPE_PAYLOAD, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -93, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)751416974, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)31084, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)13384, (uint16_t)45569, (uint16_t)15615, (uint16_t)55444, (uint16_t)8628, (uint16_t)9982, (uint16_t)45475, (uint16_t)42987, (uint16_t)18725, (uint16_t)37811};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_energy_consumed_SET((int32_t)55248091, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -16344, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t uid2[] =  {(uint8_t)170, (uint8_t)82, (uint8_t)230, (uint8_t)56, (uint8_t)233, (uint8_t)230, (uint8_t)118, (uint8_t)75, (uint8_t)173, (uint8_t)158, (uint8_t)58, (uint8_t)104, (uint8_t)201, (uint8_t)193, (uint8_t)26, (uint8_t)230, (uint8_t)137, (uint8_t)27};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION), PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)195, (uint8_t)108, (uint8_t)233, (uint8_t)20, (uint8_t)100, (uint8_t)3, (uint8_t)124, (uint8_t)182};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_middleware_sw_version_SET((uint32_t)1025672981L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)2649506992L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)2340035441L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)149, (uint8_t)23, (uint8_t)163, (uint8_t)61, (uint8_t)117, (uint8_t)128, (uint8_t)104, (uint8_t)116};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)41553, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)6, (uint8_t)168, (uint8_t)249, (uint8_t)21, (uint8_t)78, (uint8_t)211, (uint8_t)5, (uint8_t)43};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)6771966564808546969L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)1228316375L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)17162, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
        p149_x_SET((float) -7.170414E37F, &PH) ;
        p149_size_x_SET((float) -1.5521891E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        {
            float q[] =  {3.236762E38F, -2.2086911E38F, 9.720465E37F, 3.2245752E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_distance_SET((float) -9.04913E37F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_z_SET((float)2.8451686E38F, &PH) ;
        p149_angle_y_SET((float) -1.6752369E38F, PH.base.pack) ;
        p149_size_y_SET((float)3.2916353E38F, PH.base.pack) ;
        p149_y_SET((float)2.416411E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)78, &PH) ;
        p149_angle_x_SET((float) -2.366068E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)7171653044739120128L, PH.base.pack) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CPU_LOAD_170(), &PH);
        p170_batVolt_SET((uint16_t)(uint16_t)17941, PH.base.pack) ;
        p170_ctrlLoad_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p170_sensLoad_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        c_CommunicationChannel_on_CPU_LOAD_170(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_BIAS_172(), &PH);
        p172_azBias_SET((float)2.3712946E38F, PH.base.pack) ;
        p172_gyBias_SET((float) -1.1456557E38F, PH.base.pack) ;
        p172_gzBias_SET((float)7.341513E37F, PH.base.pack) ;
        p172_gxBias_SET((float)1.0470745E38F, PH.base.pack) ;
        p172_axBias_SET((float)1.3166431E38F, PH.base.pack) ;
        p172_ayBias_SET((float) -3.2870851E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_BIAS_172(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DIAGNOSTIC_173(), &PH);
        p173_diagFl1_SET((float) -2.8129893E38F, PH.base.pack) ;
        p173_diagSh3_SET((int16_t)(int16_t)8482, PH.base.pack) ;
        p173_diagSh1_SET((int16_t)(int16_t)22251, PH.base.pack) ;
        p173_diagSh2_SET((int16_t)(int16_t) -14224, PH.base.pack) ;
        p173_diagFl2_SET((float)1.3951366E38F, PH.base.pack) ;
        p173_diagFl3_SET((float) -8.804417E37F, PH.base.pack) ;
        c_CommunicationChannel_on_DIAGNOSTIC_173(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_NAVIGATION_176(), &PH);
        p176_ay_body_SET((float)3.3508044E38F, PH.base.pack) ;
        p176_totalDist_SET((float) -2.83225E38F, PH.base.pack) ;
        p176_fromWP_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p176_toWP_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p176_phi_c_SET((float)3.2726863E38F, PH.base.pack) ;
        p176_psiDot_c_SET((float)7.2772124E37F, PH.base.pack) ;
        p176_u_m_SET((float) -1.95483E38F, PH.base.pack) ;
        p176_theta_c_SET((float) -2.163432E38F, PH.base.pack) ;
        p176_h_c_SET((uint16_t)(uint16_t)25870, PH.base.pack) ;
        p176_dist2Go_SET((float)2.1877445E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_NAVIGATION_176(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DATA_LOG_177(), &PH);
        p177_fl_3_SET((float)3.2931802E38F, PH.base.pack) ;
        p177_fl_6_SET((float) -1.9883373E38F, PH.base.pack) ;
        p177_fl_5_SET((float)1.4994007E38F, PH.base.pack) ;
        p177_fl_2_SET((float)1.2383974E38F, PH.base.pack) ;
        p177_fl_4_SET((float) -2.7703782E38F, PH.base.pack) ;
        p177_fl_1_SET((float)1.2769154E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_LOG_177(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_DATE_TIME_179(), &PH);
        p179_year_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p179_sigUsedMask_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p179_sec_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p179_month_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p179_day_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p179_visSat_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        p179_clockStat_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p179_useSat_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p179_hour_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p179_min_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p179_GppGl_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p179_percentUsed_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_DATE_TIME_179(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MID_LVL_CMDS_180(), &PH);
        p180_uCommand_SET((float) -1.1837362E38F, PH.base.pack) ;
        p180_hCommand_SET((float)2.2638E37F, PH.base.pack) ;
        p180_target_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p180_rCommand_SET((float)2.4772779E38F, PH.base.pack) ;
        c_CommunicationChannel_on_MID_LVL_CMDS_180(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CTRL_SRFC_PT_181(), &PH);
        p181_target_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p181_bitfieldPt_SET((uint16_t)(uint16_t)46960, PH.base.pack) ;
        c_CommunicationChannel_on_CTRL_SRFC_PT_181(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CAMERA_ORDER_184(), &PH);
        p184_zoom_SET((int8_t)(int8_t)14, PH.base.pack) ;
        p184_target_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        p184_moveHome_SET((int8_t)(int8_t) -104, PH.base.pack) ;
        p184_tilt_SET((int8_t)(int8_t) -47, PH.base.pack) ;
        p184_pan_SET((int8_t)(int8_t)31, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CONTROL_SURFACE_185(), &PH);
        p185_idSurface_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p185_target_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p185_mControl_SET((float) -1.4701675E38F, PH.base.pack) ;
        p185_bControl_SET((float) -2.4601088E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SURFACE_185(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_MOBILE_LOCATION_186(), &PH);
        p186_target_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p186_longitude_SET((float)1.5686384E37F, PH.base.pack) ;
        p186_latitude_SET((float) -1.6373453E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SLUGS_CONFIGURATION_CAMERA_188(), &PH);
        p188_idOrder_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p188_target_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p188_order_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ISR_LOCATION_189(), &PH);
        p189_latitude_SET((float) -1.5179171E38F, PH.base.pack) ;
        p189_option3_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p189_option2_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p189_height_SET((float)2.0739581E38F, PH.base.pack) ;
        p189_option1_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p189_target_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p189_longitude_SET((float) -6.074059E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ISR_LOCATION_189(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VOLT_SENSOR_191(), &PH);
        p191_voltage_SET((uint16_t)(uint16_t)11692, PH.base.pack) ;
        p191_reading2_SET((uint16_t)(uint16_t)5562, PH.base.pack) ;
        p191_r2Type_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        c_CommunicationChannel_on_VOLT_SENSOR_191(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PTZ_STATUS_192(), &PH);
        p192_tilt_SET((int16_t)(int16_t)17176, PH.base.pack) ;
        p192_pan_SET((int16_t)(int16_t)495, PH.base.pack) ;
        p192_zoom_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_CommunicationChannel_on_PTZ_STATUS_192(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAV_STATUS_193(), &PH);
        p193_course_SET((float) -4.364703E36F, PH.base.pack) ;
        p193_longitude_SET((float) -8.1235494E36F, PH.base.pack) ;
        p193_speed_SET((float)1.7351549E38F, PH.base.pack) ;
        p193_altitude_SET((float) -2.1225223E38F, PH.base.pack) ;
        p193_target_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p193_latitude_SET((float)1.3339058E38F, PH.base.pack) ;
        c_CommunicationChannel_on_UAV_STATUS_193(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUS_GPS_194(), &PH);
        p194_magVar_SET((float)2.4075656E38F, PH.base.pack) ;
        p194_gpsQuality_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p194_magDir_SET((int8_t)(int8_t)50, PH.base.pack) ;
        p194_msgsType_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p194_posStatus_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p194_modeInd_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p194_csFails_SET((uint16_t)(uint16_t)172, PH.base.pack) ;
        c_CommunicationChannel_on_STATUS_GPS_194(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NOVATEL_DIAG_195(), &PH);
        p195_solStatus_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p195_timeStatus_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p195_posType_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p195_receiverStatus_SET((uint32_t)496074806L, PH.base.pack) ;
        p195_csFails_SET((uint16_t)(uint16_t)29890, PH.base.pack) ;
        p195_velType_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p195_posSolAge_SET((float)4.6098895E37F, PH.base.pack) ;
        c_CommunicationChannel_on_NOVATEL_DIAG_195(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSOR_DIAG_196(), &PH);
        p196_char1_SET((int8_t)(int8_t) -44, PH.base.pack) ;
        p196_int1_SET((int16_t)(int16_t)5253, PH.base.pack) ;
        p196_float2_SET((float) -2.6506899E38F, PH.base.pack) ;
        p196_float1_SET((float)1.9607516E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SENSOR_DIAG_196(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BOOT_197(), &PH);
        p197_version_SET((uint32_t)257663186L, PH.base.pack) ;
        c_CommunicationChannel_on_BOOT_197(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_accuracy_SET((float) -1.0908332E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -4.8233984E37F, PH.base.pack) ;
        p230_vel_ratio_SET((float)2.0955638E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -2.4584455E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)5.8033283E37F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -6.3105477E37F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)1.6140117E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)8.529798E37F, PH.base.pack) ;
        p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                        e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS), PH.base.pack) ;
        p230_time_usec_SET((uint64_t)9208457423408500L, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
        p231_wind_z_SET((float)1.3281879E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -8.894556E37F, PH.base.pack) ;
        p231_var_horiz_SET((float) -2.2498153E38F, PH.base.pack) ;
        p231_var_vert_SET((float)3.8379952E37F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -1.5604916E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -3.166215E37F, PH.base.pack) ;
        p231_wind_y_SET((float) -1.088157E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -1.7053923E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)2839261002752190233L, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
        p232_time_week_SET((uint16_t)(uint16_t)4177, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p232_vdop_SET((float) -2.3996587E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)6205240408801939510L, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p232_alt_SET((float) -3.0147989E38F, PH.base.pack) ;
        p232_hdop_SET((float) -1.8329702E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2686258151L, PH.base.pack) ;
        p232_vd_SET((float) -1.306838E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)8.651104E36F, PH.base.pack) ;
        p232_lat_SET((int32_t)2029335380, PH.base.pack) ;
        p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP |
                               e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY), PH.base.pack) ;
        p232_vn_SET((float) -1.398853E38F, PH.base.pack) ;
        p232_ve_SET((float)2.7527102E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)3.0317978E38F, PH.base.pack) ;
        p232_lon_SET((int32_t)308225254, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p232_vert_accuracy_SET((float)8.520085E37F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)157, (uint8_t)87, (uint8_t)0, (uint8_t)7, (uint8_t)49, (uint8_t)108, (uint8_t)134, (uint8_t)174, (uint8_t)159, (uint8_t)156, (uint8_t)56, (uint8_t)105, (uint8_t)127, (uint8_t)106, (uint8_t)80, (uint8_t)170, (uint8_t)211, (uint8_t)109, (uint8_t)33, (uint8_t)200, (uint8_t)139, (uint8_t)49, (uint8_t)145, (uint8_t)176, (uint8_t)167, (uint8_t)229, (uint8_t)224, (uint8_t)221, (uint8_t)6, (uint8_t)37, (uint8_t)113, (uint8_t)141, (uint8_t)250, (uint8_t)33, (uint8_t)93, (uint8_t)144, (uint8_t)173, (uint8_t)141, (uint8_t)63, (uint8_t)59, (uint8_t)164, (uint8_t)178, (uint8_t)254, (uint8_t)71, (uint8_t)228, (uint8_t)246, (uint8_t)107, (uint8_t)140, (uint8_t)230, (uint8_t)204, (uint8_t)168, (uint8_t)226, (uint8_t)24, (uint8_t)44, (uint8_t)234, (uint8_t)238, (uint8_t)152, (uint8_t)85, (uint8_t)39, (uint8_t)208, (uint8_t)172, (uint8_t)112, (uint8_t)59, (uint8_t)94, (uint8_t)114, (uint8_t)156, (uint8_t)24, (uint8_t)145, (uint8_t)145, (uint8_t)38, (uint8_t)138, (uint8_t)152, (uint8_t)77, (uint8_t)129, (uint8_t)55, (uint8_t)252, (uint8_t)235, (uint8_t)43, (uint8_t)36, (uint8_t)94, (uint8_t)169, (uint8_t)23, (uint8_t)12, (uint8_t)22, (uint8_t)166, (uint8_t)214, (uint8_t)112, (uint8_t)20, (uint8_t)175, (uint8_t)203, (uint8_t)244, (uint8_t)56, (uint8_t)178, (uint8_t)40, (uint8_t)156, (uint8_t)213, (uint8_t)179, (uint8_t)222, (uint8_t)68, (uint8_t)22, (uint8_t)219, (uint8_t)65, (uint8_t)237, (uint8_t)127, (uint8_t)206, (uint8_t)101, (uint8_t)50, (uint8_t)254, (uint8_t)209, (uint8_t)179, (uint8_t)162, (uint8_t)154, (uint8_t)148, (uint8_t)145, (uint8_t)251, (uint8_t)41, (uint8_t)15, (uint8_t)54, (uint8_t)99, (uint8_t)246, (uint8_t)250, (uint8_t)97, (uint8_t)213, (uint8_t)67, (uint8_t)242, (uint8_t)26, (uint8_t)107, (uint8_t)152, (uint8_t)207, (uint8_t)19, (uint8_t)84, (uint8_t)132, (uint8_t)59, (uint8_t)245, (uint8_t)98, (uint8_t)188, (uint8_t)152, (uint8_t)252, (uint8_t)78, (uint8_t)150, (uint8_t)178, (uint8_t)143, (uint8_t)129, (uint8_t)129, (uint8_t)50, (uint8_t)69, (uint8_t)48, (uint8_t)9, (uint8_t)184, (uint8_t)138, (uint8_t)38, (uint8_t)44, (uint8_t)73, (uint8_t)90, (uint8_t)164, (uint8_t)155, (uint8_t)131, (uint8_t)233, (uint8_t)193, (uint8_t)254, (uint8_t)131, (uint8_t)96, (uint8_t)226, (uint8_t)217, (uint8_t)133, (uint8_t)117, (uint8_t)126, (uint8_t)155, (uint8_t)222, (uint8_t)16, (uint8_t)120, (uint8_t)100, (uint8_t)145, (uint8_t)18, (uint8_t)137, (uint8_t)176, (uint8_t)187, (uint8_t)187, (uint8_t)116, (uint8_t)217};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_climb_rate_SET((int8_t)(int8_t) -80, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -53, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)57105, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_longitude_SET((int32_t)2061642597, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)12319, PH.base.pack) ;
        p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                            e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -26780, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)114, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)34769, PH.base.pack) ;
        p234_latitude_SET((int32_t) -1845363456, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)87, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -6604, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1295893122L, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -29078, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)15328, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_vibration_x_SET((float)8.88594E37F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3898909041L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.9035678E38F, PH.base.pack) ;
        p241_vibration_z_SET((float)1.6957717E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)1628964820L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)7753734045770659766L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3396370242L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        {
            float q[] =  {-1.6978115E38F, 2.8245178E38F, -8.317952E37F, 3.503049E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_altitude_SET((int32_t) -895546505, PH.base.pack) ;
        p242_longitude_SET((int32_t)1021630051, PH.base.pack) ;
        p242_approach_z_SET((float)1.9080141E38F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)3655407604944890551L, &PH) ;
        p242_latitude_SET((int32_t) -726004260, PH.base.pack) ;
        p242_x_SET((float)1.2591977E38F, PH.base.pack) ;
        p242_y_SET((float)2.3926528E38F, PH.base.pack) ;
        p242_approach_y_SET((float)7.62881E36F, PH.base.pack) ;
        p242_z_SET((float) -8.632061E37F, PH.base.pack) ;
        p242_approach_x_SET((float)2.5482511E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        p243_longitude_SET((int32_t)1836327592, PH.base.pack) ;
        p243_x_SET((float) -1.9187488E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t)1824677614, PH.base.pack) ;
        p243_altitude_SET((int32_t)1094736259, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)3455242827807158572L, &PH) ;
        p243_approach_x_SET((float) -1.2765116E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -1.7105381E38F, PH.base.pack) ;
        p243_y_SET((float)1.5637175E37F, PH.base.pack) ;
        p243_approach_z_SET((float) -1.5212334E38F, PH.base.pack) ;
        {
            float q[] =  {1.9325936E38F, -1.4690068E38F, 3.0035275E38F, -2.1767672E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float)3.1667113E38F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -854985435, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)20543, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING |
                        e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN), PH.base.pack) ;
        p246_lon_SET((int32_t) -1118690624, PH.base.pack) ;
        p246_altitude_SET((int32_t)1496757153, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)46487, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)63451827L, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_ROTOCRAFT, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)29947, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)16201, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)25749, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        {
            char16_t* callsign = u"upjj";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_lat_SET((int32_t)375246231, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_threat_level_SET((e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW |
                               e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH), PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.3550403E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)2.816482E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_RTL, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.6636986E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)586448800L, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)11916, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)227, (uint8_t)125, (uint8_t)176, (uint8_t)21, (uint8_t)69, (uint8_t)233, (uint8_t)105, (uint8_t)156, (uint8_t)206, (uint8_t)179, (uint8_t)89, (uint8_t)57, (uint8_t)16, (uint8_t)226, (uint8_t)173, (uint8_t)90, (uint8_t)183, (uint8_t)112, (uint8_t)185, (uint8_t)9, (uint8_t)248, (uint8_t)171, (uint8_t)42, (uint8_t)117, (uint8_t)105, (uint8_t)170, (uint8_t)176, (uint8_t)60, (uint8_t)246, (uint8_t)32, (uint8_t)12, (uint8_t)2, (uint8_t)107, (uint8_t)92, (uint8_t)171, (uint8_t)213, (uint8_t)20, (uint8_t)34, (uint8_t)84, (uint8_t)208, (uint8_t)91, (uint8_t)157, (uint8_t)31, (uint8_t)194, (uint8_t)99, (uint8_t)144, (uint8_t)108, (uint8_t)185, (uint8_t)55, (uint8_t)182, (uint8_t)21, (uint8_t)252, (uint8_t)34, (uint8_t)222, (uint8_t)169, (uint8_t)254, (uint8_t)37, (uint8_t)208, (uint8_t)72, (uint8_t)204, (uint8_t)104, (uint8_t)49, (uint8_t)84, (uint8_t)159, (uint8_t)249, (uint8_t)110, (uint8_t)159, (uint8_t)160, (uint8_t)102, (uint8_t)117, (uint8_t)50, (uint8_t)212, (uint8_t)26, (uint8_t)191, (uint8_t)38, (uint8_t)15, (uint8_t)26, (uint8_t)171, (uint8_t)217, (uint8_t)70, (uint8_t)102, (uint8_t)135, (uint8_t)208, (uint8_t)214, (uint8_t)14, (uint8_t)187, (uint8_t)200, (uint8_t)43, (uint8_t)23, (uint8_t)32, (uint8_t)101, (uint8_t)9, (uint8_t)51, (uint8_t)136, (uint8_t)210, (uint8_t)107, (uint8_t)214, (uint8_t)255, (uint8_t)31, (uint8_t)107, (uint8_t)59, (uint8_t)16, (uint8_t)52, (uint8_t)144, (uint8_t)218, (uint8_t)95, (uint8_t)31, (uint8_t)1, (uint8_t)15, (uint8_t)231, (uint8_t)209, (uint8_t)211, (uint8_t)90, (uint8_t)236, (uint8_t)61, (uint8_t)18, (uint8_t)140, (uint8_t)200, (uint8_t)29, (uint8_t)223, (uint8_t)120, (uint8_t)230, (uint8_t)210, (uint8_t)49, (uint8_t)101, (uint8_t)144, (uint8_t)86, (uint8_t)61, (uint8_t)228, (uint8_t)212, (uint8_t)131, (uint8_t)4, (uint8_t)120, (uint8_t)183, (uint8_t)208, (uint8_t)82, (uint8_t)38, (uint8_t)233, (uint8_t)96, (uint8_t)250, (uint8_t)84, (uint8_t)157, (uint8_t)208, (uint8_t)61, (uint8_t)220, (uint8_t)255, (uint8_t)214, (uint8_t)107, (uint8_t)64, (uint8_t)0, (uint8_t)5, (uint8_t)243, (uint8_t)80, (uint8_t)32, (uint8_t)57, (uint8_t)161, (uint8_t)79, (uint8_t)28, (uint8_t)222, (uint8_t)129, (uint8_t)193, (uint8_t)57, (uint8_t)110, (uint8_t)223, (uint8_t)51, (uint8_t)5, (uint8_t)10, (uint8_t)146, (uint8_t)24, (uint8_t)75, (uint8_t)216, (uint8_t)140, (uint8_t)230, (uint8_t)85, (uint8_t)185, (uint8_t)100, (uint8_t)175, (uint8_t)104, (uint8_t)210, (uint8_t)195, (uint8_t)245, (uint8_t)130, (uint8_t)67, (uint8_t)154, (uint8_t)201, (uint8_t)130, (uint8_t)122, (uint8_t)62, (uint8_t)146, (uint8_t)110, (uint8_t)249, (uint8_t)53, (uint8_t)102, (uint8_t)176, (uint8_t)157, (uint8_t)79, (uint8_t)239, (uint8_t)152, (uint8_t)92, (uint8_t)26, (uint8_t)0, (uint8_t)93, (uint8_t)186, (uint8_t)188, (uint8_t)9, (uint8_t)11, (uint8_t)127, (uint8_t)32, (uint8_t)168, (uint8_t)17, (uint8_t)38, (uint8_t)223, (uint8_t)232, (uint8_t)45, (uint8_t)65, (uint8_t)2, (uint8_t)189, (uint8_t)100, (uint8_t)46, (uint8_t)240, (uint8_t)139, (uint8_t)165, (uint8_t)228, (uint8_t)37, (uint8_t)176, (uint8_t)169, (uint8_t)116, (uint8_t)140, (uint8_t)21, (uint8_t)253, (uint8_t)157, (uint8_t)123, (uint8_t)200, (uint8_t)120, (uint8_t)207, (uint8_t)14, (uint8_t)147, (uint8_t)15, (uint8_t)159, (uint8_t)227, (uint8_t)47, (uint8_t)141, (uint8_t)188, (uint8_t)168, (uint8_t)71, (uint8_t)159, (uint8_t)59, (uint8_t)25, (uint8_t)197};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t) -57, (int8_t)112, (int8_t)106, (int8_t)65, (int8_t) -36, (int8_t) -89, (int8_t) -26, (int8_t)8, (int8_t) -13, (int8_t)47, (int8_t)54, (int8_t) -94, (int8_t) -14, (int8_t) -62, (int8_t)60, (int8_t) -120, (int8_t) -18, (int8_t)50, (int8_t) -2, (int8_t) -74, (int8_t)35, (int8_t) -100, (int8_t) -24, (int8_t) -124, (int8_t)8, (int8_t) -86, (int8_t)25, (int8_t)32, (int8_t) -105, (int8_t)84, (int8_t)106, (int8_t)7};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)16627, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        p250_x_SET((float) -1.1351017E38F, PH.base.pack) ;
        p250_z_SET((float) -2.971191E38F, PH.base.pack) ;
        p250_y_SET((float)7.739887E36F, PH.base.pack) ;
        {
            char16_t* name = u"nptunwuW";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)3889429453381407108L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -2.4793783E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)2837358595L, PH.base.pack) ;
        {
            char16_t* name = u"ryxf";
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
            char16_t* name = u"a";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)3272189847L, PH.base.pack) ;
        p252_value_SET((int32_t) -1615440492, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, PH.base.pack) ;
        {
            char16_t* text = u"bkfxmhufvFffvxiKjTq";
            p253_text_SET_(text, &PH) ;
        }
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p254_value_SET((float)1.1443905E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1288113467L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        {
            uint8_t secret_key[] =  {(uint8_t)198, (uint8_t)116, (uint8_t)45, (uint8_t)247, (uint8_t)180, (uint8_t)55, (uint8_t)164, (uint8_t)240, (uint8_t)29, (uint8_t)53, (uint8_t)133, (uint8_t)40, (uint8_t)37, (uint8_t)90, (uint8_t)4, (uint8_t)135, (uint8_t)182, (uint8_t)9, (uint8_t)32, (uint8_t)116, (uint8_t)86, (uint8_t)110, (uint8_t)192, (uint8_t)76, (uint8_t)225, (uint8_t)138, (uint8_t)167, (uint8_t)174, (uint8_t)80, (uint8_t)194, (uint8_t)27, (uint8_t)123};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)3804130512939255754L, PH.base.pack) ;
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_last_change_ms_SET((uint32_t)547027225L, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)24836270L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        {
            char16_t* tune = u"xVtRxr";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_sensor_size_v_SET((float) -1.4299655E38F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)34083, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)139, (uint8_t)68, (uint8_t)125, (uint8_t)149, (uint8_t)150, (uint8_t)169, (uint8_t)146, (uint8_t)188, (uint8_t)160, (uint8_t)122, (uint8_t)118, (uint8_t)101, (uint8_t)241, (uint8_t)84, (uint8_t)105, (uint8_t)230, (uint8_t)97, (uint8_t)55, (uint8_t)213, (uint8_t)45, (uint8_t)105, (uint8_t)46, (uint8_t)34, (uint8_t)208, (uint8_t)166, (uint8_t)193, (uint8_t)2, (uint8_t)170, (uint8_t)20, (uint8_t)83, (uint8_t)190, (uint8_t)20};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)51716, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)56, (uint8_t)125, (uint8_t)28, (uint8_t)218, (uint8_t)62, (uint8_t)17, (uint8_t)212, (uint8_t)14, (uint8_t)119, (uint8_t)238, (uint8_t)198, (uint8_t)18, (uint8_t)138, (uint8_t)143, (uint8_t)189, (uint8_t)251, (uint8_t)123, (uint8_t)43, (uint8_t)190, (uint8_t)65, (uint8_t)115, (uint8_t)92, (uint8_t)254, (uint8_t)254, (uint8_t)75, (uint8_t)206, (uint8_t)163, (uint8_t)108, (uint8_t)129, (uint8_t)184, (uint8_t)147, (uint8_t)181};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_focal_length_SET((float)2.9310163E38F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)311101875L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)33931, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)786836195L, PH.base.pack) ;
        p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE), PH.base.pack) ;
        p259_sensor_size_h_SET((float) -3.0575876E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"nDoBdhaxjrizhmeyxUUkCfZzzgukrguoQutoskkjfxopkkbfdlqknbxhlwxluyzhxryygxnkWxtxiflgxvzdpnynhgGTaLeNdcgcletqhHwirdmkhkzcadgvjuhfrvvaEq";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)3254092763L, PH.base.pack) ;
        p260_mode_id_SET((e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY), PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float) -1.3462915E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2832701868L, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p261_used_capacity_SET((float)2.5781693E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -3.0473856E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p261_write_speed_SET((float) -1.7756785E38F, PH.base.pack) ;
        p261_read_speed_SET((float)2.9386455E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)1780001994L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p262_image_interval_SET((float)4.453051E36F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p262_available_capacity_SET((float)7.27429E37F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)1432752928L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_time_utc_SET((uint64_t)703317381551829819L, PH.base.pack) ;
        p263_alt_SET((int32_t)1104459243, PH.base.pack) ;
        {
            float q[] =  {-3.1696979E38F, 3.3108026E38F, -2.474192E38F, 1.1462996E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_time_boot_ms_SET((uint32_t)1120284865L, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1145425973, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)98, PH.base.pack) ;
        {
            char16_t* file_url = u"djzvulsxrSwfblsyieuiraoyrrxzEuzbmcypvTmvjhqoybxTvPjZggvvkqapdSmjeEtjwlhgrhlvztxiezfrwbklxjy";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p263_lat_SET((int32_t) -988653941, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -1452437474, PH.base.pack) ;
        p263_lon_SET((int32_t)1751530392, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_flight_uuid_SET((uint64_t)2094253198347421623L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)3489916229L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)5446581557599514829L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)1691260246187801663L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)452821304L, PH.base.pack) ;
        p265_roll_SET((float) -3.3782696E37F, PH.base.pack) ;
        p265_yaw_SET((float)2.726115E38F, PH.base.pack) ;
        p265_pitch_SET((float) -6.358186E37F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        p266_sequence_SET((uint16_t)(uint16_t)23762, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)197, (uint8_t)190, (uint8_t)221, (uint8_t)188, (uint8_t)7, (uint8_t)172, (uint8_t)231, (uint8_t)31, (uint8_t)208, (uint8_t)216, (uint8_t)36, (uint8_t)46, (uint8_t)254, (uint8_t)205, (uint8_t)224, (uint8_t)217, (uint8_t)7, (uint8_t)16, (uint8_t)96, (uint8_t)8, (uint8_t)207, (uint8_t)120, (uint8_t)33, (uint8_t)243, (uint8_t)186, (uint8_t)61, (uint8_t)230, (uint8_t)154, (uint8_t)46, (uint8_t)41, (uint8_t)173, (uint8_t)15, (uint8_t)146, (uint8_t)19, (uint8_t)2, (uint8_t)127, (uint8_t)131, (uint8_t)114, (uint8_t)162, (uint8_t)3, (uint8_t)171, (uint8_t)86, (uint8_t)19, (uint8_t)83, (uint8_t)160, (uint8_t)105, (uint8_t)208, (uint8_t)62, (uint8_t)177, (uint8_t)222, (uint8_t)35, (uint8_t)175, (uint8_t)48, (uint8_t)169, (uint8_t)112, (uint8_t)161, (uint8_t)218, (uint8_t)12, (uint8_t)25, (uint8_t)34, (uint8_t)63, (uint8_t)60, (uint8_t)47, (uint8_t)133, (uint8_t)240, (uint8_t)45, (uint8_t)229, (uint8_t)156, (uint8_t)145, (uint8_t)197, (uint8_t)97, (uint8_t)138, (uint8_t)26, (uint8_t)30, (uint8_t)132, (uint8_t)57, (uint8_t)122, (uint8_t)7, (uint8_t)28, (uint8_t)109, (uint8_t)248, (uint8_t)217, (uint8_t)191, (uint8_t)75, (uint8_t)248, (uint8_t)103, (uint8_t)140, (uint8_t)103, (uint8_t)234, (uint8_t)58, (uint8_t)176, (uint8_t)176, (uint8_t)195, (uint8_t)150, (uint8_t)120, (uint8_t)53, (uint8_t)213, (uint8_t)158, (uint8_t)45, (uint8_t)147, (uint8_t)88, (uint8_t)73, (uint8_t)59, (uint8_t)232, (uint8_t)227, (uint8_t)49, (uint8_t)62, (uint8_t)91, (uint8_t)155, (uint8_t)149, (uint8_t)68, (uint8_t)138, (uint8_t)94, (uint8_t)102, (uint8_t)204, (uint8_t)52, (uint8_t)23, (uint8_t)0, (uint8_t)235, (uint8_t)223, (uint8_t)150, (uint8_t)136, (uint8_t)192, (uint8_t)21, (uint8_t)134, (uint8_t)213, (uint8_t)134, (uint8_t)90, (uint8_t)60, (uint8_t)204, (uint8_t)201, (uint8_t)25, (uint8_t)177, (uint8_t)219, (uint8_t)165, (uint8_t)186, (uint8_t)138, (uint8_t)209, (uint8_t)64, (uint8_t)89, (uint8_t)127, (uint8_t)55, (uint8_t)245, (uint8_t)56, (uint8_t)201, (uint8_t)148, (uint8_t)141, (uint8_t)31, (uint8_t)125, (uint8_t)26, (uint8_t)213, (uint8_t)158, (uint8_t)60, (uint8_t)22, (uint8_t)87, (uint8_t)162, (uint8_t)145, (uint8_t)110, (uint8_t)84, (uint8_t)114, (uint8_t)154, (uint8_t)136, (uint8_t)230, (uint8_t)24, (uint8_t)104, (uint8_t)19, (uint8_t)0, (uint8_t)164, (uint8_t)161, (uint8_t)164, (uint8_t)191, (uint8_t)101, (uint8_t)147, (uint8_t)116, (uint8_t)131, (uint8_t)228, (uint8_t)37, (uint8_t)33, (uint8_t)160, (uint8_t)244, (uint8_t)86, (uint8_t)254, (uint8_t)228, (uint8_t)2, (uint8_t)124, (uint8_t)207, (uint8_t)233, (uint8_t)130, (uint8_t)66, (uint8_t)26, (uint8_t)232, (uint8_t)7, (uint8_t)189, (uint8_t)47, (uint8_t)119, (uint8_t)163, (uint8_t)116, (uint8_t)163, (uint8_t)70, (uint8_t)149, (uint8_t)6, (uint8_t)129, (uint8_t)254, (uint8_t)238, (uint8_t)112, (uint8_t)140, (uint8_t)116, (uint8_t)206, (uint8_t)227, (uint8_t)150, (uint8_t)206, (uint8_t)39, (uint8_t)240, (uint8_t)59, (uint8_t)16, (uint8_t)59, (uint8_t)38, (uint8_t)128, (uint8_t)40, (uint8_t)64, (uint8_t)174, (uint8_t)46, (uint8_t)55, (uint8_t)190, (uint8_t)1, (uint8_t)122, (uint8_t)120, (uint8_t)236, (uint8_t)73, (uint8_t)50, (uint8_t)255, (uint8_t)244, (uint8_t)93, (uint8_t)133, (uint8_t)110, (uint8_t)45, (uint8_t)181, (uint8_t)189, (uint8_t)133, (uint8_t)228, (uint8_t)58, (uint8_t)51, (uint8_t)59, (uint8_t)238, (uint8_t)158, (uint8_t)205, (uint8_t)122, (uint8_t)152, (uint8_t)161};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_length_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_length_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)3596, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)142, (uint8_t)151, (uint8_t)113, (uint8_t)173, (uint8_t)95, (uint8_t)75, (uint8_t)207, (uint8_t)141, (uint8_t)222, (uint8_t)63, (uint8_t)46, (uint8_t)217, (uint8_t)22, (uint8_t)23, (uint8_t)93, (uint8_t)185, (uint8_t)111, (uint8_t)194, (uint8_t)118, (uint8_t)0, (uint8_t)50, (uint8_t)232, (uint8_t)250, (uint8_t)251, (uint8_t)217, (uint8_t)40, (uint8_t)42, (uint8_t)134, (uint8_t)31, (uint8_t)161, (uint8_t)250, (uint8_t)156, (uint8_t)38, (uint8_t)65, (uint8_t)43, (uint8_t)43, (uint8_t)71, (uint8_t)162, (uint8_t)189, (uint8_t)75, (uint8_t)46, (uint8_t)100, (uint8_t)13, (uint8_t)127, (uint8_t)7, (uint8_t)176, (uint8_t)220, (uint8_t)101, (uint8_t)232, (uint8_t)128, (uint8_t)220, (uint8_t)171, (uint8_t)0, (uint8_t)37, (uint8_t)112, (uint8_t)4, (uint8_t)204, (uint8_t)25, (uint8_t)174, (uint8_t)19, (uint8_t)239, (uint8_t)215, (uint8_t)94, (uint8_t)194, (uint8_t)185, (uint8_t)71, (uint8_t)88, (uint8_t)80, (uint8_t)180, (uint8_t)61, (uint8_t)86, (uint8_t)171, (uint8_t)5, (uint8_t)108, (uint8_t)159, (uint8_t)97, (uint8_t)21, (uint8_t)247, (uint8_t)149, (uint8_t)24, (uint8_t)207, (uint8_t)13, (uint8_t)237, (uint8_t)24, (uint8_t)3, (uint8_t)38, (uint8_t)71, (uint8_t)184, (uint8_t)33, (uint8_t)63, (uint8_t)224, (uint8_t)128, (uint8_t)235, (uint8_t)154, (uint8_t)171, (uint8_t)196, (uint8_t)243, (uint8_t)97, (uint8_t)70, (uint8_t)67, (uint8_t)211, (uint8_t)15, (uint8_t)167, (uint8_t)74, (uint8_t)108, (uint8_t)75, (uint8_t)170, (uint8_t)146, (uint8_t)166, (uint8_t)87, (uint8_t)2, (uint8_t)140, (uint8_t)71, (uint8_t)56, (uint8_t)56, (uint8_t)129, (uint8_t)30, (uint8_t)76, (uint8_t)128, (uint8_t)104, (uint8_t)31, (uint8_t)72, (uint8_t)117, (uint8_t)46, (uint8_t)155, (uint8_t)150, (uint8_t)199, (uint8_t)197, (uint8_t)76, (uint8_t)67, (uint8_t)240, (uint8_t)62, (uint8_t)224, (uint8_t)207, (uint8_t)31, (uint8_t)134, (uint8_t)184, (uint8_t)46, (uint8_t)130, (uint8_t)202, (uint8_t)102, (uint8_t)223, (uint8_t)121, (uint8_t)172, (uint8_t)8, (uint8_t)23, (uint8_t)159, (uint8_t)26, (uint8_t)10, (uint8_t)96, (uint8_t)219, (uint8_t)51, (uint8_t)25, (uint8_t)238, (uint8_t)171, (uint8_t)200, (uint8_t)67, (uint8_t)209, (uint8_t)90, (uint8_t)135, (uint8_t)254, (uint8_t)251, (uint8_t)192, (uint8_t)143, (uint8_t)254, (uint8_t)107, (uint8_t)7, (uint8_t)246, (uint8_t)129, (uint8_t)59, (uint8_t)249, (uint8_t)107, (uint8_t)152, (uint8_t)225, (uint8_t)64, (uint8_t)53, (uint8_t)27, (uint8_t)205, (uint8_t)17, (uint8_t)150, (uint8_t)172, (uint8_t)139, (uint8_t)179, (uint8_t)218, (uint8_t)41, (uint8_t)113, (uint8_t)131, (uint8_t)28, (uint8_t)237, (uint8_t)235, (uint8_t)107, (uint8_t)228, (uint8_t)72, (uint8_t)93, (uint8_t)203, (uint8_t)61, (uint8_t)137, (uint8_t)42, (uint8_t)252, (uint8_t)84, (uint8_t)44, (uint8_t)20, (uint8_t)6, (uint8_t)169, (uint8_t)52, (uint8_t)181, (uint8_t)127, (uint8_t)14, (uint8_t)55, (uint8_t)199, (uint8_t)58, (uint8_t)135, (uint8_t)5, (uint8_t)44, (uint8_t)47, (uint8_t)40, (uint8_t)20, (uint8_t)143, (uint8_t)9, (uint8_t)27, (uint8_t)41, (uint8_t)228, (uint8_t)59, (uint8_t)195, (uint8_t)215, (uint8_t)199, (uint8_t)214, (uint8_t)231, (uint8_t)107, (uint8_t)227, (uint8_t)173, (uint8_t)185, (uint8_t)120, (uint8_t)33, (uint8_t)107, (uint8_t)215, (uint8_t)98, (uint8_t)237, (uint8_t)104, (uint8_t)132, (uint8_t)81, (uint8_t)113, (uint8_t)36, (uint8_t)234, (uint8_t)174, (uint8_t)5, (uint8_t)64, (uint8_t)158, (uint8_t)88};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)10035, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_framerate_SET((float)1.0979723E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        {
            char16_t* uri = u"iwruYwrdxjxgjilspdchxidbnaVggcdybmymFrPiwjsliwMNvqcQmcacvskpimkpipmwjqirtiqzcfsljmgnmtdobvxdgzKwxxlqt";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_v_SET((uint16_t)(uint16_t)49655, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)63929, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)2961, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1986013198L, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_h_SET((uint16_t)(uint16_t)38961, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)54538, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)54549, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)393694024L, PH.base.pack) ;
        p270_framerate_SET((float)3.1184466E37F, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        {
            char16_t* uri = u"mdzwnkevulkesqUwbzzrkJylageyenkgilmqlkbmWyerwFaeayeaqMtziejbpNgbhxhmwcwuxfuBmohrsiojjvjzefywczwZxwzowirVblsezlrtfepHblkprsuajjwegaLahwtgndfqpxeitdumbnBsybbPuhvgfmekhmqw";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_component_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"qybxddouWfhzdi";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"phmmxpujvpbKhouaflqjwlxsjsiImuavZsxpxkvjlsxQirEdxEhrflkvhsBkj";
            p299_password_SET_(password, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)22489, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)104, (uint8_t)108, (uint8_t)62, (uint8_t)244, (uint8_t)180, (uint8_t)192, (uint8_t)185, (uint8_t)198};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)47312, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)13571, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)175, (uint8_t)235, (uint8_t)74, (uint8_t)68, (uint8_t)90, (uint8_t)32, (uint8_t)207, (uint8_t)215};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_time_usec_SET((uint64_t)2157251526487921455L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)40763, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)2380015174L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            char16_t* name = u"uergcnzjkqvffxiEwwqdhyiupbjagvsrqrsvrgwdzpqknqXTogeoDvqpphjlyqkqTgqPtdhh";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_version_minor_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)844082885L, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1305355254L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)4857865205420038110L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)144, (uint8_t)159, (uint8_t)67, (uint8_t)153, (uint8_t)47, (uint8_t)78, (uint8_t)40, (uint8_t)204, (uint8_t)109, (uint8_t)47, (uint8_t)125, (uint8_t)230, (uint8_t)20, (uint8_t)20, (uint8_t)149, (uint8_t)216};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        {
            char16_t* param_id = u"kwCaqqnffwfeoiFc";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t) -8842, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"Ztlngijtmylq";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)45024, PH.base.pack) ;
        {
            char16_t* param_value = u"jvfxsygkxaohqmGrdxbhYjxreeajtoyun";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)10654, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_id = u"grezlmeszkmnmi";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"zzjAwcnilzcjyzsdwLPgxshrltrmvAhjmnhcjyyeangwbznphzdlxviqrdyfbqFnobezsSNSDfaqlaenlstfdNGLdnsuktyffahq";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
        {
            char16_t* param_id = u"Oupjvslhcflte";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_CUSTOM, PH.base.pack) ;
        {
            char16_t* param_value = u"adntptzbjnbrnlVvlzwzAfzubevamqpqkviPxhvbosCicfghzNowxadqcoOSbkjpaaQcswjymzXXthEwzUz";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        {
            uint16_t distances[] =  {(uint16_t)59295, (uint16_t)62395, (uint16_t)17591, (uint16_t)23234, (uint16_t)61648, (uint16_t)30936, (uint16_t)30154, (uint16_t)11086, (uint16_t)43930, (uint16_t)15156, (uint16_t)12306, (uint16_t)23896, (uint16_t)19772, (uint16_t)34804, (uint16_t)58249, (uint16_t)48037, (uint16_t)14730, (uint16_t)817, (uint16_t)36370, (uint16_t)4240, (uint16_t)366, (uint16_t)15960, (uint16_t)32124, (uint16_t)27882, (uint16_t)65249, (uint16_t)16514, (uint16_t)61893, (uint16_t)31158, (uint16_t)40034, (uint16_t)17464, (uint16_t)6395, (uint16_t)49209, (uint16_t)40212, (uint16_t)36767, (uint16_t)47158, (uint16_t)27870, (uint16_t)8482, (uint16_t)29651, (uint16_t)6502, (uint16_t)19108, (uint16_t)27331, (uint16_t)56813, (uint16_t)29770, (uint16_t)62859, (uint16_t)7754, (uint16_t)58721, (uint16_t)37193, (uint16_t)6772, (uint16_t)39793, (uint16_t)49631, (uint16_t)64536, (uint16_t)19869, (uint16_t)33570, (uint16_t)29953, (uint16_t)6635, (uint16_t)9051, (uint16_t)60912, (uint16_t)24246, (uint16_t)53898, (uint16_t)34722, (uint16_t)10384, (uint16_t)33768, (uint16_t)9276, (uint16_t)39570, (uint16_t)50555, (uint16_t)28696, (uint16_t)25685, (uint16_t)8396, (uint16_t)50970, (uint16_t)13050, (uint16_t)40167, (uint16_t)35461};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)1364087422723474166L, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)12842, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)50346, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}


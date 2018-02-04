
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
    switch(get_bits(data, 50, 4))
    {
        case 0:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        case 1:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED;
        case 2:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED;
        case 3:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED;
        case 4:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED;
        case 5:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED;
        case 6:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        case 7:
            return e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
INLINER e_MAV_STATE p0_system_status_GET(Pack * src)//System status flag, see MAV_STATE ENUM
{
    uint8_t * data = src->data;
    return  0 + (int)get_bits(data, 54, 4);
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
    switch(get_bits(data, 152, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
/**
*Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of
*	1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_enabled_GET(Pack * src)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 157, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
}
/**
*Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not
*	enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSO*/
INLINER e_MAV_SYS_STATUS_SENSOR p1_onboard_control_sensors_health_GET(Pack * src)
{
    uint8_t * data = src->data;
    switch(get_bits(data, 162, 5))
    {
        case 0:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO;
        case 1:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        case 2:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG;
        case 3:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        case 4:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        case 5:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS;
        case 6:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        case 7:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        case 8:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        case 9:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH;
        case 10:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        case 11:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        case 12:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        case 13:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        case 14:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        case 15:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        case 16:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        case 17:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        case 18:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        case 19:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2;
        case 20:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE;
        case 21:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS;
        case 22:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN;
        case 23:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR;
        case 24:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING;
        case 25:
            return e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
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
    switch(get_bits(data, 40, 4))
    {
        case 0:
            return e_MAV_MODE_MAV_MODE_PREFLIGHT;
        case 1:
            return e_MAV_MODE_MAV_MODE_MANUAL_DISARMED;
        case 2:
            return e_MAV_MODE_MAV_MODE_TEST_DISARMED;
        case 3:
            return e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED;
        case 4:
            return e_MAV_MODE_MAV_MODE_GUIDED_DISARMED;
        case 5:
            return e_MAV_MODE_MAV_MODE_AUTO_DISARMED;
        case 6:
            return e_MAV_MODE_MAV_MODE_MANUAL_ARMED;
        case 7:
            return e_MAV_MODE_MAV_MODE_TEST_ARMED;
        case 8:
            return e_MAV_MODE_MAV_MODE_STABILIZE_ARMED;
        case 9:
            return e_MAV_MODE_MAV_MODE_GUIDED_ARMED;
        case 10:
            return e_MAV_MODE_MAV_MODE_AUTO_ARMED;
        default: ;//assert(false);//("Unknown enum ID " + id);
    }
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
/**
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
    int32_t id;
    switch(src)
    {
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
            id = 0;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT:
            id = 1;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT:
            id = 2;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
            id = 3;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION:
            id = 4;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP:
            id = 5;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
            id = 6;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:
            id = 7;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
            id = 8;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN:
            id = 9;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET:
            id = 10;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION:
            id = 11;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION:
            id = 12;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2:
            id = 13;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE:
            id = 14;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY:
            id = 15;
            break;
        case e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION:
            id = 16;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 5, data, 416);
}
/**
*UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
*	use uid*/
INLINER void p148_uid2_SET(uint8_t*  src, int32_t pos, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 421)insert_field(dst, 421, 0);
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
    set_bits(- 0 +   src, 3, data, 236);
}
INLINER void p149_x_SET(float  src, Bounds_Inside * dst)//X Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 239)insert_field(dst, 239, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_y_SET(float  src, Bounds_Inside * dst)//Y Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 240)insert_field(dst, 240, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_z_SET(float  src, Bounds_Inside * dst)//Z Position of the landing target on MAV_FRAME
{
    if(dst->base.field_bit != 241)insert_field(dst, 241, 0);
    uint8_t * data = dst->base.pack->data;
    set_bytes(floatToIntBits(src), 4, data,  dst->BYTE);
}
INLINER void p149_q_SET(float*  src, int32_t pos, Bounds_Inside * dst) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
{
    if(dst->base.field_bit != 242)insert_field(dst, 242, 0);
    uint8_t * data = dst->base.pack->data;
    for(int32_t BYTE =  dst->BYTE, src_max = pos + 4; pos < src_max; pos++, BYTE += 4)
        set_bytes(floatToIntBits(src[pos]), 4, data,  BYTE);
}/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	the landing targe*/
INLINER void p149_position_valid_SET(uint8_t  src, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 243)insert_field(dst, 243, 0);
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
    int32_t id;
    switch(src)
    {
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE:
            id = 0;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ:
            id = 1;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT:
            id = 2;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL:
            id = 3;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS:
            id = 4;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS:
            id = 5;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL:
            id = 6;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE:
            id = 7;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL:
            id = 8;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS:
            id = 9;
            break;
        case e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH:
            id = 10;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 320);
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
    int32_t id;
    switch(src)
    {
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT:
            id = 0;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP:
            id = 1;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP:
            id = 2;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ:
            id = 3;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT:
            id = 4;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY:
            id = 5;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY:
            id = 6;
            break;
        case e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 488);
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
    int32_t id;
    switch(src)
    {
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            id = 0;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED:
            id = 1;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED:
            id = 2;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED:
            id = 3;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED:
            id = 4;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED:
            id = 5;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
            id = 6;
            break;
        case e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED:
            id = 7;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 4, data, 296);
}
INLINER void p234_landed_state_SET(e_MAV_LANDED_STATE  src, Pack * dst)//The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 300);
}
INLINER void p234_gps_fix_type_SET(e_GPS_FIX_TYPE  src, Pack * dst)//See the GPS_FIX_TYPE enum.
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 4, data, 303);
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
    set_bits(- 0 +   src, 2, data, 200);
}
INLINER void p246_emitter_type_SET(e_ADSB_EMITTER_TYPE  src, Pack * dst)//Type from ADSB_EMITTER_TYPE enum
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 5, data, 202);
}
INLINER void p246_flags_SET(e_ADSB_FLAGS  src, Pack * dst)//Flags to indicate various statuses including valid data fields
{
    uint8_t * data = dst->data;
    int32_t id;
    switch(src)
    {
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_COORDS:
            id = 0;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE:
            id = 1;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING:
            id = 2;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY:
            id = 3;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_CALLSIGN:
            id = 4;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK:
            id = 5;
            break;
        case e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED:
            id = 6;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 207);
}
INLINER void p246_callsign_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //The callsign, 8+null
{
    if(dst->base.field_bit != 210 && insert_field(dst, 210, items) ||
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
    set_bits(- 0 +   src, 2, data, 128);
}
INLINER void p247_action_SET(e_MAV_COLLISION_ACTION  src, Pack * dst)//Action that is being taken to avoid this collision
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 3, data, 130);
}
INLINER void p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL  src, Pack * dst)//How concerned the aircraft is about this collision
{
    uint8_t * data = dst->data;
    set_bits(- 0 +   src, 2, data, 133);
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
    set_bits(- 0 +   src, 4, data, 0);
}
INLINER void p253_text_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Status text message, without null termination character
{
    if(dst->base.field_bit != 4 && insert_field(dst, 4, items) ||
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
    int32_t id;
    switch(src)
    {
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO:
            id = 0;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE:
            id = 1;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES:
            id = 2;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE:
            id = 3;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE:
            id = 4;
            break;
        case e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE:
            id = 5;
            break;
        default: ;// assert(false);//("Unknown enum" + id);
    }
    set_bits(id, 3, data, 728);
}
INLINER void p259_cam_definition_uri_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst) //Camera definition URI (if any, otherwise only basic functions will be available).
{
    if(dst->base.field_bit != 731 && insert_field(dst, 731, items) ||
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
    set_bits(- 0 +   src, 3, data, 120);
}
INLINER void p310_mode_SET(e_UAVCAN_NODE_MODE  src, Pack * dst)//Generalized operating mode.
{
    uint8_t * data = dst->data;
    int32_t id;
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
    set_bits(id, 3, data, 123);
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
    set_bits(- 0 +   src, 3, data, 4);
}
/**
*Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination
*	(NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the
*	ID is stored as strin*/
INLINER void p324_param_id_SET(char16_t *  src, int32_t pos, int32_t items, Bounds_Inside * dst)
{
    if(dst->base.field_bit != 9 && insert_field(dst, 9, items) ||
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
    if(dst->base.field_bit != 10 && insert_field(dst, 10, items) ||
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED);
    assert(p0_custom_mode_GET(pack) == (uint32_t)1060580138L);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_HEXAROTOR);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_EMERGENCY);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_TEST_Channel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)51255);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)4324);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)62993);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)61);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)36571);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)5213);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)22582);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)25889);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)28302);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)34970);
};


void c_TEST_Channel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2242517153L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)3198018199476426707L);
};


void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vx_GET(pack) == (float)2.5359652E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2116895762L);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)1146);
    assert(p3_afx_GET(pack) == (float)1.2898879E38F);
    assert(p3_yaw_rate_GET(pack) == (float)1.9060226E38F);
    assert(p3_y_GET(pack) == (float)2.8609913E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p3_vz_GET(pack) == (float)2.567307E38F);
    assert(p3_yaw_GET(pack) == (float) -9.536966E37F);
    assert(p3_afz_GET(pack) == (float) -2.846884E38F);
    assert(p3_x_GET(pack) == (float)4.26203E37F);
    assert(p3_z_GET(pack) == (float)7.6401357E37F);
    assert(p3_afy_GET(pack) == (float) -2.3905748E38F);
    assert(p3_vy_GET(pack) == (float) -1.9872486E38F);
};


void c_TEST_Channel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)3812964835L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p4_time_usec_GET(pack) == (uint64_t)4011420812040978761L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)70);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p5_passkey_LEN(ph) == 9);
    {
        char16_t * exemplary = u"qykdsDevf";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)178);
};


void c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)36);
};


void c_TEST_Channel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 13);
    {
        char16_t * exemplary = u"tibqiPwObeTsu";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_custom_mode_GET(pack) == (uint32_t)3465195074L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_PREFLIGHT);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_TEST_Channel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"nWC";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -11262);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)56);
};


void c_TEST_Channel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)148);
};


void c_TEST_Channel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)39806);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64);
    assert(p22_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"h";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_value_GET(pack) == (float)1.3713126E38F);
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)4123);
};


void c_TEST_Channel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"y";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)3);
    assert(p23_param_value_GET(pack) == (float) -7.0358137E37F);
};


void c_TEST_Channel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_lat_GET(pack) == (int32_t)1443088790);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -558919851);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1404997642L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)57104);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p24_h_acc_TRY(ph) == (uint32_t)3288883985L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)50883);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)39781);
    assert(p24_time_usec_GET(pack) == (uint64_t)499163218996559150L);
    assert(p24_lon_GET(pack) == (int32_t)114680914);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)33617);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1012419365L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)2861303950L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p24_alt_GET(pack) == (int32_t)1417941598);
};


void c_TEST_Channel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)180, (uint8_t)101, (uint8_t)218, (uint8_t)73, (uint8_t)219, (uint8_t)219, (uint8_t)235, (uint8_t)19, (uint8_t)64, (uint8_t)65, (uint8_t)246, (uint8_t)86, (uint8_t)217, (uint8_t)215, (uint8_t)178, (uint8_t)5, (uint8_t)252, (uint8_t)93, (uint8_t)3, (uint8_t)183} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)26, (uint8_t)64, (uint8_t)252, (uint8_t)31, (uint8_t)65, (uint8_t)74, (uint8_t)167, (uint8_t)2, (uint8_t)144, (uint8_t)237, (uint8_t)199, (uint8_t)84, (uint8_t)73, (uint8_t)19, (uint8_t)68, (uint8_t)235, (uint8_t)81, (uint8_t)94, (uint8_t)247, (uint8_t)4} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)114, (uint8_t)180, (uint8_t)153, (uint8_t)61, (uint8_t)214, (uint8_t)200, (uint8_t)123, (uint8_t)36, (uint8_t)132, (uint8_t)222, (uint8_t)158, (uint8_t)47, (uint8_t)14, (uint8_t)125, (uint8_t)232, (uint8_t)51, (uint8_t)117, (uint8_t)188, (uint8_t)231, (uint8_t)121} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)181, (uint8_t)61, (uint8_t)194, (uint8_t)99, (uint8_t)44, (uint8_t)32, (uint8_t)45, (uint8_t)181, (uint8_t)141, (uint8_t)32, (uint8_t)159, (uint8_t)69, (uint8_t)200, (uint8_t)8, (uint8_t)63, (uint8_t)108, (uint8_t)154, (uint8_t)101, (uint8_t)197, (uint8_t)155} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)176);
    {
        uint8_t exemplary[] =  {(uint8_t)216, (uint8_t)250, (uint8_t)23, (uint8_t)50, (uint8_t)218, (uint8_t)87, (uint8_t)13, (uint8_t)27, (uint8_t)83, (uint8_t)89, (uint8_t)134, (uint8_t)85, (uint8_t)147, (uint8_t)118, (uint8_t)249, (uint8_t)227, (uint8_t)238, (uint8_t)143, (uint8_t)227, (uint8_t)67} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -2529);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)12414);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)1621);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)4195035627L);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)4274);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -15466);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -23804);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -9897);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)30655);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -13993);
};


void c_TEST_Channel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -22445);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -17759);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -27893);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -22131);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)13999);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)21572);
    assert(p27_time_usec_GET(pack) == (uint64_t)2185917278329366456L);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)9074);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)9019);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -29433);
};


void c_TEST_Channel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)7069648733676375890L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -19865);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)26205);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -4349);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -11262);
};


void c_TEST_Channel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)2684787132L);
    assert(p29_press_diff_GET(pack) == (float)1.1910772E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)3316);
    assert(p29_press_abs_GET(pack) == (float)2.9470258E38F);
};


void c_TEST_Channel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_yawspeed_GET(pack) == (float)2.2654634E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -1.936333E38F);
    assert(p30_roll_GET(pack) == (float) -2.1461897E38F);
    assert(p30_rollspeed_GET(pack) == (float)2.441565E38F);
    assert(p30_yaw_GET(pack) == (float)2.3116E38F);
    assert(p30_pitch_GET(pack) == (float)7.469015E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)2254311448L);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q2_GET(pack) == (float) -1.4622262E38F);
    assert(p31_rollspeed_GET(pack) == (float)2.8034677E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)3204280937L);
    assert(p31_q3_GET(pack) == (float) -1.8792308E38F);
    assert(p31_yawspeed_GET(pack) == (float) -9.218744E36F);
    assert(p31_q1_GET(pack) == (float) -2.8604319E38F);
    assert(p31_q4_GET(pack) == (float)6.939969E37F);
    assert(p31_pitchspeed_GET(pack) == (float) -1.831005E38F);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vx_GET(pack) == (float)1.9415148E38F);
    assert(p32_z_GET(pack) == (float) -2.7297203E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)3952334373L);
    assert(p32_x_GET(pack) == (float) -3.0636393E38F);
    assert(p32_y_GET(pack) == (float) -2.0961057E38F);
    assert(p32_vy_GET(pack) == (float)5.8132296E37F);
    assert(p32_vz_GET(pack) == (float)5.6205295E37F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)31509);
    assert(p33_lat_GET(pack) == (int32_t) -1998603350);
    assert(p33_alt_GET(pack) == (int32_t) -1771582194);
    assert(p33_relative_alt_GET(pack) == (int32_t) -754597052);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1626725965L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)19453);
    assert(p33_lon_GET(pack) == (int32_t) -2099759895);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)28480);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -3740);
};


void c_TEST_Channel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -26844);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)14911);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)4195384067L);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)4751);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -29964);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -15668);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -5405);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -26774);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -2515);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)32);
};


void c_TEST_Channel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)5290);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)48665);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)62484);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)34231);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)16410);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)62099);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)32202);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)439454459L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)61503);
};


void c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)64);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)25335);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)54166);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)39226);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)30656);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)62327);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)15811);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)24026);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)21278);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)61762);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)26914);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)62428);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)716);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)19883);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)32540);
    assert(p36_time_usec_GET(pack) == (uint32_t)316320968L);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)2717);
};


void c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -938);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)30639);
};


void c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -26582);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)1174);
};


void c_TEST_Channel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL);
    assert(p39_param4_GET(pack) == (float) -2.4730587E38F);
    assert(p39_y_GET(pack) == (float) -2.8680394E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p39_param3_GET(pack) == (float) -2.0935497E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p39_x_GET(pack) == (float)2.1805667E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_param1_GET(pack) == (float) -2.2971332E38F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)21421);
    assert(p39_z_GET(pack) == (float)8.042283E37F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p39_param2_GET(pack) == (float)9.873219E37F);
};


void c_TEST_Channel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)1034);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_TEST_Channel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)55467);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_TEST_Channel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)2176);
};


void c_TEST_Channel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)65385);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_TEST_Channel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)10227);
};


void c_TEST_Channel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_longitude_GET(pack) == (int32_t)1548038265);
    assert(p48_latitude_GET(pack) == (int32_t) -1107747989);
    assert(p48_time_usec_TRY(ph) == (uint64_t)7986251425166636900L);
    assert(p48_altitude_GET(pack) == (int32_t) -104330735);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)7499366453014257578L);
    assert(p49_longitude_GET(pack) == (int32_t)2021373281);
    assert(p49_altitude_GET(pack) == (int32_t) -160417034);
    assert(p49_latitude_GET(pack) == (int32_t) -1230851608);
};


void c_TEST_Channel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)24009);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p50_param_value_max_GET(pack) == (float)1.5763475E38F);
    assert(p50_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"framFjuskfblfvf";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_min_GET(pack) == (float) -1.3585218E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p50_param_value0_GET(pack) == (float) -1.8214967E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p50_scale_GET(pack) == (float) -2.2894135E38F);
};


void c_TEST_Channel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)43580);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)75);
};


void c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1y_GET(pack) == (float)1.8246376E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p54_p1z_GET(pack) == (float)2.4033465E38F);
    assert(p54_p2z_GET(pack) == (float)2.4105056E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p54_p2x_GET(pack) == (float)1.8909043E38F);
    assert(p54_p2y_GET(pack) == (float)2.1583709E38F);
    assert(p54_p1x_GET(pack) == (float) -2.6919992E38F);
};


void c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float)1.0506598E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p55_p2z_GET(pack) == (float)2.7621934E38F);
    assert(p55_p1z_GET(pack) == (float) -2.5925513E38F);
    assert(p55_p1y_GET(pack) == (float) -2.0453664E38F);
    assert(p55_p2y_GET(pack) == (float) -2.8159947E38F);
    assert(p55_p2x_GET(pack) == (float) -1.6551789E38F);
};


void c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_rollspeed_GET(pack) == (float)2.030161E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)1915774830010135902L);
    {
        float exemplary[] =  {2.5586175E38F, 3.2426476E38F, 3.3034587E38F, 1.031372E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.2544935E38F, -1.3090321E38F, -7.6117075E36F, -3.1805494E38F, 3.231305E38F, -1.3326197E38F, -2.9180166E37F, 9.425019E37F, 2.1401365E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_yawspeed_GET(pack) == (float)9.815524E36F);
    assert(p61_pitchspeed_GET(pack) == (float)1.9826268E38F);
};


void c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -11707);
    assert(p62_alt_error_GET(pack) == (float)2.3127817E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)6638);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -7825);
    assert(p62_xtrack_error_GET(pack) == (float)2.9014549E38F);
    assert(p62_aspd_error_GET(pack) == (float) -4.0966386E37F);
    assert(p62_nav_pitch_GET(pack) == (float) -1.8868835E38F);
    assert(p62_nav_roll_GET(pack) == (float) -1.9347344E38F);
};


void c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vx_GET(pack) == (float) -5.0829863E37F);
    assert(p63_lat_GET(pack) == (int32_t)1367472929);
    assert(p63_vy_GET(pack) == (float) -7.3954726E37F);
    assert(p63_relative_alt_GET(pack) == (int32_t) -669613037);
    assert(p63_lon_GET(pack) == (int32_t) -1290148841);
    {
        float exemplary[] =  {4.3032686E37F, -3.3043925E38F, 1.7522638E37F, -2.1548908E38F, 2.6958395E38F, -1.507681E38F, -1.9416324E38F, 2.7541218E38F, 1.0569717E38F, -3.1261572E38F, -2.0381335E37F, 1.9274942E37F, 2.7061322E38F, 3.2801656E38F, 1.848939E37F, 1.4732196E38F, 1.0297665E38F, -2.7870622E37F, 1.3491902E36F, -1.6531066E38F, -1.1556947E38F, 1.3148132E38F, 1.7905364E38F, 2.4111812E37F, -1.290915E38F, 1.6316415E37F, 1.204887E38F, -1.5766196E38F, -1.5768208E37F, 2.7508154E38F, 3.1390063E38F, 3.2842574E38F, -2.799611E38F, 4.0775916E37F, -2.3163526E38F, -2.0488146E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_time_usec_GET(pack) == (uint64_t)335045694904186663L);
    assert(p63_alt_GET(pack) == (int32_t)170972593);
    assert(p63_vz_GET(pack) == (float) -1.2070635E38F);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
};


void c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vx_GET(pack) == (float) -5.2273966E37F);
    assert(p64_x_GET(pack) == (float)2.925525E38F);
    assert(p64_vy_GET(pack) == (float)2.6171121E38F);
    assert(p64_z_GET(pack) == (float)1.02569204E37F);
    assert(p64_ay_GET(pack) == (float)1.6268983E37F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p64_y_GET(pack) == (float) -3.1103284E38F);
    assert(p64_ax_GET(pack) == (float)2.5563645E38F);
    assert(p64_vz_GET(pack) == (float)2.1655444E38F);
    assert(p64_az_GET(pack) == (float)2.875956E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)4317392645009816901L);
    {
        float exemplary[] =  {-1.3613394E37F, 2.5741477E38F, 1.7874338E38F, 1.0178005E38F, 1.6722275E38F, -1.6315701E38F, -1.4450042E38F, 1.1964758E38F, 1.5416366E38F, -5.573217E37F, -2.7760096E38F, 1.188656E38F, -7.890439E37F, 2.1677586E38F, -4.4547635E37F, 1.7279702E38F, -2.8537478E38F, 3.1657858E38F, 2.6241907E38F, 2.6692401E38F, 2.018655E38F, 2.7895207E38F, -2.538418E37F, -7.8054856E37F, -3.0850519E38F, 1.5630342E38F, 1.7423997E38F, -4.5964666E37F, -2.1680815E38F, 3.2695774E38F, -2.8202881E38F, 4.4809284E37F, 1.8079497E38F, -2.995123E38F, 2.0301725E38F, -3.3471916E37F, -2.5865127E38F, -3.3549012E38F, -2.9246915E37F, -3.0546023E38F, -2.9720985E38F, 2.9709889E38F, 8.237016E36F, -1.3781805E38F, -1.1599966E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_TEST_Channel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)14019);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)7479);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)41492);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)39722);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)49688);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)36273);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)62715);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)30766);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)19864);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)55809);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)14572);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)10388);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)23451);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)77253232L);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)20523);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)2457);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)8414);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)45369);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)51503);
};


void c_TEST_Channel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)26411);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)128);
};


void c_TEST_Channel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)36726);
};


void c_TEST_Channel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -22903);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -21645);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)11518);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)4273);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -2498);
};


void c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)32744);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)58120);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)10505);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)45129);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)55176);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)57039);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)14203);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)59301);
};


void c_TEST_Channel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param4_GET(pack) == (float)2.2699821E38F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)11386);
    assert(p73_param1_GET(pack) == (float) -4.0458141E37F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_LOGGING_START);
    assert(p73_z_GET(pack) == (float) -2.0959286E38F);
    assert(p73_param3_GET(pack) == (float) -3.2134194E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p73_param2_GET(pack) == (float)3.3725096E38F);
    assert(p73_y_GET(pack) == (int32_t) -83858132);
    assert(p73_x_GET(pack) == (int32_t) -2039775292);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_TEST_Channel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)8178);
    assert(p74_climb_GET(pack) == (float) -1.9481005E38F);
    assert(p74_alt_GET(pack) == (float) -8.705219E37F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -21757);
    assert(p74_airspeed_GET(pack) == (float) -1.6576167E38F);
    assert(p74_groundspeed_GET(pack) == (float) -1.4176743E38F);
};


void c_TEST_Channel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
    assert(p75_z_GET(pack) == (float)2.770522E38F);
    assert(p75_x_GET(pack) == (int32_t) -219261861);
    assert(p75_param3_GET(pack) == (float)1.8315884E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p75_param1_GET(pack) == (float)1.9833775E37F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p75_param4_GET(pack) == (float)5.7881034E36F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p75_param2_GET(pack) == (float) -2.743197E38F);
    assert(p75_y_GET(pack) == (int32_t) -113448102);
};


void c_TEST_Channel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param6_GET(pack) == (float) -2.2959288E38F);
    assert(p76_param7_GET(pack) == (float) -1.8072177E38F);
    assert(p76_param5_GET(pack) == (float) -2.7841055E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p76_param4_GET(pack) == (float)1.351585E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p76_param1_GET(pack) == (float) -1.8340146E38F);
    assert(p76_param3_GET(pack) == (float)2.5268905E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS);
    assert(p76_param2_GET(pack) == (float) -1.6833519E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_TEST_Channel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)183);
    assert(p77_result_param2_TRY(ph) == (int32_t)665842807);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)171);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_HOME);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)154);
};


void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)4225741204L);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p81_roll_GET(pack) == (float)4.5945154E37F);
    assert(p81_pitch_GET(pack) == (float)3.0220742E38F);
    assert(p81_thrust_GET(pack) == (float)1.4718034E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p81_yaw_GET(pack) == (float)6.652891E37F);
};


void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2070898241L);
    assert(p82_thrust_GET(pack) == (float) -1.314141E36F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p82_body_yaw_rate_GET(pack) == (float) -2.2000601E38F);
    {
        float exemplary[] =  {-9.777236E37F, -2.6039803E38F, -3.3574221E38F, -1.3705904E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.0858724E38F);
    assert(p82_body_roll_rate_GET(pack) == (float) -1.5351622E38F);
};


void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_yaw_rate_GET(pack) == (float)4.239599E37F);
    assert(p83_body_pitch_rate_GET(pack) == (float)3.1390826E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)2033250346L);
    assert(p83_thrust_GET(pack) == (float)9.998173E37F);
    assert(p83_body_roll_rate_GET(pack) == (float) -2.8035207E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)37);
    {
        float exemplary[] =  {-7.904438E37F, -8.821262E37F, -6.275489E37F, 7.5294207E37F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float) -8.2111155E37F);
    assert(p84_vy_GET(pack) == (float) -3.3671682E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p84_z_GET(pack) == (float) -1.4436025E38F);
    assert(p84_afz_GET(pack) == (float) -4.105676E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p84_vz_GET(pack) == (float)1.497382E38F);
    assert(p84_yaw_rate_GET(pack) == (float)1.1079266E38F);
    assert(p84_yaw_GET(pack) == (float) -1.0709296E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)18054);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p84_vx_GET(pack) == (float)9.45532E37F);
    assert(p84_afx_GET(pack) == (float) -6.496121E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)3511398888L);
    assert(p84_x_GET(pack) == (float) -1.8201643E38F);
    assert(p84_y_GET(pack) == (float)1.1264997E36F);
};


void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p86_vy_GET(pack) == (float)2.0226205E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p86_lon_int_GET(pack) == (int32_t) -966399492);
    assert(p86_afx_GET(pack) == (float) -2.3135305E38F);
    assert(p86_yaw_GET(pack) == (float)6.723813E37F);
    assert(p86_afz_GET(pack) == (float)1.5777334E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)1962799749);
    assert(p86_vx_GET(pack) == (float) -1.544076E38F);
    assert(p86_vz_GET(pack) == (float) -2.9870685E38F);
    assert(p86_alt_GET(pack) == (float) -1.0835042E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3002894767L);
    assert(p86_yaw_rate_GET(pack) == (float) -3.2392503E38F);
    assert(p86_afy_GET(pack) == (float) -2.5404909E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)60122);
};


void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_afz_GET(pack) == (float)1.17171E37F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)1001772587L);
    assert(p87_vy_GET(pack) == (float) -1.6337131E38F);
    assert(p87_vx_GET(pack) == (float) -2.6443656E38F);
    assert(p87_yaw_rate_GET(pack) == (float)6.187788E37F);
    assert(p87_afy_GET(pack) == (float) -2.3721803E38F);
    assert(p87_alt_GET(pack) == (float)2.382105E37F);
    assert(p87_yaw_GET(pack) == (float) -1.3909719E38F);
    assert(p87_vz_GET(pack) == (float) -2.9371126E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)53839);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p87_afx_GET(pack) == (float) -3.2138715E38F);
    assert(p87_lon_int_GET(pack) == (int32_t) -1754130825);
    assert(p87_lat_int_GET(pack) == (int32_t) -676265800);
};


void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_y_GET(pack) == (float) -1.0917352E38F);
    assert(p89_roll_GET(pack) == (float)5.4484683E37F);
    assert(p89_yaw_GET(pack) == (float) -1.3802795E38F);
    assert(p89_x_GET(pack) == (float)1.2166309E38F);
    assert(p89_pitch_GET(pack) == (float) -1.7910382E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2024965337L);
    assert(p89_z_GET(pack) == (float) -1.6120388E38F);
};


void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)11000);
    assert(p90_pitch_GET(pack) == (float) -2.6244746E38F);
    assert(p90_rollspeed_GET(pack) == (float)3.298934E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -30525);
    assert(p90_yawspeed_GET(pack) == (float) -1.0768847E38F);
    assert(p90_alt_GET(pack) == (int32_t) -184169428);
    assert(p90_pitchspeed_GET(pack) == (float)1.7891363E38F);
    assert(p90_roll_GET(pack) == (float) -1.2666776E38F);
    assert(p90_lon_GET(pack) == (int32_t) -2056748264);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)18230);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)10279);
    assert(p90_yaw_GET(pack) == (float)6.295997E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)14439);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -30814);
    assert(p90_time_usec_GET(pack) == (uint64_t)6716823869778920230L);
    assert(p90_lat_GET(pack) == (int32_t) -2088499972);
};


void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_pitch_elevator_GET(pack) == (float) -1.6926099E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p91_aux4_GET(pack) == (float)3.0319442E38F);
    assert(p91_throttle_GET(pack) == (float) -2.086259E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p91_aux1_GET(pack) == (float)1.1346848E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)2.194926E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)8540019472069159366L);
    assert(p91_roll_ailerons_GET(pack) == (float)3.0765994E38F);
    assert(p91_aux3_GET(pack) == (float) -5.274441E37F);
    assert(p91_aux2_GET(pack) == (float)2.1515497E38F);
};


void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)55177);
    assert(p92_time_usec_GET(pack) == (uint64_t)1462258763290575984L);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)32008);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)64017);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)32616);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)54943);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)53302);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)7289);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)37584);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)1096);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)31592);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)11173);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)13188);
};


void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)8344546389011401049L);
    {
        float exemplary[] =  {2.5554183E38F, 1.5796311E37F, 3.1755946E38F, -2.316234E38F, -3.144315E38F, 2.3686692E38F, -5.880912E37F, 1.5459401E38F, -2.4586562E38F, 2.6649699E38F, -2.9820188E38F, 3.9155493E37F, 2.9556454E38F, -3.2946507E38F, 2.4246767E38F, -2.1724986E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)4135797548745712459L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)13037);
    assert(p100_flow_rate_y_TRY(ph) == (float)2.4640844E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)5168);
    assert(p100_flow_comp_m_x_GET(pack) == (float)4.90704E37F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -1.1287986E38F);
    assert(p100_time_usec_GET(pack) == (uint64_t)6904346464935342242L);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -2.6449457E38F);
    assert(p100_ground_distance_GET(pack) == (float)4.9035925E37F);
};


void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_z_GET(pack) == (float)2.086941E38F);
    assert(p101_usec_GET(pack) == (uint64_t)2819395135687547625L);
    assert(p101_yaw_GET(pack) == (float)3.4803672E37F);
    assert(p101_pitch_GET(pack) == (float) -4.288903E37F);
    assert(p101_y_GET(pack) == (float) -3.3646487E38F);
    assert(p101_roll_GET(pack) == (float) -2.1555616E38F);
    assert(p101_x_GET(pack) == (float)1.0158182E37F);
};


void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_yaw_GET(pack) == (float)1.386923E38F);
    assert(p102_pitch_GET(pack) == (float)1.2945842E37F);
    assert(p102_usec_GET(pack) == (uint64_t)5612488769664453163L);
    assert(p102_x_GET(pack) == (float) -7.2088773E36F);
    assert(p102_roll_GET(pack) == (float) -3.0252234E38F);
    assert(p102_z_GET(pack) == (float)2.8532562E38F);
    assert(p102_y_GET(pack) == (float)1.8284464E38F);
};


void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float) -1.4921893E38F);
    assert(p103_y_GET(pack) == (float) -3.240979E38F);
    assert(p103_z_GET(pack) == (float) -1.8257888E38F);
    assert(p103_usec_GET(pack) == (uint64_t)1543944488958140563L);
};


void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float) -6.0092495E37F);
    assert(p104_y_GET(pack) == (float) -2.4612653E38F);
    assert(p104_roll_GET(pack) == (float) -2.3077502E38F);
    assert(p104_usec_GET(pack) == (uint64_t)8132488405968501057L);
    assert(p104_pitch_GET(pack) == (float)2.399117E38F);
    assert(p104_z_GET(pack) == (float)2.5148332E38F);
    assert(p104_x_GET(pack) == (float)2.8211985E37F);
};


void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_time_usec_GET(pack) == (uint64_t)4818936541614807020L);
    assert(p105_xmag_GET(pack) == (float) -2.4842913E38F);
    assert(p105_pressure_alt_GET(pack) == (float)3.0766527E38F);
    assert(p105_zmag_GET(pack) == (float) -2.3750957E38F);
    assert(p105_temperature_GET(pack) == (float)8.12181E37F);
    assert(p105_ygyro_GET(pack) == (float)9.18551E37F);
    assert(p105_zacc_GET(pack) == (float)2.8924716E37F);
    assert(p105_yacc_GET(pack) == (float) -2.733123E38F);
    assert(p105_xgyro_GET(pack) == (float)2.2974354E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -2.8829368E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)10105);
    assert(p105_diff_pressure_GET(pack) == (float) -3.2143908E38F);
    assert(p105_xacc_GET(pack) == (float)2.9979271E38F);
    assert(p105_ymag_GET(pack) == (float)1.2587687E38F);
    assert(p105_zgyro_GET(pack) == (float) -5.499719E37F);
};


void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_xgyro_GET(pack) == (float) -5.966226E37F);
    assert(p106_distance_GET(pack) == (float)5.4539744E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)1309587125L);
    assert(p106_integrated_y_GET(pack) == (float)1.9702888E38F);
    assert(p106_time_usec_GET(pack) == (uint64_t)2794753736358470316L);
    assert(p106_integrated_x_GET(pack) == (float) -4.462486E37F);
    assert(p106_integrated_ygyro_GET(pack) == (float)3.2819079E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float)1.8037364E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2430743697L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -7592);
};


void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float) -2.016031E38F);
    assert(p107_zgyro_GET(pack) == (float) -1.0848417E38F);
    assert(p107_diff_pressure_GET(pack) == (float)1.8186287E38F);
    assert(p107_abs_pressure_GET(pack) == (float)3.0776287E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1466803618L);
    assert(p107_ygyro_GET(pack) == (float)6.889505E37F);
    assert(p107_xgyro_GET(pack) == (float)1.9320502E38F);
    assert(p107_xmag_GET(pack) == (float) -1.9260785E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)3838243168774387163L);
    assert(p107_xacc_GET(pack) == (float)2.4662392E38F);
    assert(p107_yacc_GET(pack) == (float) -2.684725E36F);
    assert(p107_pressure_alt_GET(pack) == (float) -2.9582725E36F);
    assert(p107_zacc_GET(pack) == (float)2.3466442E38F);
    assert(p107_temperature_GET(pack) == (float) -2.7282173E38F);
    assert(p107_ymag_GET(pack) == (float) -2.4542693E38F);
};


void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_zacc_GET(pack) == (float)1.3480861E38F);
    assert(p108_pitch_GET(pack) == (float) -1.563403E38F);
    assert(p108_q1_GET(pack) == (float)4.7498194E37F);
    assert(p108_lon_GET(pack) == (float) -5.3640564E37F);
    assert(p108_q3_GET(pack) == (float)2.8175362E38F);
    assert(p108_vd_GET(pack) == (float)1.502585E38F);
    assert(p108_q2_GET(pack) == (float) -8.405625E37F);
    assert(p108_q4_GET(pack) == (float)5.429458E37F);
    assert(p108_ygyro_GET(pack) == (float)3.0464193E38F);
    assert(p108_ve_GET(pack) == (float) -1.1028674E37F);
    assert(p108_yacc_GET(pack) == (float) -1.378438E38F);
    assert(p108_xgyro_GET(pack) == (float) -1.453832E38F);
    assert(p108_lat_GET(pack) == (float) -3.262706E38F);
    assert(p108_xacc_GET(pack) == (float) -2.205983E38F);
    assert(p108_vn_GET(pack) == (float) -2.2065006E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)2.6489195E36F);
    assert(p108_zgyro_GET(pack) == (float)9.3399457E36F);
    assert(p108_alt_GET(pack) == (float) -4.3811114E36F);
    assert(p108_roll_GET(pack) == (float)2.5953454E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -4.1091658E36F);
    assert(p108_yaw_GET(pack) == (float)2.0865239E38F);
};


void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)203);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)59272);
};


void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)73);
    {
        uint8_t exemplary[] =  {(uint8_t)17, (uint8_t)3, (uint8_t)205, (uint8_t)118, (uint8_t)161, (uint8_t)39, (uint8_t)154, (uint8_t)68, (uint8_t)96, (uint8_t)142, (uint8_t)97, (uint8_t)213, (uint8_t)197, (uint8_t)40, (uint8_t)143, (uint8_t)55, (uint8_t)70, (uint8_t)233, (uint8_t)106, (uint8_t)127, (uint8_t)110, (uint8_t)40, (uint8_t)175, (uint8_t)62, (uint8_t)57, (uint8_t)46, (uint8_t)254, (uint8_t)3, (uint8_t)200, (uint8_t)80, (uint8_t)207, (uint8_t)135, (uint8_t)69, (uint8_t)125, (uint8_t)93, (uint8_t)161, (uint8_t)115, (uint8_t)100, (uint8_t)33, (uint8_t)131, (uint8_t)183, (uint8_t)152, (uint8_t)145, (uint8_t)128, (uint8_t)178, (uint8_t)111, (uint8_t)17, (uint8_t)7, (uint8_t)51, (uint8_t)51, (uint8_t)74, (uint8_t)212, (uint8_t)217, (uint8_t)181, (uint8_t)6, (uint8_t)228, (uint8_t)174, (uint8_t)213, (uint8_t)188, (uint8_t)144, (uint8_t)11, (uint8_t)174, (uint8_t)110, (uint8_t)92, (uint8_t)59, (uint8_t)171, (uint8_t)182, (uint8_t)84, (uint8_t)17, (uint8_t)195, (uint8_t)245, (uint8_t)1, (uint8_t)164, (uint8_t)187, (uint8_t)247, (uint8_t)65, (uint8_t)29, (uint8_t)32, (uint8_t)120, (uint8_t)74, (uint8_t)173, (uint8_t)141, (uint8_t)206, (uint8_t)16, (uint8_t)44, (uint8_t)173, (uint8_t)134, (uint8_t)240, (uint8_t)35, (uint8_t)158, (uint8_t)122, (uint8_t)156, (uint8_t)7, (uint8_t)249, (uint8_t)165, (uint8_t)254, (uint8_t)66, (uint8_t)217, (uint8_t)227, (uint8_t)194, (uint8_t)53, (uint8_t)245, (uint8_t)221, (uint8_t)220, (uint8_t)88, (uint8_t)160, (uint8_t)229, (uint8_t)193, (uint8_t)101, (uint8_t)150, (uint8_t)224, (uint8_t)20, (uint8_t)96, (uint8_t)100, (uint8_t)198, (uint8_t)211, (uint8_t)72, (uint8_t)125, (uint8_t)183, (uint8_t)141, (uint8_t)174, (uint8_t)187, (uint8_t)19, (uint8_t)169, (uint8_t)130, (uint8_t)42, (uint8_t)101, (uint8_t)121, (uint8_t)124, (uint8_t)148, (uint8_t)35, (uint8_t)23, (uint8_t)98, (uint8_t)92, (uint8_t)212, (uint8_t)38, (uint8_t)127, (uint8_t)156, (uint8_t)160, (uint8_t)117, (uint8_t)218, (uint8_t)89, (uint8_t)138, (uint8_t)57, (uint8_t)50, (uint8_t)30, (uint8_t)123, (uint8_t)85, (uint8_t)57, (uint8_t)61, (uint8_t)81, (uint8_t)149, (uint8_t)114, (uint8_t)71, (uint8_t)206, (uint8_t)188, (uint8_t)245, (uint8_t)184, (uint8_t)35, (uint8_t)252, (uint8_t)105, (uint8_t)141, (uint8_t)233, (uint8_t)63, (uint8_t)32, (uint8_t)232, (uint8_t)30, (uint8_t)4, (uint8_t)243, (uint8_t)17, (uint8_t)159, (uint8_t)182, (uint8_t)51, (uint8_t)13, (uint8_t)234, (uint8_t)98, (uint8_t)128, (uint8_t)229, (uint8_t)75, (uint8_t)5, (uint8_t)166, (uint8_t)72, (uint8_t)138, (uint8_t)82, (uint8_t)213, (uint8_t)252, (uint8_t)96, (uint8_t)4, (uint8_t)45, (uint8_t)227, (uint8_t)237, (uint8_t)86, (uint8_t)40, (uint8_t)184, (uint8_t)167, (uint8_t)187, (uint8_t)123, (uint8_t)19, (uint8_t)151, (uint8_t)141, (uint8_t)55, (uint8_t)239, (uint8_t)113, (uint8_t)132, (uint8_t)247, (uint8_t)39, (uint8_t)130, (uint8_t)60, (uint8_t)238, (uint8_t)77, (uint8_t)161, (uint8_t)119, (uint8_t)56, (uint8_t)93, (uint8_t)64, (uint8_t)29, (uint8_t)15, (uint8_t)158, (uint8_t)187, (uint8_t)71, (uint8_t)197, (uint8_t)130, (uint8_t)87, (uint8_t)112, (uint8_t)100, (uint8_t)48, (uint8_t)211, (uint8_t)24, (uint8_t)51, (uint8_t)252, (uint8_t)129, (uint8_t)150, (uint8_t)217, (uint8_t)71, (uint8_t)17, (uint8_t)36, (uint8_t)146, (uint8_t)24, (uint8_t)105, (uint8_t)40, (uint8_t)170, (uint8_t)138, (uint8_t)40, (uint8_t)103, (uint8_t)35, (uint8_t)214, (uint8_t)95, (uint8_t)174, (uint8_t)243, (uint8_t)198, (uint8_t)45} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)72);
};


void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -5667210633751528184L);
    assert(p111_tc1_GET(pack) == (int64_t)3568525415637484616L);
};


void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)3001557915414442564L);
    assert(p112_seq_GET(pack) == (uint32_t)1419833907L);
};


void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)64017);
    assert(p113_time_usec_GET(pack) == (uint64_t)1408405687273781154L);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)38668);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -14522);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)46246);
    assert(p113_lat_GET(pack) == (int32_t)68216505);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)56507);
    assert(p113_alt_GET(pack) == (int32_t) -1236455657);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)16626);
    assert(p113_lon_GET(pack) == (int32_t) -1874514058);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)17501);
};


void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_ygyro_GET(pack) == (float)1.951686E38F);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.9333603E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p114_time_usec_GET(pack) == (uint64_t)7473669562463883079L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p114_integrated_zgyro_GET(pack) == (float)2.8159436E37F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)11045);
    assert(p114_integrated_x_GET(pack) == (float) -1.9721577E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2377926790L);
    assert(p114_integrated_y_GET(pack) == (float) -2.4725803E38F);
    assert(p114_distance_GET(pack) == (float) -6.833924E37F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)436279911L);
};


void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)11392);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -4376);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)21289);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)34125);
    assert(p115_pitchspeed_GET(pack) == (float)2.3216714E38F);
    assert(p115_alt_GET(pack) == (int32_t)1946422435);
    assert(p115_lon_GET(pack) == (int32_t)1622026339);
    assert(p115_lat_GET(pack) == (int32_t)882053765);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -2654);
    assert(p115_yawspeed_GET(pack) == (float)2.8908267E38F);
    assert(p115_time_usec_GET(pack) == (uint64_t)6053786514724450452L);
    {
        float exemplary[] =  {1.7051483E38F, 7.762479E36F, -2.3824203E38F, 4.499651E37F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -26726);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -21310);
    assert(p115_rollspeed_GET(pack) == (float)2.220006E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)44064);
};


void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)6334);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)14229);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -16363);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t)7646);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -16328);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -24915);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -27175);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)31289);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)952925679L);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)11575);
};


void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)62274);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)8323);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)49);
};


void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)4252422775L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)46645);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)56143);
    assert(p118_time_utc_GET(pack) == (uint32_t)3232270520L);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)35469);
};


void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_ofs_GET(pack) == (uint32_t)2612820405L);
    assert(p119_count_GET(pack) == (uint32_t)2245987504L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)18610);
};


void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)18025);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)182);
    {
        uint8_t exemplary[] =  {(uint8_t)236, (uint8_t)38, (uint8_t)249, (uint8_t)125, (uint8_t)163, (uint8_t)108, (uint8_t)168, (uint8_t)121, (uint8_t)23, (uint8_t)139, (uint8_t)62, (uint8_t)189, (uint8_t)62, (uint8_t)148, (uint8_t)10, (uint8_t)243, (uint8_t)20, (uint8_t)120, (uint8_t)199, (uint8_t)8, (uint8_t)241, (uint8_t)164, (uint8_t)229, (uint8_t)159, (uint8_t)246, (uint8_t)129, (uint8_t)128, (uint8_t)134, (uint8_t)134, (uint8_t)78, (uint8_t)168, (uint8_t)218, (uint8_t)177, (uint8_t)52, (uint8_t)248, (uint8_t)234, (uint8_t)201, (uint8_t)113, (uint8_t)59, (uint8_t)195, (uint8_t)22, (uint8_t)218, (uint8_t)21, (uint8_t)66, (uint8_t)126, (uint8_t)205, (uint8_t)106, (uint8_t)11, (uint8_t)151, (uint8_t)114, (uint8_t)121, (uint8_t)179, (uint8_t)49, (uint8_t)101, (uint8_t)46, (uint8_t)152, (uint8_t)17, (uint8_t)203, (uint8_t)223, (uint8_t)149, (uint8_t)21, (uint8_t)252, (uint8_t)136, (uint8_t)85, (uint8_t)46, (uint8_t)109, (uint8_t)245, (uint8_t)248, (uint8_t)197, (uint8_t)48, (uint8_t)105, (uint8_t)91, (uint8_t)82, (uint8_t)117, (uint8_t)53, (uint8_t)22, (uint8_t)86, (uint8_t)77, (uint8_t)74, (uint8_t)166, (uint8_t)195, (uint8_t)119, (uint8_t)108, (uint8_t)203, (uint8_t)36, (uint8_t)122, (uint8_t)255, (uint8_t)48, (uint8_t)10, (uint8_t)178} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)1263689835L);
};


void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)13);
    {
        uint8_t exemplary[] =  {(uint8_t)218, (uint8_t)75, (uint8_t)219, (uint8_t)180, (uint8_t)35, (uint8_t)7, (uint8_t)150, (uint8_t)119, (uint8_t)9, (uint8_t)214, (uint8_t)211, (uint8_t)57, (uint8_t)224, (uint8_t)199, (uint8_t)232, (uint8_t)202, (uint8_t)209, (uint8_t)155, (uint8_t)26, (uint8_t)45, (uint8_t)136, (uint8_t)77, (uint8_t)23, (uint8_t)212, (uint8_t)217, (uint8_t)21, (uint8_t)204, (uint8_t)235, (uint8_t)66, (uint8_t)199, (uint8_t)155, (uint8_t)168, (uint8_t)228, (uint8_t)31, (uint8_t)173, (uint8_t)63, (uint8_t)246, (uint8_t)113, (uint8_t)187, (uint8_t)252, (uint8_t)245, (uint8_t)128, (uint8_t)82, (uint8_t)129, (uint8_t)238, (uint8_t)236, (uint8_t)50, (uint8_t)244, (uint8_t)44, (uint8_t)138, (uint8_t)116, (uint8_t)246, (uint8_t)170, (uint8_t)195, (uint8_t)6, (uint8_t)107, (uint8_t)171, (uint8_t)120, (uint8_t)81, (uint8_t)244, (uint8_t)202, (uint8_t)112, (uint8_t)183, (uint8_t)13, (uint8_t)142, (uint8_t)191, (uint8_t)46, (uint8_t)153, (uint8_t)163, (uint8_t)51, (uint8_t)205, (uint8_t)35, (uint8_t)83, (uint8_t)5, (uint8_t)5, (uint8_t)101, (uint8_t)92, (uint8_t)246, (uint8_t)9, (uint8_t)152, (uint8_t)200, (uint8_t)59, (uint8_t)247, (uint8_t)172, (uint8_t)68, (uint8_t)115, (uint8_t)24, (uint8_t)243, (uint8_t)81, (uint8_t)243, (uint8_t)129, (uint8_t)83, (uint8_t)221, (uint8_t)155, (uint8_t)43, (uint8_t)173, (uint8_t)97, (uint8_t)143, (uint8_t)230, (uint8_t)32, (uint8_t)6, (uint8_t)29, (uint8_t)47, (uint8_t)79, (uint8_t)154, (uint8_t)125, (uint8_t)5, (uint8_t)61, (uint8_t)44, (uint8_t)213} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)41539);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)14342);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)41047);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)2495);
    assert(p124_time_usec_GET(pack) == (uint64_t)4814211722413217027L);
    assert(p124_alt_GET(pack) == (int32_t) -1746422963);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1604588518L);
    assert(p124_lat_GET(pack) == (int32_t)2023476196);
    assert(p124_lon_GET(pack) == (int32_t)1728716117);
};


void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)36011);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)53108);
};


void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_baudrate_GET(pack) == (uint32_t)1338472155L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)33841);
    {
        uint8_t exemplary[] =  {(uint8_t)130, (uint8_t)214, (uint8_t)185, (uint8_t)78, (uint8_t)146, (uint8_t)17, (uint8_t)214, (uint8_t)48, (uint8_t)185, (uint8_t)127, (uint8_t)179, (uint8_t)38, (uint8_t)7, (uint8_t)48, (uint8_t)64, (uint8_t)218, (uint8_t)123, (uint8_t)27, (uint8_t)36, (uint8_t)220, (uint8_t)36, (uint8_t)79, (uint8_t)92, (uint8_t)5, (uint8_t)203, (uint8_t)60, (uint8_t)223, (uint8_t)35, (uint8_t)180, (uint8_t)109, (uint8_t)90, (uint8_t)89, (uint8_t)150, (uint8_t)34, (uint8_t)82, (uint8_t)172, (uint8_t)108, (uint8_t)113, (uint8_t)98, (uint8_t)128, (uint8_t)74, (uint8_t)238, (uint8_t)197, (uint8_t)114, (uint8_t)82, (uint8_t)12, (uint8_t)73, (uint8_t)104, (uint8_t)52, (uint8_t)94, (uint8_t)176, (uint8_t)206, (uint8_t)70, (uint8_t)191, (uint8_t)140, (uint8_t)169, (uint8_t)60, (uint8_t)97, (uint8_t)149, (uint8_t)34, (uint8_t)44, (uint8_t)136, (uint8_t)30, (uint8_t)46, (uint8_t)169, (uint8_t)194, (uint8_t)191, (uint8_t)90, (uint8_t)38, (uint8_t)24} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)77);
};


void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1038492860);
    assert(p127_tow_GET(pack) == (uint32_t)943052820L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)31749);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)285805169);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1136902089);
    assert(p127_accuracy_GET(pack) == (uint32_t)1978968367L);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1753346544);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1941885079L);
};


void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)11566174);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1081456821);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p128_tow_GET(pack) == (uint32_t)385377152L);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)1050784417L);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)52533);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)509406860);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)1979860776);
    assert(p128_accuracy_GET(pack) == (uint32_t)136909085L);
};


void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)32415);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -5832);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)4127783241L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t) -28441);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -26184);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)20858);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -19338);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -9208);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t) -17723);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -30588);
};


void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)38797);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)21412);
    assert(p130_size_GET(pack) == (uint32_t)2459682499L);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)62686);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)44800);
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)62, (uint8_t)18, (uint8_t)174, (uint8_t)25, (uint8_t)140, (uint8_t)133, (uint8_t)180, (uint8_t)52, (uint8_t)166, (uint8_t)237, (uint8_t)180, (uint8_t)78, (uint8_t)113, (uint8_t)146, (uint8_t)198, (uint8_t)12, (uint8_t)168, (uint8_t)229, (uint8_t)174, (uint8_t)124, (uint8_t)151, (uint8_t)243, (uint8_t)11, (uint8_t)249, (uint8_t)29, (uint8_t)88, (uint8_t)5, (uint8_t)254, (uint8_t)18, (uint8_t)175, (uint8_t)21, (uint8_t)195, (uint8_t)180, (uint8_t)103, (uint8_t)140, (uint8_t)230, (uint8_t)217, (uint8_t)126, (uint8_t)151, (uint8_t)204, (uint8_t)93, (uint8_t)27, (uint8_t)35, (uint8_t)89, (uint8_t)7, (uint8_t)139, (uint8_t)173, (uint8_t)138, (uint8_t)231, (uint8_t)125, (uint8_t)176, (uint8_t)70, (uint8_t)113, (uint8_t)173, (uint8_t)171, (uint8_t)232, (uint8_t)33, (uint8_t)95, (uint8_t)152, (uint8_t)122, (uint8_t)101, (uint8_t)11, (uint8_t)126, (uint8_t)48, (uint8_t)119, (uint8_t)207, (uint8_t)145, (uint8_t)123, (uint8_t)30, (uint8_t)212, (uint8_t)131, (uint8_t)185, (uint8_t)254, (uint8_t)59, (uint8_t)106, (uint8_t)185, (uint8_t)93, (uint8_t)92, (uint8_t)138, (uint8_t)21, (uint8_t)220, (uint8_t)5, (uint8_t)252, (uint8_t)87, (uint8_t)28, (uint8_t)194, (uint8_t)80, (uint8_t)249, (uint8_t)242, (uint8_t)24, (uint8_t)20, (uint8_t)132, (uint8_t)148, (uint8_t)216, (uint8_t)11, (uint8_t)72, (uint8_t)199, (uint8_t)109, (uint8_t)234, (uint8_t)122, (uint8_t)167, (uint8_t)162, (uint8_t)152, (uint8_t)227, (uint8_t)151, (uint8_t)95, (uint8_t)121, (uint8_t)167, (uint8_t)121, (uint8_t)105, (uint8_t)133, (uint8_t)22, (uint8_t)219, (uint8_t)156, (uint8_t)249, (uint8_t)254, (uint8_t)140, (uint8_t)248, (uint8_t)194, (uint8_t)95, (uint8_t)53, (uint8_t)115, (uint8_t)249, (uint8_t)150, (uint8_t)128, (uint8_t)99, (uint8_t)168, (uint8_t)85, (uint8_t)190, (uint8_t)168, (uint8_t)118, (uint8_t)21, (uint8_t)104, (uint8_t)164, (uint8_t)93, (uint8_t)79, (uint8_t)163, (uint8_t)188, (uint8_t)100, (uint8_t)56, (uint8_t)87, (uint8_t)13, (uint8_t)151, (uint8_t)114, (uint8_t)101, (uint8_t)182, (uint8_t)26, (uint8_t)17, (uint8_t)227, (uint8_t)122, (uint8_t)67, (uint8_t)8, (uint8_t)51, (uint8_t)83, (uint8_t)225, (uint8_t)213, (uint8_t)66, (uint8_t)144, (uint8_t)87, (uint8_t)192, (uint8_t)21, (uint8_t)234, (uint8_t)252, (uint8_t)62, (uint8_t)46, (uint8_t)248, (uint8_t)188, (uint8_t)216, (uint8_t)70, (uint8_t)241, (uint8_t)121, (uint8_t)161, (uint8_t)43, (uint8_t)91, (uint8_t)50, (uint8_t)45, (uint8_t)220, (uint8_t)100, (uint8_t)63, (uint8_t)252, (uint8_t)183, (uint8_t)70, (uint8_t)122, (uint8_t)222, (uint8_t)98, (uint8_t)105, (uint8_t)103, (uint8_t)251, (uint8_t)168, (uint8_t)210, (uint8_t)229, (uint8_t)30, (uint8_t)201, (uint8_t)28, (uint8_t)19, (uint8_t)61, (uint8_t)71, (uint8_t)133, (uint8_t)149, (uint8_t)191, (uint8_t)141, (uint8_t)8, (uint8_t)106, (uint8_t)246, (uint8_t)221, (uint8_t)111, (uint8_t)100, (uint8_t)100, (uint8_t)247, (uint8_t)35, (uint8_t)116, (uint8_t)230, (uint8_t)62, (uint8_t)131, (uint8_t)18, (uint8_t)127, (uint8_t)206, (uint8_t)114, (uint8_t)220, (uint8_t)57, (uint8_t)102, (uint8_t)12, (uint8_t)242, (uint8_t)30, (uint8_t)132, (uint8_t)107, (uint8_t)24, (uint8_t)138, (uint8_t)139, (uint8_t)209, (uint8_t)224, (uint8_t)14, (uint8_t)43, (uint8_t)178, (uint8_t)241, (uint8_t)253, (uint8_t)187, (uint8_t)105, (uint8_t)35, (uint8_t)247, (uint8_t)77, (uint8_t)56, (uint8_t)0, (uint8_t)11, (uint8_t)48, (uint8_t)96, (uint8_t)107, (uint8_t)66, (uint8_t)23, (uint8_t)22, (uint8_t)180, (uint8_t)31} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_270);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)26920);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)26322);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)46663);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)2023773760L);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)296669069);
    assert(p133_lon_GET(pack) == (int32_t)1577526344);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)56038);
    assert(p133_mask_GET(pack) == (uint64_t)8106080634754672482L);
};


void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lat_GET(pack) == (int32_t)1522265006);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)37331);
    {
        int16_t exemplary[] =  {(int16_t)13560, (int16_t) -31927, (int16_t)15489, (int16_t) -2833, (int16_t)26221, (int16_t)22117, (int16_t) -16031, (int16_t)23544, (int16_t) -31335, (int16_t) -24443, (int16_t) -17365, (int16_t) -6413, (int16_t)1880, (int16_t) -25847, (int16_t) -25205, (int16_t)27272} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t) -982298845);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)16);
};


void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)599081529);
    assert(p135_lon_GET(pack) == (int32_t) -82111113);
};


void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float) -1.6061361E38F);
    assert(p136_lon_GET(pack) == (int32_t) -631927872);
    assert(p136_lat_GET(pack) == (int32_t)1777913876);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)40093);
    assert(p136_current_height_GET(pack) == (float) -3.3936992E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)7329);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)50265);
};


void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)3415711517L);
    assert(p137_press_abs_GET(pack) == (float) -3.1501886E38F);
    assert(p137_press_diff_GET(pack) == (float)6.458087E37F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)9865);
};


void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float)2.2214597E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)4451140005938754895L);
    {
        float exemplary[] =  {1.5490164E38F, -1.5164821E38F, -1.852958E38F, 9.522645E36F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float)1.7456341E36F);
    assert(p138_y_GET(pack) == (float) -3.283186E38F);
};


void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)28);
    {
        float exemplary[] =  {-3.2098457E38F, -1.4387232E37F, -1.7950539E38F, -2.3428621E38F, 1.894254E38F, -1.1545533E38F, 1.4783821E38F, 8.516883E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p139_time_usec_GET(pack) == (uint64_t)4359677886207123806L);
};


void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p140_time_usec_GET(pack) == (uint64_t)7637525349737860074L);
    {
        float exemplary[] =  {-1.5229614E38F, -3.3273897E38F, 1.4001146E38F, -8.527506E37F, -3.0628039E38F, 1.4643756E38F, 2.1201748E38F, 2.768402E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_relative_GET(pack) == (float)1.5514764E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)1.7449557E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -3.2138814E37F);
    assert(p141_altitude_local_GET(pack) == (float) -5.644832E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float)9.385611E37F);
    assert(p141_bottom_clearance_GET(pack) == (float)2.1620055E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)7864052235922642545L);
};


void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)48);
    {
        uint8_t exemplary[] =  {(uint8_t)236, (uint8_t)176, (uint8_t)228, (uint8_t)242, (uint8_t)161, (uint8_t)221, (uint8_t)67, (uint8_t)53, (uint8_t)223, (uint8_t)128, (uint8_t)150, (uint8_t)45, (uint8_t)252, (uint8_t)221, (uint8_t)18, (uint8_t)138, (uint8_t)1, (uint8_t)219, (uint8_t)37, (uint8_t)87, (uint8_t)201, (uint8_t)184, (uint8_t)29, (uint8_t)145, (uint8_t)28, (uint8_t)22, (uint8_t)41, (uint8_t)72, (uint8_t)208, (uint8_t)6, (uint8_t)192, (uint8_t)52, (uint8_t)170, (uint8_t)6, (uint8_t)247, (uint8_t)39, (uint8_t)168, (uint8_t)209, (uint8_t)2, (uint8_t)220, (uint8_t)155, (uint8_t)216, (uint8_t)195, (uint8_t)9, (uint8_t)79, (uint8_t)65, (uint8_t)205, (uint8_t)161, (uint8_t)9, (uint8_t)11, (uint8_t)189, (uint8_t)160, (uint8_t)80, (uint8_t)149, (uint8_t)92, (uint8_t)201, (uint8_t)25, (uint8_t)1, (uint8_t)204, (uint8_t)234, (uint8_t)31, (uint8_t)172, (uint8_t)94, (uint8_t)129, (uint8_t)116, (uint8_t)114, (uint8_t)57, (uint8_t)167, (uint8_t)152, (uint8_t)24, (uint8_t)73, (uint8_t)166, (uint8_t)242, (uint8_t)98, (uint8_t)23, (uint8_t)161, (uint8_t)39, (uint8_t)76, (uint8_t)234, (uint8_t)237, (uint8_t)69, (uint8_t)54, (uint8_t)114, (uint8_t)189, (uint8_t)154, (uint8_t)48, (uint8_t)91, (uint8_t)91, (uint8_t)110, (uint8_t)6, (uint8_t)192, (uint8_t)30, (uint8_t)97, (uint8_t)227, (uint8_t)140, (uint8_t)185, (uint8_t)164, (uint8_t)72, (uint8_t)89, (uint8_t)241, (uint8_t)12, (uint8_t)19, (uint8_t)61, (uint8_t)239, (uint8_t)149, (uint8_t)80, (uint8_t)199, (uint8_t)173, (uint8_t)223, (uint8_t)191, (uint8_t)28, (uint8_t)150, (uint8_t)44, (uint8_t)50, (uint8_t)197, (uint8_t)56, (uint8_t)156, (uint8_t)197, (uint8_t)73, (uint8_t)208} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)43, (uint8_t)26, (uint8_t)165, (uint8_t)52, (uint8_t)91, (uint8_t)92, (uint8_t)168, (uint8_t)8, (uint8_t)162, (uint8_t)194, (uint8_t)123, (uint8_t)169, (uint8_t)68, (uint8_t)151, (uint8_t)128, (uint8_t)170, (uint8_t)145, (uint8_t)129, (uint8_t)194, (uint8_t)92, (uint8_t)195, (uint8_t)45, (uint8_t)39, (uint8_t)44, (uint8_t)236, (uint8_t)20, (uint8_t)235, (uint8_t)14, (uint8_t)163, (uint8_t)68, (uint8_t)229, (uint8_t)243, (uint8_t)97, (uint8_t)110, (uint8_t)63, (uint8_t)189, (uint8_t)19, (uint8_t)26, (uint8_t)51, (uint8_t)214, (uint8_t)253, (uint8_t)34, (uint8_t)252, (uint8_t)98, (uint8_t)168, (uint8_t)173, (uint8_t)7, (uint8_t)128, (uint8_t)112, (uint8_t)225, (uint8_t)31, (uint8_t)192, (uint8_t)81, (uint8_t)175, (uint8_t)108, (uint8_t)238, (uint8_t)57, (uint8_t)62, (uint8_t)152, (uint8_t)83, (uint8_t)50, (uint8_t)184, (uint8_t)226, (uint8_t)235, (uint8_t)138, (uint8_t)130, (uint8_t)214, (uint8_t)0, (uint8_t)62, (uint8_t)108, (uint8_t)33, (uint8_t)247, (uint8_t)242, (uint8_t)54, (uint8_t)140, (uint8_t)155, (uint8_t)160, (uint8_t)114, (uint8_t)44, (uint8_t)171, (uint8_t)217, (uint8_t)48, (uint8_t)249, (uint8_t)235, (uint8_t)87, (uint8_t)212, (uint8_t)206, (uint8_t)192, (uint8_t)122, (uint8_t)133, (uint8_t)254, (uint8_t)31, (uint8_t)37, (uint8_t)187, (uint8_t)253, (uint8_t)167, (uint8_t)133, (uint8_t)134, (uint8_t)103, (uint8_t)120, (uint8_t)124, (uint8_t)207, (uint8_t)188, (uint8_t)105, (uint8_t)181, (uint8_t)13, (uint8_t)223, (uint8_t)219, (uint8_t)82, (uint8_t)78, (uint8_t)130, (uint8_t)151, (uint8_t)56, (uint8_t)90, (uint8_t)167, (uint8_t)149, (uint8_t)36, (uint8_t)62, (uint8_t)29, (uint8_t)155} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)146);
};


void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)4.9160266E37F);
    assert(p143_press_diff_GET(pack) == (float) -3.5158677E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -22174);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2359553157L);
};


void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_alt_GET(pack) == (float)2.1144631E38F);
    assert(p144_custom_state_GET(pack) == (uint64_t)4209062715672247912L);
    {
        float exemplary[] =  {-2.6875495E38F, -3.281374E38F, 7.872014E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.5671176E37F, -3.018196E37F, -6.0053507E37F, -2.6931308E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1858418652);
    {
        float exemplary[] =  {1.1018877E37F, 6.367287E37F, -2.6419368E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.7202179E38F, 2.0193544E38F, -3.0448513E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)8956847255645276457L);
    assert(p144_lat_GET(pack) == (int32_t)1596419731);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)218);
    {
        float exemplary[] =  {8.951343E37F, 3.117916E38F, -1.8079976E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {1.1046906E38F, 3.1029176E38F, -2.108547E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float) -1.4965751E38F);
    assert(p146_y_pos_GET(pack) == (float) -9.717474E37F);
    assert(p146_x_pos_GET(pack) == (float) -4.472403E37F);
    assert(p146_yaw_rate_GET(pack) == (float)7.707957E37F);
    assert(p146_x_acc_GET(pack) == (float) -3.1939538E38F);
    assert(p146_z_vel_GET(pack) == (float) -3.0920475E38F);
    assert(p146_y_vel_GET(pack) == (float)2.116511E38F);
    assert(p146_pitch_rate_GET(pack) == (float) -2.618427E38F);
    assert(p146_z_acc_GET(pack) == (float)1.3430503E37F);
    {
        float exemplary[] =  {-3.3591755E38F, -2.5037318E38F, 2.1179478E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_time_usec_GET(pack) == (uint64_t)2918029663234418531L);
    {
        float exemplary[] =  {3.0479594E38F, 1.3256498E38F, -3.0771192E38F, -2.542414E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_pos_GET(pack) == (float) -2.3547214E38F);
    assert(p146_airspeed_GET(pack) == (float)2.8122687E38F);
    assert(p146_x_vel_GET(pack) == (float) -2.6246418E38F);
    assert(p146_roll_rate_GET(pack) == (float)1.3036767E38F);
};


void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_consumed_GET(pack) == (int32_t)347988419);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -8141);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -21742);
    {
        uint16_t exemplary[] =  {(uint16_t)58934, (uint16_t)124, (uint16_t)62917, (uint16_t)24663, (uint16_t)54367, (uint16_t)35736, (uint16_t)55552, (uint16_t)15135, (uint16_t)51385, (uint16_t)51733} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -23);
    assert(p147_energy_consumed_GET(pack) == (int32_t)1309377689);
};


void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)44830);
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)50, (uint8_t)30, (uint8_t)48, (uint8_t)86, (uint8_t)72, (uint8_t)251, (uint8_t)204} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3598887045L);
    {
        uint8_t exemplary[] =  {(uint8_t)100, (uint8_t)192, (uint8_t)136, (uint8_t)83, (uint8_t)106, (uint8_t)39, (uint8_t)192, (uint8_t)135, (uint8_t)46, (uint8_t)107, (uint8_t)162, (uint8_t)231, (uint8_t)189, (uint8_t)150, (uint8_t)190, (uint8_t)48, (uint8_t)121, (uint8_t)181} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)2723965636L);
    {
        uint8_t exemplary[] =  {(uint8_t)119, (uint8_t)116, (uint8_t)251, (uint8_t)4, (uint8_t)222, (uint8_t)84, (uint8_t)166, (uint8_t)34} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)235, (uint8_t)86, (uint8_t)165, (uint8_t)217, (uint8_t)145, (uint8_t)19, (uint8_t)111, (uint8_t)211} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)30218);
    assert(p148_uid_GET(pack) == (uint64_t)9151205780219602572L);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1074952307L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3309572193L);
};


void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.4998983E38F, 1.1361951E38F, -8.745379E36F, 2.7987099E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_size_x_GET(pack) == (float)1.0114478E38F);
    assert(p149_x_TRY(ph) == (float)2.434688E37F);
    assert(p149_z_TRY(ph) == (float) -7.4776944E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p149_angle_y_GET(pack) == (float) -2.2380221E38F);
    assert(p149_angle_x_GET(pack) == (float) -1.5743556E38F);
    assert(p149_time_usec_GET(pack) == (uint64_t)3644853406791709879L);
    assert(p149_y_TRY(ph) == (float) -2.5083864E38F);
    assert(p149_distance_GET(pack) == (float)1.3688232E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p149_size_y_GET(pack) == (float)2.1862681E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)145);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
};


void c_CommunicationChannel_on_SENS_POWER_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_adc121_cspb_amp_GET(pack) == (float) -3.158572E38F);
    assert(p201_adc121_cs2_amp_GET(pack) == (float)7.8663394E37F);
    assert(p201_adc121_vspb_volt_GET(pack) == (float) -1.0743103E38F);
    assert(p201_adc121_cs1_amp_GET(pack) == (float)8.763983E37F);
};


void c_CommunicationChannel_on_SENS_MPPT_202(Bounds_Inside * ph, Pack * pack)
{
    assert(p202_mppt2_status_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p202_mppt2_pwm_GET(pack) == (uint16_t)(uint16_t)38458);
    assert(p202_mppt3_status_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p202_mppt2_volt_GET(pack) == (float) -2.4080094E38F);
    assert(p202_mppt1_pwm_GET(pack) == (uint16_t)(uint16_t)23457);
    assert(p202_mppt1_volt_GET(pack) == (float)3.2195344E38F);
    assert(p202_mppt1_amp_GET(pack) == (float) -2.967833E38F);
    assert(p202_mppt_timestamp_GET(pack) == (uint64_t)1589611839310857974L);
    assert(p202_mppt2_amp_GET(pack) == (float)9.45335E37F);
    assert(p202_mppt3_amp_GET(pack) == (float) -1.5907607E38F);
    assert(p202_mppt1_status_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p202_mppt3_volt_GET(pack) == (float) -1.144486E38F);
    assert(p202_mppt3_pwm_GET(pack) == (uint16_t)(uint16_t)61673);
};


void c_CommunicationChannel_on_ASLCTRL_DATA_203(Bounds_Inside * ph, Pack * pack)
{
    assert(p203_nZ_GET(pack) == (float) -3.367486E38F);
    assert(p203_RollAngleRef_GET(pack) == (float)2.3801199E38F);
    assert(p203_RollAngle_GET(pack) == (float)6.726057E36F);
    assert(p203_AirspeedRef_GET(pack) == (float)2.2915147E38F);
    assert(p203_SpoilersEngaged_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p203_uRud_GET(pack) == (float) -2.0790656E38F);
    assert(p203_hRef_t_GET(pack) == (float)2.9623236E38F);
    assert(p203_r_GET(pack) == (float) -8.846739E37F);
    assert(p203_hRef_GET(pack) == (float) -2.430506E38F);
    assert(p203_YawAngleRef_GET(pack) == (float)1.4427437E38F);
    assert(p203_rRef_GET(pack) == (float)1.1557756E38F);
    assert(p203_pRef_GET(pack) == (float) -3.0680389E35F);
    assert(p203_PitchAngleRef_GET(pack) == (float) -3.0928588E37F);
    assert(p203_q_GET(pack) == (float)2.969788E38F);
    assert(p203_uElev_GET(pack) == (float)2.768959E38F);
    assert(p203_p_GET(pack) == (float) -2.686208E38F);
    assert(p203_h_GET(pack) == (float)2.3624464E38F);
    assert(p203_qRef_GET(pack) == (float)2.0289322E38F);
    assert(p203_uThrot_GET(pack) == (float) -1.3882778E37F);
    assert(p203_PitchAngle_GET(pack) == (float)2.7899628E37F);
    assert(p203_aslctrl_mode_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p203_timestamp_GET(pack) == (uint64_t)5166557155616542377L);
    assert(p203_uThrot2_GET(pack) == (float) -2.070046E38F);
    assert(p203_uAil_GET(pack) == (float)1.7985086E38F);
    assert(p203_YawAngle_GET(pack) == (float) -7.6713194E37F);
};


void c_CommunicationChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * ph, Pack * pack)
{
    assert(p204_f_3_GET(pack) == (float) -1.0303898E38F);
    assert(p204_i32_1_GET(pack) == (uint32_t)2703423410L);
    assert(p204_f_2_GET(pack) == (float)6.340419E37F);
    assert(p204_i8_2_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p204_f_7_GET(pack) == (float) -3.9735043E37F);
    assert(p204_f_1_GET(pack) == (float) -1.4729333E38F);
    assert(p204_f_8_GET(pack) == (float)2.188823E38F);
    assert(p204_f_5_GET(pack) == (float)3.1010752E38F);
    assert(p204_i8_1_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p204_f_6_GET(pack) == (float)3.371732E38F);
    assert(p204_f_4_GET(pack) == (float)1.948133E38F);
};


void c_CommunicationChannel_on_ASLUAV_STATUS_205(Bounds_Inside * ph, Pack * pack)
{
    assert(p205_SATCOM_status_GET(pack) == (uint8_t)(uint8_t)50);
    {
        uint8_t exemplary[] =  {(uint8_t)145, (uint8_t)8, (uint8_t)77, (uint8_t)235, (uint8_t)231, (uint8_t)6, (uint8_t)228, (uint8_t)125} ;
        uint8_t*  sample = p205_Servo_status_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p205_LED_status_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p205_Motor_rpm_GET(pack) == (float) -1.9063204E38F);
};


void c_CommunicationChannel_on_EKF_EXT_206(Bounds_Inside * ph, Pack * pack)
{
    assert(p206_WindZ_GET(pack) == (float)1.2600393E38F);
    assert(p206_beta_GET(pack) == (float) -2.21007E38F);
    assert(p206_Windspeed_GET(pack) == (float) -1.5652006E38F);
    assert(p206_alpha_GET(pack) == (float) -1.8216705E38F);
    assert(p206_Airspeed_GET(pack) == (float) -1.3611187E38F);
    assert(p206_WindDir_GET(pack) == (float)5.692203E37F);
    assert(p206_timestamp_GET(pack) == (uint64_t)7561150924485705285L);
};


void c_CommunicationChannel_on_ASL_OBCTRL_207(Bounds_Inside * ph, Pack * pack)
{
    assert(p207_uElev_GET(pack) == (float) -1.9815245E38F);
    assert(p207_uAilL_GET(pack) == (float) -2.1196035E38F);
    assert(p207_uThrot2_GET(pack) == (float)7.2236537E37F);
    assert(p207_obctrl_status_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p207_uRud_GET(pack) == (float)1.999115E38F);
    assert(p207_uAilR_GET(pack) == (float) -4.306409E37F);
    assert(p207_timestamp_GET(pack) == (uint64_t)5348039869466635868L);
    assert(p207_uThrot_GET(pack) == (float)1.7286371E38F);
};


void c_CommunicationChannel_on_SENS_ATMOS_208(Bounds_Inside * ph, Pack * pack)
{
    assert(p208_TempAmbient_GET(pack) == (float)6.8565877E37F);
    assert(p208_Humidity_GET(pack) == (float)7.8395625E37F);
};


void c_CommunicationChannel_on_SENS_BATMON_209(Bounds_Inside * ph, Pack * pack)
{
    assert(p209_voltage_GET(pack) == (uint16_t)(uint16_t)9601);
    assert(p209_cellvoltage1_GET(pack) == (uint16_t)(uint16_t)10338);
    assert(p209_cellvoltage6_GET(pack) == (uint16_t)(uint16_t)17682);
    assert(p209_cellvoltage4_GET(pack) == (uint16_t)(uint16_t)2231);
    assert(p209_cellvoltage5_GET(pack) == (uint16_t)(uint16_t)53514);
    assert(p209_cellvoltage2_GET(pack) == (uint16_t)(uint16_t)20507);
    assert(p209_serialnumber_GET(pack) == (uint16_t)(uint16_t)38541);
    assert(p209_hostfetcontrol_GET(pack) == (uint16_t)(uint16_t)29623);
    assert(p209_batterystatus_GET(pack) == (uint16_t)(uint16_t)15713);
    assert(p209_current_GET(pack) == (int16_t)(int16_t)4271);
    assert(p209_SoC_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p209_cellvoltage3_GET(pack) == (uint16_t)(uint16_t)2145);
    assert(p209_temperature_GET(pack) == (float) -9.828813E37F);
};


void c_CommunicationChannel_on_FW_SOARING_DATA_210(Bounds_Inside * ph, Pack * pack)
{
    assert(p210_DebugVar2_GET(pack) == (float)9.136628E37F);
    assert(p210_LoiterDirection_GET(pack) == (float) -1.6534109E38F);
    assert(p210_vSinkExp_GET(pack) == (float)1.6874205E38F);
    assert(p210_xR_GET(pack) == (float) -2.5286666E38F);
    assert(p210_xLon_GET(pack) == (float)1.797705E38F);
    assert(p210_TSE_dot_GET(pack) == (float) -1.9331968E38F);
    assert(p210_ThermalGSEast_GET(pack) == (float)2.541719E38F);
    assert(p210_z1_exp_GET(pack) == (float) -1.5488301E38F);
    assert(p210_z2_DeltaRoll_GET(pack) == (float)1.026046E38F);
    assert(p210_VarW_GET(pack) == (float)1.7280131E37F);
    assert(p210_ControlMode_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p210_ThermalGSNorth_GET(pack) == (float)2.392028E38F);
    assert(p210_VarR_GET(pack) == (float)6.396188E37F);
    assert(p210_xLat_GET(pack) == (float)1.0341247E38F);
    assert(p210_timestampModeChanged_GET(pack) == (uint64_t)1828941308768555541L);
    assert(p210_DebugVar1_GET(pack) == (float) -2.51025E38F);
    assert(p210_DistToSoarPoint_GET(pack) == (float) -2.0443705E38F);
    assert(p210_z1_LocalUpdraftSpeed_GET(pack) == (float) -1.0587598E38F);
    assert(p210_z2_exp_GET(pack) == (float)1.9987678E38F);
    assert(p210_VarLon_GET(pack) == (float)5.2604833E37F);
    assert(p210_VarLat_GET(pack) == (float) -6.742835E37F);
    assert(p210_xW_GET(pack) == (float)4.1702973E36F);
    assert(p210_valid_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p210_timestamp_GET(pack) == (uint64_t)3820723982267108933L);
    assert(p210_LoiterRadius_GET(pack) == (float)6.6588936E37F);
};


void c_CommunicationChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * ph, Pack * pack)
{
    assert(p211_recording_nodes_count_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p211_cpu_temp_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p211_free_space_GET(pack) == (uint16_t)(uint16_t)20030);
    assert(p211_visensor_rate_1_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p211_visensor_rate_3_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p211_timestamp_GET(pack) == (uint64_t)8729322884687610528L);
    assert(p211_visensor_rate_4_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p211_visensor_rate_2_GET(pack) == (uint8_t)(uint8_t)67);
};


void c_CommunicationChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * ph, Pack * pack)
{
    assert(p212_pwr_brd_servo_2_amp_GET(pack) == (float)2.4520673E38F);
    assert(p212_timestamp_GET(pack) == (uint64_t)893739549085753316L);
    assert(p212_pwr_brd_mot_l_amp_GET(pack) == (float)2.7112337E38F);
    assert(p212_pwr_brd_servo_4_amp_GET(pack) == (float)2.6488594E38F);
    assert(p212_pwr_brd_servo_3_amp_GET(pack) == (float) -3.214206E37F);
    assert(p212_pwr_brd_aux_amp_GET(pack) == (float) -5.673672E37F);
    assert(p212_pwr_brd_status_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p212_pwr_brd_led_status_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p212_pwr_brd_system_volt_GET(pack) == (float)4.8474523E37F);
    assert(p212_pwr_brd_servo_volt_GET(pack) == (float)2.9115186E38F);
    assert(p212_pwr_brd_mot_r_amp_GET(pack) == (float)1.6861242E38F);
    assert(p212_pwr_brd_servo_1_amp_GET(pack) == (float)3.4008717E38F);
};


void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -6.722603E36F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.4845516E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -2.9719354E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.074521E37F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -8.47303E37F);
    assert(p230_mag_ratio_GET(pack) == (float)8.935133E37F);
    assert(p230_hagl_ratio_GET(pack) == (float)1.0402149E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)7120263297860276450L);
    assert(p230_tas_ratio_GET(pack) == (float)1.4045236E38F);
};


void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_horiz_GET(pack) == (float) -7.518901E37F);
    assert(p231_wind_z_GET(pack) == (float)2.4464235E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -3.0613247E38F);
    assert(p231_wind_alt_GET(pack) == (float)2.6603463E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -2.874331E36F);
    assert(p231_var_vert_GET(pack) == (float)3.3730613E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)2282892920318562949L);
    assert(p231_wind_x_GET(pack) == (float)2.0203284E38F);
    assert(p231_wind_y_GET(pack) == (float) -1.3888808E38F);
};


void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_vdop_GET(pack) == (float) -1.65425E38F);
    assert(p232_vn_GET(pack) == (float)1.0285761E38F);
    assert(p232_lon_GET(pack) == (int32_t) -766692634);
    assert(p232_lat_GET(pack) == (int32_t)139581998);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p232_speed_accuracy_GET(pack) == (float) -3.1446568E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)476664284L);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p232_time_usec_GET(pack) == (uint64_t)1455260027055112642L);
    assert(p232_ve_GET(pack) == (float) -2.7106713E37F);
    assert(p232_vert_accuracy_GET(pack) == (float)1.8346529E38F);
    assert(p232_hdop_GET(pack) == (float) -2.9749336E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP);
    assert(p232_alt_GET(pack) == (float)3.0945008E38F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)2934);
    assert(p232_horiz_accuracy_GET(pack) == (float) -7.500916E37F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p232_vd_GET(pack) == (float) -1.8301727E38F);
};


void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)190, (uint8_t)105, (uint8_t)67, (uint8_t)133, (uint8_t)64, (uint8_t)20, (uint8_t)145, (uint8_t)234, (uint8_t)35, (uint8_t)141, (uint8_t)238, (uint8_t)116, (uint8_t)0, (uint8_t)171, (uint8_t)194, (uint8_t)19, (uint8_t)133, (uint8_t)17, (uint8_t)88, (uint8_t)152, (uint8_t)100, (uint8_t)99, (uint8_t)27, (uint8_t)7, (uint8_t)195, (uint8_t)181, (uint8_t)55, (uint8_t)82, (uint8_t)80, (uint8_t)116, (uint8_t)24, (uint8_t)45, (uint8_t)11, (uint8_t)59, (uint8_t)210, (uint8_t)115, (uint8_t)176, (uint8_t)199, (uint8_t)34, (uint8_t)147, (uint8_t)83, (uint8_t)230, (uint8_t)165, (uint8_t)61, (uint8_t)85, (uint8_t)133, (uint8_t)76, (uint8_t)52, (uint8_t)205, (uint8_t)2, (uint8_t)164, (uint8_t)180, (uint8_t)114, (uint8_t)211, (uint8_t)21, (uint8_t)184, (uint8_t)64, (uint8_t)65, (uint8_t)193, (uint8_t)96, (uint8_t)7, (uint8_t)208, (uint8_t)234, (uint8_t)104, (uint8_t)101, (uint8_t)156, (uint8_t)237, (uint8_t)220, (uint8_t)92, (uint8_t)64, (uint8_t)51, (uint8_t)46, (uint8_t)197, (uint8_t)158, (uint8_t)188, (uint8_t)85, (uint8_t)239, (uint8_t)175, (uint8_t)1, (uint8_t)90, (uint8_t)244, (uint8_t)62, (uint8_t)108, (uint8_t)111, (uint8_t)172, (uint8_t)205, (uint8_t)3, (uint8_t)237, (uint8_t)42, (uint8_t)212, (uint8_t)160, (uint8_t)10, (uint8_t)179, (uint8_t)170, (uint8_t)157, (uint8_t)97, (uint8_t)13, (uint8_t)121, (uint8_t)188, (uint8_t)249, (uint8_t)81, (uint8_t)247, (uint8_t)128, (uint8_t)34, (uint8_t)185, (uint8_t)91, (uint8_t)78, (uint8_t)41, (uint8_t)23, (uint8_t)8, (uint8_t)94, (uint8_t)207, (uint8_t)237, (uint8_t)123, (uint8_t)4, (uint8_t)22, (uint8_t)189, (uint8_t)16, (uint8_t)189, (uint8_t)33, (uint8_t)26, (uint8_t)182, (uint8_t)204, (uint8_t)16, (uint8_t)123, (uint8_t)224, (uint8_t)164, (uint8_t)216, (uint8_t)236, (uint8_t)152, (uint8_t)87, (uint8_t)55, (uint8_t)188, (uint8_t)100, (uint8_t)49, (uint8_t)29, (uint8_t)167, (uint8_t)178, (uint8_t)72, (uint8_t)181, (uint8_t)80, (uint8_t)9, (uint8_t)118, (uint8_t)183, (uint8_t)17, (uint8_t)230, (uint8_t)118, (uint8_t)118, (uint8_t)147, (uint8_t)175, (uint8_t)140, (uint8_t)34, (uint8_t)91, (uint8_t)77, (uint8_t)185, (uint8_t)102, (uint8_t)29, (uint8_t)2, (uint8_t)78, (uint8_t)229, (uint8_t)153, (uint8_t)211, (uint8_t)108, (uint8_t)210, (uint8_t)40, (uint8_t)56, (uint8_t)11, (uint8_t)160, (uint8_t)213, (uint8_t)109, (uint8_t)125, (uint8_t)248, (uint8_t)248, (uint8_t)164, (uint8_t)207, (uint8_t)110, (uint8_t)235, (uint8_t)185, (uint8_t)45, (uint8_t)42} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)204);
};


void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX);
    assert(p234_longitude_GET(pack) == (int32_t)1358338555);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -100);
    assert(p234_custom_mode_GET(pack) == (uint32_t)4114096407L);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -28086);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -26246);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)24062);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)62110);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)94);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)58309);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)46);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)4);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
    assert(p234_latitude_GET(pack) == (int32_t)108767120);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -725);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -5671);
};


void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)4041231426L);
    assert(p241_time_usec_GET(pack) == (uint64_t)882635702220982184L);
    assert(p241_vibration_z_GET(pack) == (float)3.116239E38F);
    assert(p241_clipping_0_GET(pack) == (uint32_t)512413848L);
    assert(p241_vibration_x_GET(pack) == (float)1.2221355E38F);
    assert(p241_vibration_y_GET(pack) == (float)3.0571384E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2471107131L);
};


void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_time_usec_TRY(ph) == (uint64_t)2454255099536895557L);
    assert(p242_approach_y_GET(pack) == (float)2.3535956E38F);
    assert(p242_x_GET(pack) == (float)1.8047728E38F);
    {
        float exemplary[] =  {-1.6533501E38F, -4.7287774E37F, -1.9639918E38F, -1.7300291E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_altitude_GET(pack) == (int32_t) -707791961);
    assert(p242_latitude_GET(pack) == (int32_t)71992710);
    assert(p242_longitude_GET(pack) == (int32_t)2146338447);
    assert(p242_z_GET(pack) == (float) -1.2219226E38F);
    assert(p242_approach_x_GET(pack) == (float)2.8618478E38F);
    assert(p242_approach_z_GET(pack) == (float) -1.3114259E38F);
    assert(p242_y_GET(pack) == (float)1.247611E38F);
};


void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_x_GET(pack) == (float)1.7091021E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)196);
    assert(p243_z_GET(pack) == (float) -2.6938314E38F);
    assert(p243_approach_y_GET(pack) == (float)1.8215078E38F);
    assert(p243_latitude_GET(pack) == (int32_t) -2114986601);
    assert(p243_time_usec_TRY(ph) == (uint64_t)3354115931226010498L);
    assert(p243_approach_x_GET(pack) == (float) -5.394461E37F);
    assert(p243_y_GET(pack) == (float)1.8753587E37F);
    assert(p243_altitude_GET(pack) == (int32_t)831345242);
    {
        float exemplary[] =  {1.4908407E38F, -2.7580852E38F, 1.0973722E38F, -2.5749864E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_z_GET(pack) == (float) -3.046313E38F);
    assert(p243_longitude_GET(pack) == (int32_t)163589690);
};


void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)42821);
    assert(p244_interval_us_GET(pack) == (int32_t)1890316597);
};


void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
};


void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_altitude_GET(pack) == (int32_t) -1393685622);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE);
    assert(p246_lat_GET(pack) == (int32_t)1226059304);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)47304);
    assert(p246_callsign_LEN(ph) == 2);
    {
        char16_t * exemplary = u"wz";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lon_GET(pack) == (int32_t)405725548);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2441476185L);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)34982);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -20173);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)29625);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)139);
};


void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_time_to_minimum_delta_GET(pack) == (float)2.9548535E37F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_id_GET(pack) == (uint32_t)2079509726L);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)1.9880447E38F);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)1.4039092E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT);
};


void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)232, (uint8_t)26, (uint8_t)217, (uint8_t)233, (uint8_t)83, (uint8_t)227, (uint8_t)105, (uint8_t)106, (uint8_t)193, (uint8_t)18, (uint8_t)16, (uint8_t)238, (uint8_t)32, (uint8_t)65, (uint8_t)25, (uint8_t)179, (uint8_t)111, (uint8_t)221, (uint8_t)55, (uint8_t)23, (uint8_t)200, (uint8_t)168, (uint8_t)96, (uint8_t)148, (uint8_t)227, (uint8_t)72, (uint8_t)111, (uint8_t)45, (uint8_t)169, (uint8_t)64, (uint8_t)208, (uint8_t)207, (uint8_t)16, (uint8_t)21, (uint8_t)53, (uint8_t)182, (uint8_t)153, (uint8_t)144, (uint8_t)146, (uint8_t)207, (uint8_t)191, (uint8_t)248, (uint8_t)244, (uint8_t)242, (uint8_t)115, (uint8_t)96, (uint8_t)163, (uint8_t)59, (uint8_t)174, (uint8_t)185, (uint8_t)36, (uint8_t)167, (uint8_t)110, (uint8_t)110, (uint8_t)195, (uint8_t)32, (uint8_t)142, (uint8_t)75, (uint8_t)25, (uint8_t)234, (uint8_t)187, (uint8_t)88, (uint8_t)75, (uint8_t)171, (uint8_t)149, (uint8_t)69, (uint8_t)26, (uint8_t)183, (uint8_t)94, (uint8_t)99, (uint8_t)189, (uint8_t)27, (uint8_t)154, (uint8_t)69, (uint8_t)13, (uint8_t)47, (uint8_t)75, (uint8_t)166, (uint8_t)100, (uint8_t)196, (uint8_t)73, (uint8_t)175, (uint8_t)67, (uint8_t)109, (uint8_t)227, (uint8_t)221, (uint8_t)126, (uint8_t)219, (uint8_t)54, (uint8_t)234, (uint8_t)177, (uint8_t)122, (uint8_t)75, (uint8_t)97, (uint8_t)217, (uint8_t)112, (uint8_t)83, (uint8_t)43, (uint8_t)82, (uint8_t)44, (uint8_t)9, (uint8_t)167, (uint8_t)205, (uint8_t)5, (uint8_t)136, (uint8_t)234, (uint8_t)17, (uint8_t)159, (uint8_t)120, (uint8_t)128, (uint8_t)192, (uint8_t)72, (uint8_t)1, (uint8_t)218, (uint8_t)85, (uint8_t)77, (uint8_t)97, (uint8_t)246, (uint8_t)120, (uint8_t)215, (uint8_t)9, (uint8_t)31, (uint8_t)157, (uint8_t)68, (uint8_t)241, (uint8_t)152, (uint8_t)170, (uint8_t)47, (uint8_t)250, (uint8_t)178, (uint8_t)61, (uint8_t)180, (uint8_t)210, (uint8_t)125, (uint8_t)9, (uint8_t)201, (uint8_t)104, (uint8_t)148, (uint8_t)80, (uint8_t)60, (uint8_t)88, (uint8_t)76, (uint8_t)51, (uint8_t)38, (uint8_t)177, (uint8_t)152, (uint8_t)241, (uint8_t)58, (uint8_t)13, (uint8_t)189, (uint8_t)83, (uint8_t)161, (uint8_t)249, (uint8_t)169, (uint8_t)252, (uint8_t)178, (uint8_t)90, (uint8_t)183, (uint8_t)151, (uint8_t)46, (uint8_t)67, (uint8_t)46, (uint8_t)96, (uint8_t)131, (uint8_t)179, (uint8_t)103, (uint8_t)223, (uint8_t)93, (uint8_t)48, (uint8_t)131, (uint8_t)39, (uint8_t)56, (uint8_t)122, (uint8_t)205, (uint8_t)14, (uint8_t)118, (uint8_t)88, (uint8_t)126, (uint8_t)138, (uint8_t)86, (uint8_t)192, (uint8_t)62, (uint8_t)96, (uint8_t)220, (uint8_t)92, (uint8_t)181, (uint8_t)43, (uint8_t)123, (uint8_t)220, (uint8_t)35, (uint8_t)124, (uint8_t)178, (uint8_t)137, (uint8_t)168, (uint8_t)229, (uint8_t)21, (uint8_t)60, (uint8_t)150, (uint8_t)244, (uint8_t)41, (uint8_t)24, (uint8_t)121, (uint8_t)48, (uint8_t)188, (uint8_t)79, (uint8_t)248, (uint8_t)109, (uint8_t)66, (uint8_t)88, (uint8_t)191, (uint8_t)250, (uint8_t)110, (uint8_t)113, (uint8_t)157, (uint8_t)127, (uint8_t)229, (uint8_t)59, (uint8_t)139, (uint8_t)189, (uint8_t)184, (uint8_t)156, (uint8_t)120, (uint8_t)248, (uint8_t)32, (uint8_t)66, (uint8_t)113, (uint8_t)94, (uint8_t)108, (uint8_t)74, (uint8_t)85, (uint8_t)106, (uint8_t)39, (uint8_t)41, (uint8_t)162, (uint8_t)172, (uint8_t)11, (uint8_t)158, (uint8_t)116, (uint8_t)253, (uint8_t)244, (uint8_t)157, (uint8_t)249, (uint8_t)56, (uint8_t)144, (uint8_t)174, (uint8_t)127, (uint8_t)224, (uint8_t)203, (uint8_t)232} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)60249);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)19);
};


void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)250);
    {
        int8_t exemplary[] =  {(int8_t)2, (int8_t) -113, (int8_t)110, (int8_t) -86, (int8_t) -3, (int8_t)1, (int8_t) -103, (int8_t) -42, (int8_t)52, (int8_t)108, (int8_t)8, (int8_t) -95, (int8_t) -97, (int8_t) -42, (int8_t) -127, (int8_t) -91, (int8_t) -37, (int8_t)107, (int8_t)71, (int8_t) -56, (int8_t)75, (int8_t)125, (int8_t)5, (int8_t) -24, (int8_t) -41, (int8_t)102, (int8_t)99, (int8_t)110, (int8_t) -47, (int8_t)85, (int8_t)94, (int8_t) -43} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)41376);
};


void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_z_GET(pack) == (float)1.5941028E38F);
    assert(p250_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"oifhJakzca";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_y_GET(pack) == (float)1.2139076E38F);
    assert(p250_x_GET(pack) == (float) -1.5332906E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)8159568209444630093L);
};


void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_value_GET(pack) == (float)3.1352003E38F);
    assert(p251_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"xagvlrCc";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2654537867L);
};


void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -1035226958);
    assert(p252_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"hzydiqpgpo";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)359935504L);
};


void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 30);
    {
        char16_t * exemplary = u"ocmnkfkwbjctalIvPsnexVcEvnuGjh";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 60);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_INFO);
};


void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p254_value_GET(pack) == (float) -2.3882503E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)2963115417L);
};


void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)62);
    {
        uint8_t exemplary[] =  {(uint8_t)177, (uint8_t)108, (uint8_t)247, (uint8_t)119, (uint8_t)73, (uint8_t)225, (uint8_t)206, (uint8_t)55, (uint8_t)44, (uint8_t)177, (uint8_t)53, (uint8_t)190, (uint8_t)27, (uint8_t)85, (uint8_t)77, (uint8_t)15, (uint8_t)165, (uint8_t)99, (uint8_t)113, (uint8_t)74, (uint8_t)128, (uint8_t)168, (uint8_t)202, (uint8_t)4, (uint8_t)36, (uint8_t)75, (uint8_t)199, (uint8_t)211, (uint8_t)62, (uint8_t)57, (uint8_t)167, (uint8_t)122} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)2741474669856642626L);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)15);
};


void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)2646368701L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)280596937L);
};


void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p258_tune_LEN(ph) == 4);
    {
        char16_t * exemplary = u"bgpi";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)217);
};


void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
    assert(p259_cam_definition_uri_LEN(ph) == 13);
    {
        char16_t * exemplary = u"csfakbquavzzl";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3118774207L);
    assert(p259_sensor_size_h_GET(pack) == (float) -2.2215445E38F);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)64635);
    assert(p259_sensor_size_v_GET(pack) == (float) -1.5833426E38F);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)22763);
    assert(p259_focal_length_GET(pack) == (float)1.0712429E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)86, (uint8_t)26, (uint8_t)212, (uint8_t)132, (uint8_t)252, (uint8_t)251, (uint8_t)104, (uint8_t)82, (uint8_t)32, (uint8_t)85, (uint8_t)152, (uint8_t)124, (uint8_t)7, (uint8_t)253, (uint8_t)220, (uint8_t)205, (uint8_t)188, (uint8_t)52, (uint8_t)210, (uint8_t)142, (uint8_t)89, (uint8_t)181, (uint8_t)103, (uint8_t)127, (uint8_t)84, (uint8_t)43, (uint8_t)223, (uint8_t)155, (uint8_t)74, (uint8_t)71, (uint8_t)8, (uint8_t)29} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_firmware_version_GET(pack) == (uint32_t)2918143373L);
    {
        uint8_t exemplary[] =  {(uint8_t)86, (uint8_t)164, (uint8_t)171, (uint8_t)128, (uint8_t)78, (uint8_t)86, (uint8_t)149, (uint8_t)142, (uint8_t)236, (uint8_t)177, (uint8_t)130, (uint8_t)191, (uint8_t)163, (uint8_t)176, (uint8_t)57, (uint8_t)102, (uint8_t)145, (uint8_t)214, (uint8_t)142, (uint8_t)32, (uint8_t)132, (uint8_t)84, (uint8_t)182, (uint8_t)229, (uint8_t)122, (uint8_t)163, (uint8_t)36, (uint8_t)144, (uint8_t)128, (uint8_t)133, (uint8_t)37, (uint8_t)30} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)21567);
};


void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3535862960L);
};


void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_total_capacity_GET(pack) == (float)3.1888506E37F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)588537755L);
    assert(p261_available_capacity_GET(pack) == (float) -2.879073E38F);
    assert(p261_used_capacity_GET(pack) == (float) -4.0689302E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p261_read_speed_GET(pack) == (float) -1.1085021E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p261_write_speed_GET(pack) == (float) -1.2239222E38F);
};


void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2211287829L);
    assert(p262_available_capacity_GET(pack) == (float) -1.0744778E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p262_image_interval_GET(pack) == (float) -1.23333E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)4159496735L);
};


void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -18);
    assert(p263_alt_GET(pack) == (int32_t) -482453204);
    {
        float exemplary[] =  {1.1775467E38F, -3.3180424E38F, -8.0694616E37F, 5.0025793E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_image_index_GET(pack) == (int32_t) -431826598);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1210269162L);
    assert(p263_file_url_LEN(ph) == 22);
    {
        char16_t * exemplary = u"wjtokqurDuxltGbVoqjcbm";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t) -736161159);
    assert(p263_lon_GET(pack) == (int32_t) -713977606);
    assert(p263_lat_GET(pack) == (int32_t) -799424672);
    assert(p263_time_utc_GET(pack) == (uint64_t)6769819443334114525L);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)197);
};


void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)6827057483460746991L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2860305402L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)5863134317068952273L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)3959992065160282516L);
};


void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_pitch_GET(pack) == (float)1.512047E38F);
    assert(p265_yaw_GET(pack) == (float)1.9685539E38F);
    assert(p265_roll_GET(pack) == (float) -5.4970527E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1418281958L);
};


void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)49581);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)164);
    {
        uint8_t exemplary[] =  {(uint8_t)211, (uint8_t)140, (uint8_t)206, (uint8_t)121, (uint8_t)56, (uint8_t)56, (uint8_t)145, (uint8_t)148, (uint8_t)10, (uint8_t)154, (uint8_t)173, (uint8_t)125, (uint8_t)162, (uint8_t)163, (uint8_t)187, (uint8_t)51, (uint8_t)96, (uint8_t)179, (uint8_t)103, (uint8_t)121, (uint8_t)182, (uint8_t)178, (uint8_t)0, (uint8_t)168, (uint8_t)179, (uint8_t)18, (uint8_t)58, (uint8_t)188, (uint8_t)127, (uint8_t)155, (uint8_t)183, (uint8_t)88, (uint8_t)57, (uint8_t)40, (uint8_t)157, (uint8_t)178, (uint8_t)122, (uint8_t)180, (uint8_t)99, (uint8_t)11, (uint8_t)151, (uint8_t)73, (uint8_t)120, (uint8_t)68, (uint8_t)32, (uint8_t)227, (uint8_t)20, (uint8_t)133, (uint8_t)153, (uint8_t)76, (uint8_t)67, (uint8_t)97, (uint8_t)242, (uint8_t)6, (uint8_t)58, (uint8_t)117, (uint8_t)127, (uint8_t)122, (uint8_t)71, (uint8_t)217, (uint8_t)226, (uint8_t)195, (uint8_t)92, (uint8_t)200, (uint8_t)80, (uint8_t)66, (uint8_t)226, (uint8_t)51, (uint8_t)141, (uint8_t)159, (uint8_t)51, (uint8_t)43, (uint8_t)68, (uint8_t)224, (uint8_t)155, (uint8_t)239, (uint8_t)26, (uint8_t)187, (uint8_t)101, (uint8_t)152, (uint8_t)57, (uint8_t)14, (uint8_t)1, (uint8_t)113, (uint8_t)162, (uint8_t)71, (uint8_t)10, (uint8_t)97, (uint8_t)222, (uint8_t)187, (uint8_t)136, (uint8_t)154, (uint8_t)98, (uint8_t)234, (uint8_t)175, (uint8_t)195, (uint8_t)1, (uint8_t)58, (uint8_t)229, (uint8_t)145, (uint8_t)191, (uint8_t)74, (uint8_t)167, (uint8_t)176, (uint8_t)227, (uint8_t)214, (uint8_t)134, (uint8_t)215, (uint8_t)187, (uint8_t)42, (uint8_t)81, (uint8_t)203, (uint8_t)151, (uint8_t)52, (uint8_t)130, (uint8_t)251, (uint8_t)93, (uint8_t)174, (uint8_t)8, (uint8_t)194, (uint8_t)144, (uint8_t)217, (uint8_t)7, (uint8_t)255, (uint8_t)79, (uint8_t)195, (uint8_t)91, (uint8_t)87, (uint8_t)179, (uint8_t)174, (uint8_t)95, (uint8_t)215, (uint8_t)13, (uint8_t)81, (uint8_t)30, (uint8_t)44, (uint8_t)239, (uint8_t)69, (uint8_t)94, (uint8_t)30, (uint8_t)24, (uint8_t)155, (uint8_t)130, (uint8_t)247, (uint8_t)151, (uint8_t)113, (uint8_t)252, (uint8_t)187, (uint8_t)10, (uint8_t)56, (uint8_t)218, (uint8_t)248, (uint8_t)76, (uint8_t)247, (uint8_t)238, (uint8_t)235, (uint8_t)45, (uint8_t)12, (uint8_t)230, (uint8_t)82, (uint8_t)93, (uint8_t)39, (uint8_t)45, (uint8_t)230, (uint8_t)220, (uint8_t)137, (uint8_t)127, (uint8_t)249, (uint8_t)192, (uint8_t)143, (uint8_t)215, (uint8_t)182, (uint8_t)215, (uint8_t)125, (uint8_t)232, (uint8_t)104, (uint8_t)147, (uint8_t)60, (uint8_t)119, (uint8_t)131, (uint8_t)220, (uint8_t)17, (uint8_t)128, (uint8_t)198, (uint8_t)238, (uint8_t)77, (uint8_t)24, (uint8_t)177, (uint8_t)252, (uint8_t)183, (uint8_t)199, (uint8_t)212, (uint8_t)66, (uint8_t)131, (uint8_t)158, (uint8_t)180, (uint8_t)255, (uint8_t)0, (uint8_t)233, (uint8_t)12, (uint8_t)86, (uint8_t)18, (uint8_t)239, (uint8_t)240, (uint8_t)64, (uint8_t)194, (uint8_t)91, (uint8_t)148, (uint8_t)246, (uint8_t)236, (uint8_t)238, (uint8_t)180, (uint8_t)195, (uint8_t)2, (uint8_t)206, (uint8_t)6, (uint8_t)154, (uint8_t)103, (uint8_t)175, (uint8_t)117, (uint8_t)38, (uint8_t)204, (uint8_t)212, (uint8_t)10, (uint8_t)61, (uint8_t)224, (uint8_t)141, (uint8_t)104, (uint8_t)204, (uint8_t)22, (uint8_t)210, (uint8_t)129, (uint8_t)14, (uint8_t)200, (uint8_t)42, (uint8_t)143, (uint8_t)26, (uint8_t)5, (uint8_t)110, (uint8_t)132, (uint8_t)226, (uint8_t)182, (uint8_t)223, (uint8_t)186, (uint8_t)59, (uint8_t)87, (uint8_t)253, (uint8_t)182, (uint8_t)128} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)24, (uint8_t)77, (uint8_t)175, (uint8_t)110, (uint8_t)56, (uint8_t)174, (uint8_t)58, (uint8_t)255, (uint8_t)248, (uint8_t)152, (uint8_t)40, (uint8_t)221, (uint8_t)115, (uint8_t)140, (uint8_t)239, (uint8_t)237, (uint8_t)167, (uint8_t)25, (uint8_t)161, (uint8_t)18, (uint8_t)189, (uint8_t)104, (uint8_t)125, (uint8_t)105, (uint8_t)181, (uint8_t)29, (uint8_t)1, (uint8_t)145, (uint8_t)94, (uint8_t)229, (uint8_t)176, (uint8_t)134, (uint8_t)134, (uint8_t)227, (uint8_t)166, (uint8_t)21, (uint8_t)128, (uint8_t)46, (uint8_t)239, (uint8_t)250, (uint8_t)161, (uint8_t)49, (uint8_t)35, (uint8_t)42, (uint8_t)121, (uint8_t)124, (uint8_t)160, (uint8_t)108, (uint8_t)162, (uint8_t)227, (uint8_t)229, (uint8_t)69, (uint8_t)81, (uint8_t)40, (uint8_t)67, (uint8_t)219, (uint8_t)233, (uint8_t)5, (uint8_t)147, (uint8_t)225, (uint8_t)218, (uint8_t)165, (uint8_t)168, (uint8_t)221, (uint8_t)107, (uint8_t)162, (uint8_t)234, (uint8_t)203, (uint8_t)35, (uint8_t)84, (uint8_t)213, (uint8_t)93, (uint8_t)174, (uint8_t)101, (uint8_t)52, (uint8_t)118, (uint8_t)8, (uint8_t)168, (uint8_t)80, (uint8_t)34, (uint8_t)191, (uint8_t)144, (uint8_t)129, (uint8_t)154, (uint8_t)225, (uint8_t)40, (uint8_t)27, (uint8_t)13, (uint8_t)46, (uint8_t)178, (uint8_t)220, (uint8_t)100, (uint8_t)223, (uint8_t)164, (uint8_t)128, (uint8_t)210, (uint8_t)86, (uint8_t)250, (uint8_t)67, (uint8_t)209, (uint8_t)190, (uint8_t)18, (uint8_t)35, (uint8_t)153, (uint8_t)23, (uint8_t)243, (uint8_t)223, (uint8_t)191, (uint8_t)12, (uint8_t)220, (uint8_t)139, (uint8_t)6, (uint8_t)193, (uint8_t)162, (uint8_t)131, (uint8_t)214, (uint8_t)90, (uint8_t)196, (uint8_t)218, (uint8_t)142, (uint8_t)153, (uint8_t)225, (uint8_t)97, (uint8_t)1, (uint8_t)203, (uint8_t)137, (uint8_t)221, (uint8_t)156, (uint8_t)188, (uint8_t)144, (uint8_t)76, (uint8_t)158, (uint8_t)167, (uint8_t)207, (uint8_t)203, (uint8_t)79, (uint8_t)143, (uint8_t)194, (uint8_t)101, (uint8_t)13, (uint8_t)30, (uint8_t)194, (uint8_t)72, (uint8_t)15, (uint8_t)169, (uint8_t)33, (uint8_t)194, (uint8_t)252, (uint8_t)21, (uint8_t)89, (uint8_t)51, (uint8_t)184, (uint8_t)132, (uint8_t)157, (uint8_t)20, (uint8_t)190, (uint8_t)163, (uint8_t)117, (uint8_t)34, (uint8_t)172, (uint8_t)107, (uint8_t)159, (uint8_t)156, (uint8_t)22, (uint8_t)142, (uint8_t)109, (uint8_t)18, (uint8_t)51, (uint8_t)51, (uint8_t)190, (uint8_t)251, (uint8_t)43, (uint8_t)194, (uint8_t)137, (uint8_t)162, (uint8_t)84, (uint8_t)25, (uint8_t)119, (uint8_t)209, (uint8_t)178, (uint8_t)137, (uint8_t)16, (uint8_t)161, (uint8_t)240, (uint8_t)103, (uint8_t)100, (uint8_t)203, (uint8_t)151, (uint8_t)190, (uint8_t)252, (uint8_t)92, (uint8_t)223, (uint8_t)193, (uint8_t)43, (uint8_t)11, (uint8_t)184, (uint8_t)226, (uint8_t)252, (uint8_t)215, (uint8_t)49, (uint8_t)54, (uint8_t)78, (uint8_t)213, (uint8_t)208, (uint8_t)80, (uint8_t)162, (uint8_t)166, (uint8_t)117, (uint8_t)30, (uint8_t)169, (uint8_t)65, (uint8_t)18, (uint8_t)249, (uint8_t)93, (uint8_t)21, (uint8_t)166, (uint8_t)19, (uint8_t)185, (uint8_t)39, (uint8_t)220, (uint8_t)111, (uint8_t)54, (uint8_t)48, (uint8_t)35, (uint8_t)241, (uint8_t)103, (uint8_t)23, (uint8_t)156, (uint8_t)107, (uint8_t)13, (uint8_t)180, (uint8_t)36, (uint8_t)38, (uint8_t)67, (uint8_t)74, (uint8_t)255, (uint8_t)148, (uint8_t)71, (uint8_t)178, (uint8_t)116, (uint8_t)214, (uint8_t)141, (uint8_t)248, (uint8_t)76, (uint8_t)39, (uint8_t)76, (uint8_t)67, (uint8_t)39, (uint8_t)0} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)7401);
};


void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)35260);
};


void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float) -2.2996314E38F);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)54015);
    assert(p269_uri_LEN(ph) == 19);
    {
        char16_t * exemplary = u"eeqesmgbaslwylbzlqz";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)30691);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)31445);
    assert(p269_bitrate_GET(pack) == (uint32_t)472520685L);
};


void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)63578);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p270_uri_LEN(ph) == 136);
    {
        char16_t * exemplary = u"dqhzmCrHDUlojctwoalGmohezmbyguoqmeslczjyhggxpqwpxartydtvtvecuixsxsuqrxymqoubdYghhsiftheSvbYfdtuxfpMwebkWOtRtqehewhhhAmghrvpsHfsgfzlcokdD";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 272);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)36259);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p270_framerate_GET(pack) == (float) -5.5285564E37F);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)11986);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p270_bitrate_GET(pack) == (uint32_t)1438391989L);
};


void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 50);
    {
        char16_t * exemplary = u"mhwcgksvkpcdpfnsiwebMfgqgegbaxxadhzDutsfutfgovYkoe";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 100);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 25);
    {
        char16_t * exemplary = u"jjPhxdwafhipmcyyqelihgfex";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)48381);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)28604);
    {
        uint8_t exemplary[] =  {(uint8_t)249, (uint8_t)226, (uint8_t)30, (uint8_t)99, (uint8_t)246, (uint8_t)27, (uint8_t)205, (uint8_t)7} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)252, (uint8_t)8, (uint8_t)12, (uint8_t)187, (uint8_t)180, (uint8_t)76, (uint8_t)158, (uint8_t)219} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)23744);
};


void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)19124);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE);
    assert(p310_time_usec_GET(pack) == (uint64_t)8154517258817837868L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)133071442L);
};


void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_time_usec_GET(pack) == (uint64_t)2876643348953795868L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)316094876L);
    assert(p311_name_LEN(ph) == 47);
    {
        char16_t * exemplary = u"lkEajxchnADgdjymiwuihykksmulBudChmgbtkDfqotiaom";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 94);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)213);
    {
        uint8_t exemplary[] =  {(uint8_t)237, (uint8_t)233, (uint8_t)76, (uint8_t)53, (uint8_t)236, (uint8_t)138, (uint8_t)190, (uint8_t)11, (uint8_t)193, (uint8_t)147, (uint8_t)220, (uint8_t)46, (uint8_t)61, (uint8_t)198, (uint8_t)60, (uint8_t)16} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1971430405L);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p320_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"vmmzjbhCfe";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -5115);
};


void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)154);
};


void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"galq";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p322_param_value_LEN(ph) == 21);
    {
        char16_t * exemplary = u"lugnLnfykmhjhMtxtnnhn";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)34112);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)48508);
};


void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p323_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"vlpalqvw";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p323_param_value_LEN(ph) == 74);
    {
        char16_t * exemplary = u"lBsmjqpqdipwllmxxdhcxqUzyauqklxQfzqzlwbEeiGegeCmLkcdagKyamRkjvotGpynzprWnd";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 148);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED);
    assert(p324_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"qskwlDoxyaVu";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 3);
    {
        char16_t * exemplary = u"iaS";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    {
        uint16_t exemplary[] =  {(uint16_t)49166, (uint16_t)52684, (uint16_t)2198, (uint16_t)56994, (uint16_t)13890, (uint16_t)59265, (uint16_t)65203, (uint16_t)44098, (uint16_t)9084, (uint16_t)6933, (uint16_t)35454, (uint16_t)13097, (uint16_t)64739, (uint16_t)41073, (uint16_t)18316, (uint16_t)43699, (uint16_t)39750, (uint16_t)37281, (uint16_t)51553, (uint16_t)65342, (uint16_t)39108, (uint16_t)22871, (uint16_t)3155, (uint16_t)13512, (uint16_t)13708, (uint16_t)60703, (uint16_t)54336, (uint16_t)2545, (uint16_t)42827, (uint16_t)42271, (uint16_t)51833, (uint16_t)15201, (uint16_t)60999, (uint16_t)28286, (uint16_t)40280, (uint16_t)50843, (uint16_t)40735, (uint16_t)393, (uint16_t)9844, (uint16_t)57105, (uint16_t)3758, (uint16_t)26045, (uint16_t)8787, (uint16_t)22542, (uint16_t)35023, (uint16_t)26569, (uint16_t)64225, (uint16_t)5695, (uint16_t)12930, (uint16_t)53732, (uint16_t)31017, (uint16_t)61962, (uint16_t)26745, (uint16_t)64794, (uint16_t)44448, (uint16_t)55557, (uint16_t)22871, (uint16_t)36455, (uint16_t)58459, (uint16_t)59300, (uint16_t)24537, (uint16_t)60795, (uint16_t)35984, (uint16_t)16702, (uint16_t)56165, (uint16_t)39272, (uint16_t)53206, (uint16_t)41548, (uint16_t)50761, (uint16_t)41295, (uint16_t)14727, (uint16_t)3777} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)13695);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p330_time_usec_GET(pack) == (uint64_t)7180420783902995200L);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)59158);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)191);
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
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)1060580138L, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_EMERGENCY, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_HEXAROTOR, PH.base.pack) ;
        c_TEST_Channel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
        p1_current_battery_SET((int16_t)(int16_t)5213, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)4324, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)25889, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)61, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)28302, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)36571, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)62993, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)51255, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)22582, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)34970, PH.base.pack) ;
        c_TEST_Channel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2242517153L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)3198018199476426707L, PH.base.pack) ;
        c_TEST_Channel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_SET((float) -9.536966E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2116895762L, PH.base.pack) ;
        p3_vx_SET((float)2.5359652E38F, PH.base.pack) ;
        p3_vz_SET((float)2.567307E38F, PH.base.pack) ;
        p3_vy_SET((float) -1.9872486E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.9060226E38F, PH.base.pack) ;
        p3_y_SET((float)2.8609913E37F, PH.base.pack) ;
        p3_z_SET((float)7.6401357E37F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)1146, PH.base.pack) ;
        p3_afx_SET((float)1.2898879E38F, PH.base.pack) ;
        p3_afy_SET((float) -2.3905748E38F, PH.base.pack) ;
        p3_afz_SET((float) -2.846884E38F, PH.base.pack) ;
        p3_x_SET((float)4.26203E37F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PING_4(), &PH);
        p4_seq_SET((uint32_t)3812964835L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)4011420812040978761L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        c_TEST_Channel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"qykdsDevf";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_gcs_system_id_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        c_TEST_Channel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"tibqiPwObeTsu";
            p7_key_SET_(key, &PH) ;
        }
        c_TEST_Channel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)3465195074L, PH.base.pack) ;
        c_TEST_Channel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t) -11262, PH.base.pack) ;
        {
            char16_t* param_id = u"nWC";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_count_SET((uint16_t)(uint16_t)4123, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT64, PH.base.pack) ;
        p22_param_value_SET((float)1.3713126E38F, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)39806, PH.base.pack) ;
        {
            char16_t* param_id = u"h";
            p22_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p23_param_value_SET((float) -7.0358137E37F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        {
            char16_t* param_id = u"y";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_TEST_Channel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
        p24_lon_SET((int32_t)114680914, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1404997642L, &PH) ;
        p24_v_acc_SET((uint32_t)1012419365L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)57104, PH.base.pack) ;
        p24_lat_SET((int32_t)1443088790, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)499163218996559150L, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p24_alt_SET((int32_t)1417941598, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)33617, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)50883, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)39781, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)3288883985L, &PH) ;
        p24_vel_acc_SET((uint32_t)2861303950L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t) -558919851, &PH) ;
        c_TEST_Channel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_prn[] =  {(uint8_t)26, (uint8_t)64, (uint8_t)252, (uint8_t)31, (uint8_t)65, (uint8_t)74, (uint8_t)167, (uint8_t)2, (uint8_t)144, (uint8_t)237, (uint8_t)199, (uint8_t)84, (uint8_t)73, (uint8_t)19, (uint8_t)68, (uint8_t)235, (uint8_t)81, (uint8_t)94, (uint8_t)247, (uint8_t)4};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)180, (uint8_t)101, (uint8_t)218, (uint8_t)73, (uint8_t)219, (uint8_t)219, (uint8_t)235, (uint8_t)19, (uint8_t)64, (uint8_t)65, (uint8_t)246, (uint8_t)86, (uint8_t)217, (uint8_t)215, (uint8_t)178, (uint8_t)5, (uint8_t)252, (uint8_t)93, (uint8_t)3, (uint8_t)183};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)114, (uint8_t)180, (uint8_t)153, (uint8_t)61, (uint8_t)214, (uint8_t)200, (uint8_t)123, (uint8_t)36, (uint8_t)132, (uint8_t)222, (uint8_t)158, (uint8_t)47, (uint8_t)14, (uint8_t)125, (uint8_t)232, (uint8_t)51, (uint8_t)117, (uint8_t)188, (uint8_t)231, (uint8_t)121};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        {
            uint8_t satellite_elevation[] =  {(uint8_t)181, (uint8_t)61, (uint8_t)194, (uint8_t)99, (uint8_t)44, (uint8_t)32, (uint8_t)45, (uint8_t)181, (uint8_t)141, (uint8_t)32, (uint8_t)159, (uint8_t)69, (uint8_t)200, (uint8_t)8, (uint8_t)63, (uint8_t)108, (uint8_t)154, (uint8_t)101, (uint8_t)197, (uint8_t)155};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)216, (uint8_t)250, (uint8_t)23, (uint8_t)50, (uint8_t)218, (uint8_t)87, (uint8_t)13, (uint8_t)27, (uint8_t)83, (uint8_t)89, (uint8_t)134, (uint8_t)85, (uint8_t)147, (uint8_t)118, (uint8_t)249, (uint8_t)227, (uint8_t)238, (uint8_t)143, (uint8_t)227, (uint8_t)67};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
        p26_xacc_SET((int16_t)(int16_t) -9897, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -15466, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -23804, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)1621, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)30655, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)4195035627L, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)4274, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)12414, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -2529, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -13993, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
        p27_ymag_SET((int16_t)(int16_t)9074, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -22445, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)21572, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -22131, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)13999, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)2185917278329366456L, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -17759, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -27893, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)9019, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -29433, PH.base.pack) ;
        c_TEST_Channel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -19865, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)7069648733676375890L, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t)26205, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -11262, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -4349, PH.base.pack) ;
        c_TEST_Channel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)2.9470258E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)3316, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)2684787132L, PH.base.pack) ;
        p29_press_diff_SET((float)1.1910772E38F, PH.base.pack) ;
        c_TEST_Channel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
        p30_roll_SET((float) -2.1461897E38F, PH.base.pack) ;
        p30_yawspeed_SET((float)2.2654634E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)2254311448L, PH.base.pack) ;
        p30_pitchspeed_SET((float) -1.936333E38F, PH.base.pack) ;
        p30_yaw_SET((float)2.3116E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)2.441565E38F, PH.base.pack) ;
        p30_pitch_SET((float)7.469015E37F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q2_SET((float) -1.4622262E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -9.218744E36F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)3204280937L, PH.base.pack) ;
        p31_pitchspeed_SET((float) -1.831005E38F, PH.base.pack) ;
        p31_q1_SET((float) -2.8604319E38F, PH.base.pack) ;
        p31_q3_SET((float) -1.8792308E38F, PH.base.pack) ;
        p31_q4_SET((float)6.939969E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)2.8034677E38F, PH.base.pack) ;
        c_TEST_Channel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vx_SET((float)1.9415148E38F, PH.base.pack) ;
        p32_vy_SET((float)5.8132296E37F, PH.base.pack) ;
        p32_vz_SET((float)5.6205295E37F, PH.base.pack) ;
        p32_z_SET((float) -2.7297203E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)3952334373L, PH.base.pack) ;
        p32_y_SET((float) -2.0961057E38F, PH.base.pack) ;
        p32_x_SET((float) -3.0636393E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_relative_alt_SET((int32_t) -754597052, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)19453, PH.base.pack) ;
        p33_lat_SET((int32_t) -1998603350, PH.base.pack) ;
        p33_alt_SET((int32_t) -1771582194, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)31509, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)28480, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -3740, PH.base.pack) ;
        p33_lon_SET((int32_t) -2099759895, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1626725965L, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan5_scaled_SET((int16_t)(int16_t) -15668, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)14911, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)4195384067L, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -2515, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t) -26774, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -5405, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -26844, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -29964, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)4751, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan4_raw_SET((uint16_t)(uint16_t)34231, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)61503, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)5290, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)62099, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)16410, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)62484, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)48665, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)439454459L, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)32202, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo1_raw_SET((uint16_t)(uint16_t)19883, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)24026, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)62428, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)2717, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)64, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)30656, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)39226, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)21278, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)716, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)15811, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)316320968L, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)32540, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)61762, &PH) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)25335, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)54166, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)26914, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)62327, PH.base.pack) ;
        c_TEST_Channel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -938, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)30639, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t)1174, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -26582, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
        p39_seq_SET((uint16_t)(uint16_t)21421, PH.base.pack) ;
        p39_param3_SET((float) -2.0935497E38F, PH.base.pack) ;
        p39_x_SET((float)2.1805667E38F, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p39_param4_SET((float) -2.4730587E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p39_param2_SET((float)9.873219E37F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p39_y_SET((float) -2.8680394E38F, PH.base.pack) ;
        p39_z_SET((float)8.042283E37F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p39_param1_SET((float) -2.2971332E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_GET_MESSAGE_INTERVAL, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)1034, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)55467, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)2176, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)65385, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)10227, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
        p47_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -104330735, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p48_longitude_SET((int32_t)1548038265, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)7986251425166636900L, &PH) ;
        p48_latitude_SET((int32_t) -1107747989, PH.base.pack) ;
        c_TEST_Channel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)7499366453014257578L, &PH) ;
        p49_latitude_SET((int32_t) -1230851608, PH.base.pack) ;
        p49_altitude_SET((int32_t) -160417034, PH.base.pack) ;
        p49_longitude_SET((int32_t)2021373281, PH.base.pack) ;
        c_TEST_Channel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p50_param_value_max_SET((float)1.5763475E38F, PH.base.pack) ;
        p50_param_value_min_SET((float) -1.3585218E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)24009, PH.base.pack) ;
        p50_param_value0_SET((float) -1.8214967E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"framFjuskfblfvf";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_scale_SET((float) -2.2894135E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        c_TEST_Channel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_seq_SET((uint16_t)(uint16_t)43580, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p54_p2z_SET((float)2.4105056E38F, PH.base.pack) ;
        p54_p1x_SET((float) -2.6919992E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p54_p2x_SET((float)1.8909043E38F, PH.base.pack) ;
        p54_p2y_SET((float)2.1583709E38F, PH.base.pack) ;
        p54_p1y_SET((float)1.8246376E38F, PH.base.pack) ;
        p54_p1z_SET((float)2.4033465E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float) -1.6551789E38F, PH.base.pack) ;
        p55_p1y_SET((float) -2.0453664E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p55_p2z_SET((float)2.7621934E38F, PH.base.pack) ;
        p55_p1x_SET((float)1.0506598E38F, PH.base.pack) ;
        p55_p1z_SET((float) -2.5925513E38F, PH.base.pack) ;
        p55_p2y_SET((float) -2.8159947E38F, PH.base.pack) ;
        c_TEST_Channel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)1915774830010135902L, PH.base.pack) ;
        p61_pitchspeed_SET((float)1.9826268E38F, PH.base.pack) ;
        p61_rollspeed_SET((float)2.030161E38F, PH.base.pack) ;
        p61_yawspeed_SET((float)9.815524E36F, PH.base.pack) ;
        {
            float q[] =  {2.5586175E38F, 3.2426476E38F, 3.3034587E38F, 1.031372E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float covariance[] =  {-3.2544935E38F, -1.3090321E38F, -7.6117075E36F, -3.1805494E38F, 3.231305E38F, -1.3326197E38F, -2.9180166E37F, 9.425019E37F, 2.1401365E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_TEST_Channel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_target_bearing_SET((int16_t)(int16_t) -11707, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)6638, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.9014549E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -1.9347344E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -1.8868835E38F, PH.base.pack) ;
        p62_aspd_error_SET((float) -4.0966386E37F, PH.base.pack) ;
        p62_alt_error_SET((float)2.3127817E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -7825, PH.base.pack) ;
        c_TEST_Channel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        {
            float covariance[] =  {4.3032686E37F, -3.3043925E38F, 1.7522638E37F, -2.1548908E38F, 2.6958395E38F, -1.507681E38F, -1.9416324E38F, 2.7541218E38F, 1.0569717E38F, -3.1261572E38F, -2.0381335E37F, 1.9274942E37F, 2.7061322E38F, 3.2801656E38F, 1.848939E37F, 1.4732196E38F, 1.0297665E38F, -2.7870622E37F, 1.3491902E36F, -1.6531066E38F, -1.1556947E38F, 1.3148132E38F, 1.7905364E38F, 2.4111812E37F, -1.290915E38F, 1.6316415E37F, 1.204887E38F, -1.5766196E38F, -1.5768208E37F, 2.7508154E38F, 3.1390063E38F, 3.2842574E38F, -2.799611E38F, 4.0775916E37F, -2.3163526E38F, -2.0488146E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p63_alt_SET((int32_t)170972593, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -669613037, PH.base.pack) ;
        p63_lat_SET((int32_t)1367472929, PH.base.pack) ;
        p63_vz_SET((float) -1.2070635E38F, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)335045694904186663L, PH.base.pack) ;
        p63_lon_SET((int32_t) -1290148841, PH.base.pack) ;
        p63_vx_SET((float) -5.0829863E37F, PH.base.pack) ;
        p63_vy_SET((float) -7.3954726E37F, PH.base.pack) ;
        c_TEST_Channel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vz_SET((float)2.1655444E38F, PH.base.pack) ;
        p64_vx_SET((float) -5.2273966E37F, PH.base.pack) ;
        p64_az_SET((float)2.875956E38F, PH.base.pack) ;
        p64_ay_SET((float)1.6268983E37F, PH.base.pack) ;
        p64_ax_SET((float)2.5563645E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)4317392645009816901L, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p64_z_SET((float)1.02569204E37F, PH.base.pack) ;
        p64_x_SET((float)2.925525E38F, PH.base.pack) ;
        {
            float covariance[] =  {-1.3613394E37F, 2.5741477E38F, 1.7874338E38F, 1.0178005E38F, 1.6722275E38F, -1.6315701E38F, -1.4450042E38F, 1.1964758E38F, 1.5416366E38F, -5.573217E37F, -2.7760096E38F, 1.188656E38F, -7.890439E37F, 2.1677586E38F, -4.4547635E37F, 1.7279702E38F, -2.8537478E38F, 3.1657858E38F, 2.6241907E38F, 2.6692401E38F, 2.018655E38F, 2.7895207E38F, -2.538418E37F, -7.8054856E37F, -3.0850519E38F, 1.5630342E38F, 1.7423997E38F, -4.5964666E37F, -2.1680815E38F, 3.2695774E38F, -2.8202881E38F, 4.4809284E37F, 1.8079497E38F, -2.995123E38F, 2.0301725E38F, -3.3471916E37F, -2.5865127E38F, -3.3549012E38F, -2.9246915E37F, -3.0546023E38F, -2.9720985E38F, 2.9709889E38F, 8.237016E36F, -1.3781805E38F, -1.1599966E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float) -3.1103284E38F, PH.base.pack) ;
        p64_vy_SET((float)2.6171121E38F, PH.base.pack) ;
        c_TEST_Channel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan18_raw_SET((uint16_t)(uint16_t)23451, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)19864, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)20523, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)30766, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)49688, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)55809, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)14572, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)39722, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)2457, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)45369, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)51503, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)41492, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)62715, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)7479, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)14019, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)36273, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)10388, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)77253232L, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)8414, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)26411, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        c_TEST_Channel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)36726, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        c_TEST_Channel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_r_SET((int16_t)(int16_t) -22903, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)11518, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -2498, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)4273, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -21645, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_TEST_Channel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan7_raw_SET((uint16_t)(uint16_t)32744, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)57039, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)55176, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)14203, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)45129, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)58120, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)59301, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)10505, PH.base.pack) ;
        c_TEST_Channel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param1_SET((float) -4.0458141E37F, PH.base.pack) ;
        p73_x_SET((int32_t) -2039775292, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)11386, PH.base.pack) ;
        p73_y_SET((int32_t) -83858132, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p73_param3_SET((float) -3.2134194E38F, PH.base.pack) ;
        p73_param2_SET((float)3.3725096E38F, PH.base.pack) ;
        p73_z_SET((float) -2.0959286E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p73_param4_SET((float)2.2699821E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_LOGGING_START, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_TEST_Channel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float) -1.4176743E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -21757, PH.base.pack) ;
        p74_alt_SET((float) -8.705219E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)8178, PH.base.pack) ;
        p74_climb_SET((float) -1.9481005E38F, PH.base.pack) ;
        p74_airspeed_SET((float) -1.6576167E38F, PH.base.pack) ;
        c_TEST_Channel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
        p75_param4_SET((float)5.7881034E36F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p75_x_SET((int32_t) -219261861, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p75_param1_SET((float)1.9833775E37F, PH.base.pack) ;
        p75_z_SET((float)2.770522E38F, PH.base.pack) ;
        p75_param3_SET((float)1.8315884E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE, PH.base.pack) ;
        p75_param2_SET((float) -2.743197E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -113448102, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
        p76_param3_SET((float)2.5268905E38F, PH.base.pack) ;
        p76_param4_SET((float)1.351585E38F, PH.base.pack) ;
        p76_param7_SET((float) -1.8072177E38F, PH.base.pack) ;
        p76_param1_SET((float) -1.8340146E38F, PH.base.pack) ;
        p76_param5_SET((float) -2.7841055E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p76_param6_SET((float) -2.2959288E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS, PH.base.pack) ;
        p76_param2_SET((float) -1.6833519E38F, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_component_SET((uint8_t)(uint8_t)171, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_HOME, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)183, &PH) ;
        p77_target_system_SET((uint8_t)(uint8_t)154, &PH) ;
        p77_result_param2_SET((int32_t)665842807, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
        c_TEST_Channel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff,  sizeof buff));) output_bytes_adv(&c_TEST_Channel, buff, len);
        c_TEST_Channel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_yaw_SET((float)6.652891E37F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)4225741204L, PH.base.pack) ;
        p81_roll_SET((float)4.5945154E37F, PH.base.pack) ;
        p81_pitch_SET((float)3.0220742E38F, PH.base.pack) ;
        p81_thrust_SET((float)1.4718034E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_CommunicationChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_yaw_rate_SET((float) -2.2000601E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -2.0858724E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)2070898241L, PH.base.pack) ;
        p82_thrust_SET((float) -1.314141E36F, PH.base.pack) ;
        {
            float q[] =  {-9.777236E37F, -2.6039803E38F, -3.3574221E38F, -1.3705904E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_roll_rate_SET((float) -1.5351622E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_thrust_SET((float)9.998173E37F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)2033250346L, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -2.8035207E38F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)3.1390826E38F, PH.base.pack) ;
        {
            float q[] =  {-7.904438E37F, -8.821262E37F, -6.275489E37F, 7.5294207E37F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_yaw_rate_SET((float)4.239599E37F, PH.base.pack) ;
        c_CommunicationChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vx_SET((float)9.45532E37F, PH.base.pack) ;
        p84_y_SET((float)1.1264997E36F, PH.base.pack) ;
        p84_z_SET((float) -1.4436025E38F, PH.base.pack) ;
        p84_vz_SET((float)1.497382E38F, PH.base.pack) ;
        p84_yaw_SET((float) -1.0709296E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p84_afy_SET((float) -8.2111155E37F, PH.base.pack) ;
        p84_x_SET((float) -1.8201643E38F, PH.base.pack) ;
        p84_vy_SET((float) -3.3671682E38F, PH.base.pack) ;
        p84_afx_SET((float) -6.496121E37F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)18054, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.1079266E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)3511398888L, PH.base.pack) ;
        p84_afz_SET((float) -4.105676E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_yaw_rate_SET((float) -3.2392503E38F, PH.base.pack) ;
        p86_yaw_SET((float)6.723813E37F, PH.base.pack) ;
        p86_afz_SET((float)1.5777334E38F, PH.base.pack) ;
        p86_afy_SET((float) -2.5404909E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)1962799749, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3002894767L, PH.base.pack) ;
        p86_afx_SET((float) -2.3135305E38F, PH.base.pack) ;
        p86_alt_SET((float) -1.0835042E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p86_vx_SET((float) -1.544076E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)60122, PH.base.pack) ;
        p86_vy_SET((float)2.0226205E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p86_lon_int_SET((int32_t) -966399492, PH.base.pack) ;
        p86_vz_SET((float) -2.9870685E38F, PH.base.pack) ;
        c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_time_boot_ms_SET((uint32_t)1001772587L, PH.base.pack) ;
        p87_vy_SET((float) -1.6337131E38F, PH.base.pack) ;
        p87_yaw_SET((float) -1.3909719E38F, PH.base.pack) ;
        p87_vz_SET((float) -2.9371126E38F, PH.base.pack) ;
        p87_alt_SET((float)2.382105E37F, PH.base.pack) ;
        p87_afx_SET((float) -3.2138715E38F, PH.base.pack) ;
        p87_afy_SET((float) -2.3721803E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)53839, PH.base.pack) ;
        p87_lon_int_SET((int32_t) -1754130825, PH.base.pack) ;
        p87_vx_SET((float) -2.6443656E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -676265800, PH.base.pack) ;
        p87_yaw_rate_SET((float)6.187788E37F, PH.base.pack) ;
        p87_afz_SET((float)1.17171E37F, PH.base.pack) ;
        c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_x_SET((float)1.2166309E38F, PH.base.pack) ;
        p89_pitch_SET((float) -1.7910382E38F, PH.base.pack) ;
        p89_y_SET((float) -1.0917352E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2024965337L, PH.base.pack) ;
        p89_z_SET((float) -1.6120388E38F, PH.base.pack) ;
        p89_roll_SET((float)5.4484683E37F, PH.base.pack) ;
        p89_yaw_SET((float) -1.3802795E38F, PH.base.pack) ;
        c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
        p90_yaw_SET((float)6.295997E37F, PH.base.pack) ;
        p90_rollspeed_SET((float)3.298934E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)11000, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)14439, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -30814, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)18230, PH.base.pack) ;
        p90_yawspeed_SET((float) -1.0768847E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)10279, PH.base.pack) ;
        p90_alt_SET((int32_t) -184169428, PH.base.pack) ;
        p90_pitch_SET((float) -2.6244746E38F, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.7891363E38F, PH.base.pack) ;
        p90_roll_SET((float) -1.2666776E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -30525, PH.base.pack) ;
        p90_lon_SET((int32_t) -2056748264, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6716823869778920230L, PH.base.pack) ;
        p90_lat_SET((int32_t) -2088499972, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
        p91_time_usec_SET((uint64_t)8540019472069159366L, PH.base.pack) ;
        p91_roll_ailerons_SET((float)3.0765994E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)2.194926E38F, PH.base.pack) ;
        p91_aux3_SET((float) -5.274441E37F, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -1.6926099E38F, PH.base.pack) ;
        p91_aux2_SET((float)2.1515497E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p91_aux4_SET((float)3.0319442E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p91_aux1_SET((float)1.1346848E38F, PH.base.pack) ;
        p91_throttle_SET((float) -2.086259E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan12_raw_SET((uint16_t)(uint16_t)54943, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)1462258763290575984L, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)32616, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)37584, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)7289, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)31592, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)13188, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)55177, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)1096, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)64017, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)11173, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)53302, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)32008, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)8344546389011401049L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)4135797548745712459L, PH.base.pack) ;
        {
            float controls[] =  {2.5554183E38F, 1.5796311E37F, 3.1755946E38F, -2.316234E38F, -3.144315E38F, 2.3686692E38F, -5.880912E37F, 1.5459401E38F, -2.4586562E38F, 2.6649699E38F, -2.9820188E38F, 3.9155493E37F, 2.9556454E38F, -3.2946507E38F, 2.4246767E38F, -2.1724986E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_sensor_id_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -2.6449457E38F, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)5168, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float)4.90704E37F, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -1.1287986E38F, &PH) ;
        p100_ground_distance_SET((float)4.9035925E37F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)13037, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)6904346464935342242L, PH.base.pack) ;
        p100_flow_rate_y_SET((float)2.4640844E38F, &PH) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_roll_SET((float) -2.1555616E38F, PH.base.pack) ;
        p101_x_SET((float)1.0158182E37F, PH.base.pack) ;
        p101_z_SET((float)2.086941E38F, PH.base.pack) ;
        p101_yaw_SET((float)3.4803672E37F, PH.base.pack) ;
        p101_pitch_SET((float) -4.288903E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)2819395135687547625L, PH.base.pack) ;
        p101_y_SET((float) -3.3646487E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_yaw_SET((float)1.386923E38F, PH.base.pack) ;
        p102_x_SET((float) -7.2088773E36F, PH.base.pack) ;
        p102_roll_SET((float) -3.0252234E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)5612488769664453163L, PH.base.pack) ;
        p102_y_SET((float)1.8284464E38F, PH.base.pack) ;
        p102_z_SET((float)2.8532562E38F, PH.base.pack) ;
        p102_pitch_SET((float)1.2945842E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_usec_SET((uint64_t)1543944488958140563L, PH.base.pack) ;
        p103_x_SET((float) -1.4921893E38F, PH.base.pack) ;
        p103_z_SET((float) -1.8257888E38F, PH.base.pack) ;
        p103_y_SET((float) -3.240979E38F, PH.base.pack) ;
        c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float)2.5148332E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)8132488405968501057L, PH.base.pack) ;
        p104_y_SET((float) -2.4612653E38F, PH.base.pack) ;
        p104_yaw_SET((float) -6.0092495E37F, PH.base.pack) ;
        p104_pitch_SET((float)2.399117E38F, PH.base.pack) ;
        p104_roll_SET((float) -2.3077502E38F, PH.base.pack) ;
        p104_x_SET((float)2.8211985E37F, PH.base.pack) ;
        c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
        p105_pressure_alt_SET((float)3.0766527E38F, PH.base.pack) ;
        p105_ygyro_SET((float)9.18551E37F, PH.base.pack) ;
        p105_zmag_SET((float) -2.3750957E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)10105, PH.base.pack) ;
        p105_xgyro_SET((float)2.2974354E38F, PH.base.pack) ;
        p105_temperature_SET((float)8.12181E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -2.8829368E38F, PH.base.pack) ;
        p105_yacc_SET((float) -2.733123E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4818936541614807020L, PH.base.pack) ;
        p105_diff_pressure_SET((float) -3.2143908E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -5.499719E37F, PH.base.pack) ;
        p105_xacc_SET((float)2.9979271E38F, PH.base.pack) ;
        p105_zacc_SET((float)2.8924716E37F, PH.base.pack) ;
        p105_ymag_SET((float)1.2587687E38F, PH.base.pack) ;
        p105_xmag_SET((float) -2.4842913E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_usec_SET((uint64_t)2794753736358470316L, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)1309587125L, PH.base.pack) ;
        p106_integrated_x_SET((float) -4.462486E37F, PH.base.pack) ;
        p106_distance_SET((float)5.4539744E37F, PH.base.pack) ;
        p106_integrated_y_SET((float)1.9702888E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)3.2819079E38F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2430743697L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -7592, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)1.8037364E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -5.966226E37F, PH.base.pack) ;
        c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
        p107_xmag_SET((float) -1.9260785E38F, PH.base.pack) ;
        p107_yacc_SET((float) -2.684725E36F, PH.base.pack) ;
        p107_zgyro_SET((float) -1.0848417E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)3.0776287E38F, PH.base.pack) ;
        p107_ygyro_SET((float)6.889505E37F, PH.base.pack) ;
        p107_ymag_SET((float) -2.4542693E38F, PH.base.pack) ;
        p107_zmag_SET((float) -2.016031E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -2.9582725E36F, PH.base.pack) ;
        p107_temperature_SET((float) -2.7282173E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)1.8186287E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)3838243168774387163L, PH.base.pack) ;
        p107_xgyro_SET((float)1.9320502E38F, PH.base.pack) ;
        p107_xacc_SET((float)2.4662392E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)1466803618L, PH.base.pack) ;
        p107_zacc_SET((float)2.3466442E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
        p108_xgyro_SET((float) -1.453832E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.205983E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)2.6489195E36F, PH.base.pack) ;
        p108_q2_SET((float) -8.405625E37F, PH.base.pack) ;
        p108_q1_SET((float)4.7498194E37F, PH.base.pack) ;
        p108_yacc_SET((float) -1.378438E38F, PH.base.pack) ;
        p108_yaw_SET((float)2.0865239E38F, PH.base.pack) ;
        p108_alt_SET((float) -4.3811114E36F, PH.base.pack) ;
        p108_ve_SET((float) -1.1028674E37F, PH.base.pack) ;
        p108_q3_SET((float)2.8175362E38F, PH.base.pack) ;
        p108_zacc_SET((float)1.3480861E38F, PH.base.pack) ;
        p108_ygyro_SET((float)3.0464193E38F, PH.base.pack) ;
        p108_roll_SET((float)2.5953454E38F, PH.base.pack) ;
        p108_pitch_SET((float) -1.563403E38F, PH.base.pack) ;
        p108_lon_SET((float) -5.3640564E37F, PH.base.pack) ;
        p108_zgyro_SET((float)9.3399457E36F, PH.base.pack) ;
        p108_vd_SET((float)1.502585E38F, PH.base.pack) ;
        p108_lat_SET((float) -3.262706E38F, PH.base.pack) ;
        p108_vn_SET((float) -2.2065006E38F, PH.base.pack) ;
        p108_q4_SET((float)5.429458E37F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -4.1091658E36F, PH.base.pack) ;
        c_CommunicationChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
        p109_rxerrors_SET((uint16_t)(uint16_t)203, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)59272, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        c_CommunicationChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)17, (uint8_t)3, (uint8_t)205, (uint8_t)118, (uint8_t)161, (uint8_t)39, (uint8_t)154, (uint8_t)68, (uint8_t)96, (uint8_t)142, (uint8_t)97, (uint8_t)213, (uint8_t)197, (uint8_t)40, (uint8_t)143, (uint8_t)55, (uint8_t)70, (uint8_t)233, (uint8_t)106, (uint8_t)127, (uint8_t)110, (uint8_t)40, (uint8_t)175, (uint8_t)62, (uint8_t)57, (uint8_t)46, (uint8_t)254, (uint8_t)3, (uint8_t)200, (uint8_t)80, (uint8_t)207, (uint8_t)135, (uint8_t)69, (uint8_t)125, (uint8_t)93, (uint8_t)161, (uint8_t)115, (uint8_t)100, (uint8_t)33, (uint8_t)131, (uint8_t)183, (uint8_t)152, (uint8_t)145, (uint8_t)128, (uint8_t)178, (uint8_t)111, (uint8_t)17, (uint8_t)7, (uint8_t)51, (uint8_t)51, (uint8_t)74, (uint8_t)212, (uint8_t)217, (uint8_t)181, (uint8_t)6, (uint8_t)228, (uint8_t)174, (uint8_t)213, (uint8_t)188, (uint8_t)144, (uint8_t)11, (uint8_t)174, (uint8_t)110, (uint8_t)92, (uint8_t)59, (uint8_t)171, (uint8_t)182, (uint8_t)84, (uint8_t)17, (uint8_t)195, (uint8_t)245, (uint8_t)1, (uint8_t)164, (uint8_t)187, (uint8_t)247, (uint8_t)65, (uint8_t)29, (uint8_t)32, (uint8_t)120, (uint8_t)74, (uint8_t)173, (uint8_t)141, (uint8_t)206, (uint8_t)16, (uint8_t)44, (uint8_t)173, (uint8_t)134, (uint8_t)240, (uint8_t)35, (uint8_t)158, (uint8_t)122, (uint8_t)156, (uint8_t)7, (uint8_t)249, (uint8_t)165, (uint8_t)254, (uint8_t)66, (uint8_t)217, (uint8_t)227, (uint8_t)194, (uint8_t)53, (uint8_t)245, (uint8_t)221, (uint8_t)220, (uint8_t)88, (uint8_t)160, (uint8_t)229, (uint8_t)193, (uint8_t)101, (uint8_t)150, (uint8_t)224, (uint8_t)20, (uint8_t)96, (uint8_t)100, (uint8_t)198, (uint8_t)211, (uint8_t)72, (uint8_t)125, (uint8_t)183, (uint8_t)141, (uint8_t)174, (uint8_t)187, (uint8_t)19, (uint8_t)169, (uint8_t)130, (uint8_t)42, (uint8_t)101, (uint8_t)121, (uint8_t)124, (uint8_t)148, (uint8_t)35, (uint8_t)23, (uint8_t)98, (uint8_t)92, (uint8_t)212, (uint8_t)38, (uint8_t)127, (uint8_t)156, (uint8_t)160, (uint8_t)117, (uint8_t)218, (uint8_t)89, (uint8_t)138, (uint8_t)57, (uint8_t)50, (uint8_t)30, (uint8_t)123, (uint8_t)85, (uint8_t)57, (uint8_t)61, (uint8_t)81, (uint8_t)149, (uint8_t)114, (uint8_t)71, (uint8_t)206, (uint8_t)188, (uint8_t)245, (uint8_t)184, (uint8_t)35, (uint8_t)252, (uint8_t)105, (uint8_t)141, (uint8_t)233, (uint8_t)63, (uint8_t)32, (uint8_t)232, (uint8_t)30, (uint8_t)4, (uint8_t)243, (uint8_t)17, (uint8_t)159, (uint8_t)182, (uint8_t)51, (uint8_t)13, (uint8_t)234, (uint8_t)98, (uint8_t)128, (uint8_t)229, (uint8_t)75, (uint8_t)5, (uint8_t)166, (uint8_t)72, (uint8_t)138, (uint8_t)82, (uint8_t)213, (uint8_t)252, (uint8_t)96, (uint8_t)4, (uint8_t)45, (uint8_t)227, (uint8_t)237, (uint8_t)86, (uint8_t)40, (uint8_t)184, (uint8_t)167, (uint8_t)187, (uint8_t)123, (uint8_t)19, (uint8_t)151, (uint8_t)141, (uint8_t)55, (uint8_t)239, (uint8_t)113, (uint8_t)132, (uint8_t)247, (uint8_t)39, (uint8_t)130, (uint8_t)60, (uint8_t)238, (uint8_t)77, (uint8_t)161, (uint8_t)119, (uint8_t)56, (uint8_t)93, (uint8_t)64, (uint8_t)29, (uint8_t)15, (uint8_t)158, (uint8_t)187, (uint8_t)71, (uint8_t)197, (uint8_t)130, (uint8_t)87, (uint8_t)112, (uint8_t)100, (uint8_t)48, (uint8_t)211, (uint8_t)24, (uint8_t)51, (uint8_t)252, (uint8_t)129, (uint8_t)150, (uint8_t)217, (uint8_t)71, (uint8_t)17, (uint8_t)36, (uint8_t)146, (uint8_t)24, (uint8_t)105, (uint8_t)40, (uint8_t)170, (uint8_t)138, (uint8_t)40, (uint8_t)103, (uint8_t)35, (uint8_t)214, (uint8_t)95, (uint8_t)174, (uint8_t)243, (uint8_t)198, (uint8_t)45};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)3568525415637484616L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -5667210633751528184L, PH.base.pack) ;
        c_CommunicationChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)1419833907L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)3001557915414442564L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
        p113_eph_SET((uint16_t)(uint16_t)64017, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)46246, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)1408405687273781154L, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)17501, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p113_lon_SET((int32_t) -1874514058, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -14522, PH.base.pack) ;
        p113_lat_SET((int32_t)68216505, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)38668, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)16626, PH.base.pack) ;
        p113_alt_SET((int32_t) -1236455657, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)56507, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_ygyro_SET((float)1.951686E38F, PH.base.pack) ;
        p114_integrated_x_SET((float) -1.9721577E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)7473669562463883079L, PH.base.pack) ;
        p114_integrated_y_SET((float) -2.4725803E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)2.8159436E37F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2377926790L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.9333603E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)436279911L, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)11045, PH.base.pack) ;
        p114_distance_SET((float) -6.833924E37F, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_ind_airspeed_SET((uint16_t)(uint16_t)34125, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)6053786514724450452L, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -4376, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -21310, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)11392, PH.base.pack) ;
        p115_rollspeed_SET((float)2.220006E38F, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)21289, PH.base.pack) ;
        p115_lat_SET((int32_t)882053765, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {1.7051483E38F, 7.762479E36F, -2.3824203E38F, 4.499651E37F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_yawspeed_SET((float)2.8908267E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -2654, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.3216714E38F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)44064, PH.base.pack) ;
        p115_alt_SET((int32_t)1946422435, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -26726, PH.base.pack) ;
        p115_lon_SET((int32_t)1622026339, PH.base.pack) ;
        c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
        p116_xgyro_SET((int16_t)(int16_t) -16328, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -24915, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)11575, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)6334, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -16363, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t)7646, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -27175, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)952925679L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)31289, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)14229, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)62274, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)8323, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)4252422775L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)35469, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)46645, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)3232270520L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)56143, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2612820405L, PH.base.pack) ;
        p119_count_SET((uint32_t)2245987504L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)18610, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)1263689835L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)18025, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)236, (uint8_t)38, (uint8_t)249, (uint8_t)125, (uint8_t)163, (uint8_t)108, (uint8_t)168, (uint8_t)121, (uint8_t)23, (uint8_t)139, (uint8_t)62, (uint8_t)189, (uint8_t)62, (uint8_t)148, (uint8_t)10, (uint8_t)243, (uint8_t)20, (uint8_t)120, (uint8_t)199, (uint8_t)8, (uint8_t)241, (uint8_t)164, (uint8_t)229, (uint8_t)159, (uint8_t)246, (uint8_t)129, (uint8_t)128, (uint8_t)134, (uint8_t)134, (uint8_t)78, (uint8_t)168, (uint8_t)218, (uint8_t)177, (uint8_t)52, (uint8_t)248, (uint8_t)234, (uint8_t)201, (uint8_t)113, (uint8_t)59, (uint8_t)195, (uint8_t)22, (uint8_t)218, (uint8_t)21, (uint8_t)66, (uint8_t)126, (uint8_t)205, (uint8_t)106, (uint8_t)11, (uint8_t)151, (uint8_t)114, (uint8_t)121, (uint8_t)179, (uint8_t)49, (uint8_t)101, (uint8_t)46, (uint8_t)152, (uint8_t)17, (uint8_t)203, (uint8_t)223, (uint8_t)149, (uint8_t)21, (uint8_t)252, (uint8_t)136, (uint8_t)85, (uint8_t)46, (uint8_t)109, (uint8_t)245, (uint8_t)248, (uint8_t)197, (uint8_t)48, (uint8_t)105, (uint8_t)91, (uint8_t)82, (uint8_t)117, (uint8_t)53, (uint8_t)22, (uint8_t)86, (uint8_t)77, (uint8_t)74, (uint8_t)166, (uint8_t)195, (uint8_t)119, (uint8_t)108, (uint8_t)203, (uint8_t)36, (uint8_t)122, (uint8_t)255, (uint8_t)48, (uint8_t)10, (uint8_t)178};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        c_CommunicationChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)218, (uint8_t)75, (uint8_t)219, (uint8_t)180, (uint8_t)35, (uint8_t)7, (uint8_t)150, (uint8_t)119, (uint8_t)9, (uint8_t)214, (uint8_t)211, (uint8_t)57, (uint8_t)224, (uint8_t)199, (uint8_t)232, (uint8_t)202, (uint8_t)209, (uint8_t)155, (uint8_t)26, (uint8_t)45, (uint8_t)136, (uint8_t)77, (uint8_t)23, (uint8_t)212, (uint8_t)217, (uint8_t)21, (uint8_t)204, (uint8_t)235, (uint8_t)66, (uint8_t)199, (uint8_t)155, (uint8_t)168, (uint8_t)228, (uint8_t)31, (uint8_t)173, (uint8_t)63, (uint8_t)246, (uint8_t)113, (uint8_t)187, (uint8_t)252, (uint8_t)245, (uint8_t)128, (uint8_t)82, (uint8_t)129, (uint8_t)238, (uint8_t)236, (uint8_t)50, (uint8_t)244, (uint8_t)44, (uint8_t)138, (uint8_t)116, (uint8_t)246, (uint8_t)170, (uint8_t)195, (uint8_t)6, (uint8_t)107, (uint8_t)171, (uint8_t)120, (uint8_t)81, (uint8_t)244, (uint8_t)202, (uint8_t)112, (uint8_t)183, (uint8_t)13, (uint8_t)142, (uint8_t)191, (uint8_t)46, (uint8_t)153, (uint8_t)163, (uint8_t)51, (uint8_t)205, (uint8_t)35, (uint8_t)83, (uint8_t)5, (uint8_t)5, (uint8_t)101, (uint8_t)92, (uint8_t)246, (uint8_t)9, (uint8_t)152, (uint8_t)200, (uint8_t)59, (uint8_t)247, (uint8_t)172, (uint8_t)68, (uint8_t)115, (uint8_t)24, (uint8_t)243, (uint8_t)81, (uint8_t)243, (uint8_t)129, (uint8_t)83, (uint8_t)221, (uint8_t)155, (uint8_t)43, (uint8_t)173, (uint8_t)97, (uint8_t)143, (uint8_t)230, (uint8_t)32, (uint8_t)6, (uint8_t)29, (uint8_t)47, (uint8_t)79, (uint8_t)154, (uint8_t)125, (uint8_t)5, (uint8_t)61, (uint8_t)44, (uint8_t)213};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
        p124_cog_SET((uint16_t)(uint16_t)41047, PH.base.pack) ;
        p124_alt_SET((int32_t) -1746422963, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)14342, PH.base.pack) ;
        p124_lat_SET((int32_t)2023476196, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
        p124_lon_SET((int32_t)1728716117, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)41539, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)4814211722413217027L, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)2495, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1604588518L, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
        p125_Vservo_SET((uint16_t)(uint16_t)53108, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)36011, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, PH.base.pack) ;
        c_CommunicationChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)33841, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1338472155L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)130, (uint8_t)214, (uint8_t)185, (uint8_t)78, (uint8_t)146, (uint8_t)17, (uint8_t)214, (uint8_t)48, (uint8_t)185, (uint8_t)127, (uint8_t)179, (uint8_t)38, (uint8_t)7, (uint8_t)48, (uint8_t)64, (uint8_t)218, (uint8_t)123, (uint8_t)27, (uint8_t)36, (uint8_t)220, (uint8_t)36, (uint8_t)79, (uint8_t)92, (uint8_t)5, (uint8_t)203, (uint8_t)60, (uint8_t)223, (uint8_t)35, (uint8_t)180, (uint8_t)109, (uint8_t)90, (uint8_t)89, (uint8_t)150, (uint8_t)34, (uint8_t)82, (uint8_t)172, (uint8_t)108, (uint8_t)113, (uint8_t)98, (uint8_t)128, (uint8_t)74, (uint8_t)238, (uint8_t)197, (uint8_t)114, (uint8_t)82, (uint8_t)12, (uint8_t)73, (uint8_t)104, (uint8_t)52, (uint8_t)94, (uint8_t)176, (uint8_t)206, (uint8_t)70, (uint8_t)191, (uint8_t)140, (uint8_t)169, (uint8_t)60, (uint8_t)97, (uint8_t)149, (uint8_t)34, (uint8_t)44, (uint8_t)136, (uint8_t)30, (uint8_t)46, (uint8_t)169, (uint8_t)194, (uint8_t)191, (uint8_t)90, (uint8_t)38, (uint8_t)24};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
        p127_time_last_baseline_ms_SET((uint32_t)1941885079L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1136902089, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1038492860, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)31749, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)1978968367L, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p127_tow_SET((uint32_t)943052820L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)1753346544, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)285805169, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_health_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1081456821, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p128_tow_SET((uint32_t)385377152L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)52533, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)509406860, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)136909085L, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)1050784417L, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)1979860776, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)11566174, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        c_CommunicationChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
        p129_ymag_SET((int16_t)(int16_t) -26184, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -9208, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -30588, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)20858, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)32415, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t) -17723, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t) -28441, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -5832, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)4127783241L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -19338, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_type_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p130_size_SET((uint32_t)2459682499L, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)21412, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)38797, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)62686, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)95, (uint8_t)62, (uint8_t)18, (uint8_t)174, (uint8_t)25, (uint8_t)140, (uint8_t)133, (uint8_t)180, (uint8_t)52, (uint8_t)166, (uint8_t)237, (uint8_t)180, (uint8_t)78, (uint8_t)113, (uint8_t)146, (uint8_t)198, (uint8_t)12, (uint8_t)168, (uint8_t)229, (uint8_t)174, (uint8_t)124, (uint8_t)151, (uint8_t)243, (uint8_t)11, (uint8_t)249, (uint8_t)29, (uint8_t)88, (uint8_t)5, (uint8_t)254, (uint8_t)18, (uint8_t)175, (uint8_t)21, (uint8_t)195, (uint8_t)180, (uint8_t)103, (uint8_t)140, (uint8_t)230, (uint8_t)217, (uint8_t)126, (uint8_t)151, (uint8_t)204, (uint8_t)93, (uint8_t)27, (uint8_t)35, (uint8_t)89, (uint8_t)7, (uint8_t)139, (uint8_t)173, (uint8_t)138, (uint8_t)231, (uint8_t)125, (uint8_t)176, (uint8_t)70, (uint8_t)113, (uint8_t)173, (uint8_t)171, (uint8_t)232, (uint8_t)33, (uint8_t)95, (uint8_t)152, (uint8_t)122, (uint8_t)101, (uint8_t)11, (uint8_t)126, (uint8_t)48, (uint8_t)119, (uint8_t)207, (uint8_t)145, (uint8_t)123, (uint8_t)30, (uint8_t)212, (uint8_t)131, (uint8_t)185, (uint8_t)254, (uint8_t)59, (uint8_t)106, (uint8_t)185, (uint8_t)93, (uint8_t)92, (uint8_t)138, (uint8_t)21, (uint8_t)220, (uint8_t)5, (uint8_t)252, (uint8_t)87, (uint8_t)28, (uint8_t)194, (uint8_t)80, (uint8_t)249, (uint8_t)242, (uint8_t)24, (uint8_t)20, (uint8_t)132, (uint8_t)148, (uint8_t)216, (uint8_t)11, (uint8_t)72, (uint8_t)199, (uint8_t)109, (uint8_t)234, (uint8_t)122, (uint8_t)167, (uint8_t)162, (uint8_t)152, (uint8_t)227, (uint8_t)151, (uint8_t)95, (uint8_t)121, (uint8_t)167, (uint8_t)121, (uint8_t)105, (uint8_t)133, (uint8_t)22, (uint8_t)219, (uint8_t)156, (uint8_t)249, (uint8_t)254, (uint8_t)140, (uint8_t)248, (uint8_t)194, (uint8_t)95, (uint8_t)53, (uint8_t)115, (uint8_t)249, (uint8_t)150, (uint8_t)128, (uint8_t)99, (uint8_t)168, (uint8_t)85, (uint8_t)190, (uint8_t)168, (uint8_t)118, (uint8_t)21, (uint8_t)104, (uint8_t)164, (uint8_t)93, (uint8_t)79, (uint8_t)163, (uint8_t)188, (uint8_t)100, (uint8_t)56, (uint8_t)87, (uint8_t)13, (uint8_t)151, (uint8_t)114, (uint8_t)101, (uint8_t)182, (uint8_t)26, (uint8_t)17, (uint8_t)227, (uint8_t)122, (uint8_t)67, (uint8_t)8, (uint8_t)51, (uint8_t)83, (uint8_t)225, (uint8_t)213, (uint8_t)66, (uint8_t)144, (uint8_t)87, (uint8_t)192, (uint8_t)21, (uint8_t)234, (uint8_t)252, (uint8_t)62, (uint8_t)46, (uint8_t)248, (uint8_t)188, (uint8_t)216, (uint8_t)70, (uint8_t)241, (uint8_t)121, (uint8_t)161, (uint8_t)43, (uint8_t)91, (uint8_t)50, (uint8_t)45, (uint8_t)220, (uint8_t)100, (uint8_t)63, (uint8_t)252, (uint8_t)183, (uint8_t)70, (uint8_t)122, (uint8_t)222, (uint8_t)98, (uint8_t)105, (uint8_t)103, (uint8_t)251, (uint8_t)168, (uint8_t)210, (uint8_t)229, (uint8_t)30, (uint8_t)201, (uint8_t)28, (uint8_t)19, (uint8_t)61, (uint8_t)71, (uint8_t)133, (uint8_t)149, (uint8_t)191, (uint8_t)141, (uint8_t)8, (uint8_t)106, (uint8_t)246, (uint8_t)221, (uint8_t)111, (uint8_t)100, (uint8_t)100, (uint8_t)247, (uint8_t)35, (uint8_t)116, (uint8_t)230, (uint8_t)62, (uint8_t)131, (uint8_t)18, (uint8_t)127, (uint8_t)206, (uint8_t)114, (uint8_t)220, (uint8_t)57, (uint8_t)102, (uint8_t)12, (uint8_t)242, (uint8_t)30, (uint8_t)132, (uint8_t)107, (uint8_t)24, (uint8_t)138, (uint8_t)139, (uint8_t)209, (uint8_t)224, (uint8_t)14, (uint8_t)43, (uint8_t)178, (uint8_t)241, (uint8_t)253, (uint8_t)187, (uint8_t)105, (uint8_t)35, (uint8_t)247, (uint8_t)77, (uint8_t)56, (uint8_t)0, (uint8_t)11, (uint8_t)48, (uint8_t)96, (uint8_t)107, (uint8_t)66, (uint8_t)23, (uint8_t)22, (uint8_t)180, (uint8_t)31};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)44800, PH.base.pack) ;
        c_CommunicationChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)2023773760L, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)46663, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)26920, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)26322, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_270, PH.base.pack) ;
        c_CommunicationChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t)296669069, PH.base.pack) ;
        p133_mask_SET((uint64_t)8106080634754672482L, PH.base.pack) ;
        p133_lon_SET((int32_t)1577526344, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)56038, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)37331, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p134_lat_SET((int32_t)1522265006, PH.base.pack) ;
        p134_lon_SET((int32_t) -982298845, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)13560, (int16_t) -31927, (int16_t)15489, (int16_t) -2833, (int16_t)26221, (int16_t)22117, (int16_t) -16031, (int16_t)23544, (int16_t) -31335, (int16_t) -24443, (int16_t) -17365, (int16_t) -6413, (int16_t)1880, (int16_t) -25847, (int16_t) -25205, (int16_t)27272};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -82111113, PH.base.pack) ;
        p135_lat_SET((int32_t)599081529, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)7329, PH.base.pack) ;
        p136_lon_SET((int32_t) -631927872, PH.base.pack) ;
        p136_terrain_height_SET((float) -1.6061361E38F, PH.base.pack) ;
        p136_lat_SET((int32_t)1777913876, PH.base.pack) ;
        p136_current_height_SET((float) -3.3936992E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)50265, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)40093, PH.base.pack) ;
        c_CommunicationChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_diff_SET((float)6.458087E37F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)3415711517L, PH.base.pack) ;
        p137_press_abs_SET((float) -3.1501886E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)9865, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_y_SET((float) -3.283186E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)4451140005938754895L, PH.base.pack) ;
        {
            float q[] =  {1.5490164E38F, -1.5164821E38F, -1.852958E38F, 9.522645E36F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)2.2214597E38F, PH.base.pack) ;
        p138_z_SET((float)1.7456341E36F, PH.base.pack) ;
        c_CommunicationChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        {
            float controls[] =  {-3.2098457E38F, -1.4387232E37F, -1.7950539E38F, -2.3428621E38F, 1.894254E38F, -1.1545533E38F, 1.4783821E38F, 8.516883E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_time_usec_SET((uint64_t)4359677886207123806L, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_group_mlx_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)7637525349737860074L, PH.base.pack) ;
        {
            float controls[] =  {-1.5229614E38F, -3.3273897E38F, 1.4001146E38F, -8.527506E37F, -3.0628039E38F, 1.4643756E38F, 2.1201748E38F, 2.768402E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
        p141_time_usec_SET((uint64_t)7864052235922642545L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)9.385611E37F, PH.base.pack) ;
        p141_altitude_local_SET((float) -5.644832E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float)1.5514764E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)1.7449557E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -3.2138814E37F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)2.1620055E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)236, (uint8_t)176, (uint8_t)228, (uint8_t)242, (uint8_t)161, (uint8_t)221, (uint8_t)67, (uint8_t)53, (uint8_t)223, (uint8_t)128, (uint8_t)150, (uint8_t)45, (uint8_t)252, (uint8_t)221, (uint8_t)18, (uint8_t)138, (uint8_t)1, (uint8_t)219, (uint8_t)37, (uint8_t)87, (uint8_t)201, (uint8_t)184, (uint8_t)29, (uint8_t)145, (uint8_t)28, (uint8_t)22, (uint8_t)41, (uint8_t)72, (uint8_t)208, (uint8_t)6, (uint8_t)192, (uint8_t)52, (uint8_t)170, (uint8_t)6, (uint8_t)247, (uint8_t)39, (uint8_t)168, (uint8_t)209, (uint8_t)2, (uint8_t)220, (uint8_t)155, (uint8_t)216, (uint8_t)195, (uint8_t)9, (uint8_t)79, (uint8_t)65, (uint8_t)205, (uint8_t)161, (uint8_t)9, (uint8_t)11, (uint8_t)189, (uint8_t)160, (uint8_t)80, (uint8_t)149, (uint8_t)92, (uint8_t)201, (uint8_t)25, (uint8_t)1, (uint8_t)204, (uint8_t)234, (uint8_t)31, (uint8_t)172, (uint8_t)94, (uint8_t)129, (uint8_t)116, (uint8_t)114, (uint8_t)57, (uint8_t)167, (uint8_t)152, (uint8_t)24, (uint8_t)73, (uint8_t)166, (uint8_t)242, (uint8_t)98, (uint8_t)23, (uint8_t)161, (uint8_t)39, (uint8_t)76, (uint8_t)234, (uint8_t)237, (uint8_t)69, (uint8_t)54, (uint8_t)114, (uint8_t)189, (uint8_t)154, (uint8_t)48, (uint8_t)91, (uint8_t)91, (uint8_t)110, (uint8_t)6, (uint8_t)192, (uint8_t)30, (uint8_t)97, (uint8_t)227, (uint8_t)140, (uint8_t)185, (uint8_t)164, (uint8_t)72, (uint8_t)89, (uint8_t)241, (uint8_t)12, (uint8_t)19, (uint8_t)61, (uint8_t)239, (uint8_t)149, (uint8_t)80, (uint8_t)199, (uint8_t)173, (uint8_t)223, (uint8_t)191, (uint8_t)28, (uint8_t)150, (uint8_t)44, (uint8_t)50, (uint8_t)197, (uint8_t)56, (uint8_t)156, (uint8_t)197, (uint8_t)73, (uint8_t)208};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)43, (uint8_t)26, (uint8_t)165, (uint8_t)52, (uint8_t)91, (uint8_t)92, (uint8_t)168, (uint8_t)8, (uint8_t)162, (uint8_t)194, (uint8_t)123, (uint8_t)169, (uint8_t)68, (uint8_t)151, (uint8_t)128, (uint8_t)170, (uint8_t)145, (uint8_t)129, (uint8_t)194, (uint8_t)92, (uint8_t)195, (uint8_t)45, (uint8_t)39, (uint8_t)44, (uint8_t)236, (uint8_t)20, (uint8_t)235, (uint8_t)14, (uint8_t)163, (uint8_t)68, (uint8_t)229, (uint8_t)243, (uint8_t)97, (uint8_t)110, (uint8_t)63, (uint8_t)189, (uint8_t)19, (uint8_t)26, (uint8_t)51, (uint8_t)214, (uint8_t)253, (uint8_t)34, (uint8_t)252, (uint8_t)98, (uint8_t)168, (uint8_t)173, (uint8_t)7, (uint8_t)128, (uint8_t)112, (uint8_t)225, (uint8_t)31, (uint8_t)192, (uint8_t)81, (uint8_t)175, (uint8_t)108, (uint8_t)238, (uint8_t)57, (uint8_t)62, (uint8_t)152, (uint8_t)83, (uint8_t)50, (uint8_t)184, (uint8_t)226, (uint8_t)235, (uint8_t)138, (uint8_t)130, (uint8_t)214, (uint8_t)0, (uint8_t)62, (uint8_t)108, (uint8_t)33, (uint8_t)247, (uint8_t)242, (uint8_t)54, (uint8_t)140, (uint8_t)155, (uint8_t)160, (uint8_t)114, (uint8_t)44, (uint8_t)171, (uint8_t)217, (uint8_t)48, (uint8_t)249, (uint8_t)235, (uint8_t)87, (uint8_t)212, (uint8_t)206, (uint8_t)192, (uint8_t)122, (uint8_t)133, (uint8_t)254, (uint8_t)31, (uint8_t)37, (uint8_t)187, (uint8_t)253, (uint8_t)167, (uint8_t)133, (uint8_t)134, (uint8_t)103, (uint8_t)120, (uint8_t)124, (uint8_t)207, (uint8_t)188, (uint8_t)105, (uint8_t)181, (uint8_t)13, (uint8_t)223, (uint8_t)219, (uint8_t)82, (uint8_t)78, (uint8_t)130, (uint8_t)151, (uint8_t)56, (uint8_t)90, (uint8_t)167, (uint8_t)149, (uint8_t)36, (uint8_t)62, (uint8_t)29, (uint8_t)155};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        c_CommunicationChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_time_boot_ms_SET((uint32_t)2359553157L, PH.base.pack) ;
        p143_press_abs_SET((float)4.9160266E37F, PH.base.pack) ;
        p143_press_diff_SET((float) -3.5158677E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -22174, PH.base.pack) ;
        c_CommunicationChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
        p144_lon_SET((int32_t)1858418652, PH.base.pack) ;
        {
            float vel[] =  {-1.7202179E38F, 2.0193544E38F, -3.0448513E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_alt_SET((float)2.1144631E38F, PH.base.pack) ;
        p144_lat_SET((int32_t)1596419731, PH.base.pack) ;
        {
            float rates[] =  {-2.6875495E38F, -3.281374E38F, 7.872014E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)4209062715672247912L, PH.base.pack) ;
        {
            float position_cov[] =  {1.1018877E37F, 6.367287E37F, -2.6419368E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float acc[] =  {8.951343E37F, 3.117916E38F, -1.8079976E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        {
            float attitude_q[] =  {-3.5671176E37F, -3.018196E37F, -6.0053507E37F, -2.6931308E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)8956847255645276457L, PH.base.pack) ;
        c_CommunicationChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_vel_SET((float) -2.6246418E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -3.1939538E38F, PH.base.pack) ;
        p146_airspeed_SET((float)2.8122687E38F, PH.base.pack) ;
        p146_z_acc_SET((float)1.3430503E37F, PH.base.pack) ;
        p146_z_vel_SET((float) -3.0920475E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float) -2.618427E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -2.3547214E38F, PH.base.pack) ;
        {
            float q[] =  {3.0479594E38F, 1.3256498E38F, -3.0771192E38F, -2.542414E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_time_usec_SET((uint64_t)2918029663234418531L, PH.base.pack) ;
        p146_roll_rate_SET((float)1.3036767E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)7.707957E37F, PH.base.pack) ;
        p146_y_acc_SET((float) -1.4965751E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {1.1046906E38F, 3.1029176E38F, -2.108547E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_y_pos_SET((float) -9.717474E37F, PH.base.pack) ;
        p146_x_pos_SET((float) -4.472403E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {-3.3591755E38F, -2.5037318E38F, 2.1179478E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_y_vel_SET((float)2.116511E38F, PH.base.pack) ;
        c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_CommunicationChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_CommunicationChannel_input_bytes(buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BATTERY_STATUS_147(), &PH);
        {
            uint16_t voltages[] =  {(uint16_t)58934, (uint16_t)124, (uint16_t)62917, (uint16_t)24663, (uint16_t)54367, (uint16_t)35736, (uint16_t)55552, (uint16_t)15135, (uint16_t)51385, (uint16_t)51733};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -23, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -8141, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -21742, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)1309377689, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)347988419, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        c_CommunicationChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)119, (uint8_t)116, (uint8_t)251, (uint8_t)4, (uint8_t)222, (uint8_t)84, (uint8_t)166, (uint8_t)34};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)100, (uint8_t)192, (uint8_t)136, (uint8_t)83, (uint8_t)106, (uint8_t)39, (uint8_t)192, (uint8_t)135, (uint8_t)46, (uint8_t)107, (uint8_t)162, (uint8_t)231, (uint8_t)189, (uint8_t)150, (uint8_t)190, (uint8_t)48, (uint8_t)121, (uint8_t)181};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_board_version_SET((uint32_t)2723965636L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)3598887045L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)125, (uint8_t)50, (uint8_t)30, (uint8_t)48, (uint8_t)86, (uint8_t)72, (uint8_t)251, (uint8_t)204};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)44830, PH.base.pack) ;
        p148_uid_SET((uint64_t)9151205780219602572L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)3309572193L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)1074952307L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)235, (uint8_t)86, (uint8_t)165, (uint8_t)217, (uint8_t)145, (uint8_t)19, (uint8_t)111, (uint8_t)211};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)30218, PH.base.pack) ;
        c_CommunicationChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LANDING_TARGET_149(), &PH);
        p149_position_valid_SET((uint8_t)(uint8_t)145, &PH) ;
        {
            float q[] =  {2.4998983E38F, 1.1361951E38F, -8.745379E36F, 2.7987099E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_y_SET((float) -2.5083864E38F, &PH) ;
        p149_distance_SET((float)1.3688232E38F, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)3644853406791709879L, PH.base.pack) ;
        p149_size_y_SET((float)2.1862681E38F, PH.base.pack) ;
        p149_angle_x_SET((float) -1.5743556E38F, PH.base.pack) ;
        p149_angle_y_SET((float) -2.2380221E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p149_size_x_SET((float)1.0114478E38F, PH.base.pack) ;
        p149_z_SET((float) -7.4776944E37F, &PH) ;
        p149_x_SET((float)2.434688E37F, &PH) ;
        c_CommunicationChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_POWER_201(), &PH);
        p201_adc121_cs1_amp_SET((float)8.763983E37F, PH.base.pack) ;
        p201_adc121_cspb_amp_SET((float) -3.158572E38F, PH.base.pack) ;
        p201_adc121_vspb_volt_SET((float) -1.0743103E38F, PH.base.pack) ;
        p201_adc121_cs2_amp_SET((float)7.8663394E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_POWER_201(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_MPPT_202(), &PH);
        p202_mppt2_pwm_SET((uint16_t)(uint16_t)38458, PH.base.pack) ;
        p202_mppt2_amp_SET((float)9.45335E37F, PH.base.pack) ;
        p202_mppt3_status_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p202_mppt2_status_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p202_mppt2_volt_SET((float) -2.4080094E38F, PH.base.pack) ;
        p202_mppt1_amp_SET((float) -2.967833E38F, PH.base.pack) ;
        p202_mppt3_amp_SET((float) -1.5907607E38F, PH.base.pack) ;
        p202_mppt3_pwm_SET((uint16_t)(uint16_t)61673, PH.base.pack) ;
        p202_mppt1_status_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p202_mppt_timestamp_SET((uint64_t)1589611839310857974L, PH.base.pack) ;
        p202_mppt1_volt_SET((float)3.2195344E38F, PH.base.pack) ;
        p202_mppt3_volt_SET((float) -1.144486E38F, PH.base.pack) ;
        p202_mppt1_pwm_SET((uint16_t)(uint16_t)23457, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_MPPT_202(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLCTRL_DATA_203(), &PH);
        p203_uThrot2_SET((float) -2.070046E38F, PH.base.pack) ;
        p203_AirspeedRef_SET((float)2.2915147E38F, PH.base.pack) ;
        p203_pRef_SET((float) -3.0680389E35F, PH.base.pack) ;
        p203_YawAngleRef_SET((float)1.4427437E38F, PH.base.pack) ;
        p203_uThrot_SET((float) -1.3882778E37F, PH.base.pack) ;
        p203_h_SET((float)2.3624464E38F, PH.base.pack) ;
        p203_p_SET((float) -2.686208E38F, PH.base.pack) ;
        p203_hRef_SET((float) -2.430506E38F, PH.base.pack) ;
        p203_YawAngle_SET((float) -7.6713194E37F, PH.base.pack) ;
        p203_RollAngleRef_SET((float)2.3801199E38F, PH.base.pack) ;
        p203_uRud_SET((float) -2.0790656E38F, PH.base.pack) ;
        p203_PitchAngleRef_SET((float) -3.0928588E37F, PH.base.pack) ;
        p203_hRef_t_SET((float)2.9623236E38F, PH.base.pack) ;
        p203_qRef_SET((float)2.0289322E38F, PH.base.pack) ;
        p203_PitchAngle_SET((float)2.7899628E37F, PH.base.pack) ;
        p203_rRef_SET((float)1.1557756E38F, PH.base.pack) ;
        p203_RollAngle_SET((float)6.726057E36F, PH.base.pack) ;
        p203_SpoilersEngaged_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p203_r_SET((float) -8.846739E37F, PH.base.pack) ;
        p203_nZ_SET((float) -3.367486E38F, PH.base.pack) ;
        p203_q_SET((float)2.969788E38F, PH.base.pack) ;
        p203_aslctrl_mode_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p203_uElev_SET((float)2.768959E38F, PH.base.pack) ;
        p203_uAil_SET((float)1.7985086E38F, PH.base.pack) ;
        p203_timestamp_SET((uint64_t)5166557155616542377L, PH.base.pack) ;
        c_CommunicationChannel_on_ASLCTRL_DATA_203(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLCTRL_DEBUG_204(), &PH);
        p204_f_7_SET((float) -3.9735043E37F, PH.base.pack) ;
        p204_i8_1_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p204_f_4_SET((float)1.948133E38F, PH.base.pack) ;
        p204_f_8_SET((float)2.188823E38F, PH.base.pack) ;
        p204_f_5_SET((float)3.1010752E38F, PH.base.pack) ;
        p204_i8_2_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p204_i32_1_SET((uint32_t)2703423410L, PH.base.pack) ;
        p204_f_6_SET((float)3.371732E38F, PH.base.pack) ;
        p204_f_2_SET((float)6.340419E37F, PH.base.pack) ;
        p204_f_1_SET((float) -1.4729333E38F, PH.base.pack) ;
        p204_f_3_SET((float) -1.0303898E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ASLCTRL_DEBUG_204(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASLUAV_STATUS_205(), &PH);
        p205_Motor_rpm_SET((float) -1.9063204E38F, PH.base.pack) ;
        p205_SATCOM_status_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        {
            uint8_t Servo_status[] =  {(uint8_t)145, (uint8_t)8, (uint8_t)77, (uint8_t)235, (uint8_t)231, (uint8_t)6, (uint8_t)228, (uint8_t)125};
            p205_Servo_status_SET(&Servo_status, 0, PH.base.pack) ;
        }
        p205_LED_status_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_CommunicationChannel_on_ASLUAV_STATUS_205(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EKF_EXT_206(), &PH);
        p206_alpha_SET((float) -1.8216705E38F, PH.base.pack) ;
        p206_WindZ_SET((float)1.2600393E38F, PH.base.pack) ;
        p206_Airspeed_SET((float) -1.3611187E38F, PH.base.pack) ;
        p206_beta_SET((float) -2.21007E38F, PH.base.pack) ;
        p206_timestamp_SET((uint64_t)7561150924485705285L, PH.base.pack) ;
        p206_WindDir_SET((float)5.692203E37F, PH.base.pack) ;
        p206_Windspeed_SET((float) -1.5652006E38F, PH.base.pack) ;
        c_CommunicationChannel_on_EKF_EXT_206(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ASL_OBCTRL_207(), &PH);
        p207_uAilL_SET((float) -2.1196035E38F, PH.base.pack) ;
        p207_obctrl_status_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p207_timestamp_SET((uint64_t)5348039869466635868L, PH.base.pack) ;
        p207_uThrot2_SET((float)7.2236537E37F, PH.base.pack) ;
        p207_uElev_SET((float) -1.9815245E38F, PH.base.pack) ;
        p207_uThrot_SET((float)1.7286371E38F, PH.base.pack) ;
        p207_uAilR_SET((float) -4.306409E37F, PH.base.pack) ;
        p207_uRud_SET((float)1.999115E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ASL_OBCTRL_207(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_ATMOS_208(), &PH);
        p208_Humidity_SET((float)7.8395625E37F, PH.base.pack) ;
        p208_TempAmbient_SET((float)6.8565877E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_ATMOS_208(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_BATMON_209(), &PH);
        p209_cellvoltage1_SET((uint16_t)(uint16_t)10338, PH.base.pack) ;
        p209_current_SET((int16_t)(int16_t)4271, PH.base.pack) ;
        p209_temperature_SET((float) -9.828813E37F, PH.base.pack) ;
        p209_voltage_SET((uint16_t)(uint16_t)9601, PH.base.pack) ;
        p209_cellvoltage4_SET((uint16_t)(uint16_t)2231, PH.base.pack) ;
        p209_cellvoltage6_SET((uint16_t)(uint16_t)17682, PH.base.pack) ;
        p209_cellvoltage2_SET((uint16_t)(uint16_t)20507, PH.base.pack) ;
        p209_SoC_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p209_serialnumber_SET((uint16_t)(uint16_t)38541, PH.base.pack) ;
        p209_hostfetcontrol_SET((uint16_t)(uint16_t)29623, PH.base.pack) ;
        p209_batterystatus_SET((uint16_t)(uint16_t)15713, PH.base.pack) ;
        p209_cellvoltage5_SET((uint16_t)(uint16_t)53514, PH.base.pack) ;
        p209_cellvoltage3_SET((uint16_t)(uint16_t)2145, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_BATMON_209(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FW_SOARING_DATA_210(), &PH);
        p210_VarW_SET((float)1.7280131E37F, PH.base.pack) ;
        p210_xW_SET((float)4.1702973E36F, PH.base.pack) ;
        p210_LoiterRadius_SET((float)6.6588936E37F, PH.base.pack) ;
        p210_z2_exp_SET((float)1.9987678E38F, PH.base.pack) ;
        p210_DebugVar2_SET((float)9.136628E37F, PH.base.pack) ;
        p210_DebugVar1_SET((float) -2.51025E38F, PH.base.pack) ;
        p210_ThermalGSNorth_SET((float)2.392028E38F, PH.base.pack) ;
        p210_VarR_SET((float)6.396188E37F, PH.base.pack) ;
        p210_VarLat_SET((float) -6.742835E37F, PH.base.pack) ;
        p210_vSinkExp_SET((float)1.6874205E38F, PH.base.pack) ;
        p210_z1_exp_SET((float) -1.5488301E38F, PH.base.pack) ;
        p210_xLon_SET((float)1.797705E38F, PH.base.pack) ;
        p210_xLat_SET((float)1.0341247E38F, PH.base.pack) ;
        p210_timestampModeChanged_SET((uint64_t)1828941308768555541L, PH.base.pack) ;
        p210_z2_DeltaRoll_SET((float)1.026046E38F, PH.base.pack) ;
        p210_valid_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p210_DistToSoarPoint_SET((float) -2.0443705E38F, PH.base.pack) ;
        p210_z1_LocalUpdraftSpeed_SET((float) -1.0587598E38F, PH.base.pack) ;
        p210_ThermalGSEast_SET((float)2.541719E38F, PH.base.pack) ;
        p210_ControlMode_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p210_LoiterDirection_SET((float) -1.6534109E38F, PH.base.pack) ;
        p210_TSE_dot_SET((float) -1.9331968E38F, PH.base.pack) ;
        p210_xR_SET((float) -2.5286666E38F, PH.base.pack) ;
        p210_timestamp_SET((uint64_t)3820723982267108933L, PH.base.pack) ;
        p210_VarLon_SET((float)5.2604833E37F, PH.base.pack) ;
        c_CommunicationChannel_on_FW_SOARING_DATA_210(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENSORPOD_STATUS_211(), &PH);
        p211_cpu_temp_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p211_visensor_rate_3_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p211_visensor_rate_4_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p211_visensor_rate_1_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p211_recording_nodes_count_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p211_free_space_SET((uint16_t)(uint16_t)20030, PH.base.pack) ;
        p211_timestamp_SET((uint64_t)8729322884687610528L, PH.base.pack) ;
        p211_visensor_rate_2_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_CommunicationChannel_on_SENSORPOD_STATUS_211(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SENS_POWER_BOARD_212(), &PH);
        p212_pwr_brd_servo_2_amp_SET((float)2.4520673E38F, PH.base.pack) ;
        p212_pwr_brd_mot_r_amp_SET((float)1.6861242E38F, PH.base.pack) ;
        p212_pwr_brd_servo_4_amp_SET((float)2.6488594E38F, PH.base.pack) ;
        p212_pwr_brd_mot_l_amp_SET((float)2.7112337E38F, PH.base.pack) ;
        p212_timestamp_SET((uint64_t)893739549085753316L, PH.base.pack) ;
        p212_pwr_brd_led_status_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p212_pwr_brd_aux_amp_SET((float) -5.673672E37F, PH.base.pack) ;
        p212_pwr_brd_servo_volt_SET((float)2.9115186E38F, PH.base.pack) ;
        p212_pwr_brd_status_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p212_pwr_brd_servo_3_amp_SET((float) -3.214206E37F, PH.base.pack) ;
        p212_pwr_brd_servo_1_amp_SET((float)3.4008717E38F, PH.base.pack) ;
        p212_pwr_brd_system_volt_SET((float)4.8474523E37F, PH.base.pack) ;
        c_CommunicationChannel_on_SENS_POWER_BOARD_212(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_time_usec_SET((uint64_t)7120263297860276450L, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -8.47303E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -2.9719354E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float)1.4045236E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)8.935133E37F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.4845516E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -6.722603E36F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.074521E37F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.0402149E38F, PH.base.pack) ;
        c_CommunicationChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIND_COV_231(), &PH);
        p231_time_usec_SET((uint64_t)2282892920318562949L, PH.base.pack) ;
        p231_wind_alt_SET((float)2.6603463E38F, PH.base.pack) ;
        p231_var_vert_SET((float)3.3730613E38F, PH.base.pack) ;
        p231_wind_y_SET((float) -1.3888808E38F, PH.base.pack) ;
        p231_wind_x_SET((float)2.0203284E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -2.874331E36F, PH.base.pack) ;
        p231_wind_z_SET((float)2.4464235E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -3.0613247E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -7.518901E37F, PH.base.pack) ;
        c_CommunicationChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_INPUT_232(), &PH);
        p232_time_week_ms_SET((uint32_t)476664284L, PH.base.pack) ;
        p232_lat_SET((int32_t)139581998, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, PH.base.pack) ;
        p232_lon_SET((int32_t) -766692634, PH.base.pack) ;
        p232_vert_accuracy_SET((float)1.8346529E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -7.500916E37F, PH.base.pack) ;
        p232_ve_SET((float) -2.7106713E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)2934, PH.base.pack) ;
        p232_hdop_SET((float) -2.9749336E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -3.1446568E38F, PH.base.pack) ;
        p232_vn_SET((float)1.0285761E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)1455260027055112642L, PH.base.pack) ;
        p232_vd_SET((float) -1.8301727E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p232_alt_SET((float)3.0945008E38F, PH.base.pack) ;
        p232_vdop_SET((float) -1.65425E38F, PH.base.pack) ;
        c_CommunicationChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)190, (uint8_t)105, (uint8_t)67, (uint8_t)133, (uint8_t)64, (uint8_t)20, (uint8_t)145, (uint8_t)234, (uint8_t)35, (uint8_t)141, (uint8_t)238, (uint8_t)116, (uint8_t)0, (uint8_t)171, (uint8_t)194, (uint8_t)19, (uint8_t)133, (uint8_t)17, (uint8_t)88, (uint8_t)152, (uint8_t)100, (uint8_t)99, (uint8_t)27, (uint8_t)7, (uint8_t)195, (uint8_t)181, (uint8_t)55, (uint8_t)82, (uint8_t)80, (uint8_t)116, (uint8_t)24, (uint8_t)45, (uint8_t)11, (uint8_t)59, (uint8_t)210, (uint8_t)115, (uint8_t)176, (uint8_t)199, (uint8_t)34, (uint8_t)147, (uint8_t)83, (uint8_t)230, (uint8_t)165, (uint8_t)61, (uint8_t)85, (uint8_t)133, (uint8_t)76, (uint8_t)52, (uint8_t)205, (uint8_t)2, (uint8_t)164, (uint8_t)180, (uint8_t)114, (uint8_t)211, (uint8_t)21, (uint8_t)184, (uint8_t)64, (uint8_t)65, (uint8_t)193, (uint8_t)96, (uint8_t)7, (uint8_t)208, (uint8_t)234, (uint8_t)104, (uint8_t)101, (uint8_t)156, (uint8_t)237, (uint8_t)220, (uint8_t)92, (uint8_t)64, (uint8_t)51, (uint8_t)46, (uint8_t)197, (uint8_t)158, (uint8_t)188, (uint8_t)85, (uint8_t)239, (uint8_t)175, (uint8_t)1, (uint8_t)90, (uint8_t)244, (uint8_t)62, (uint8_t)108, (uint8_t)111, (uint8_t)172, (uint8_t)205, (uint8_t)3, (uint8_t)237, (uint8_t)42, (uint8_t)212, (uint8_t)160, (uint8_t)10, (uint8_t)179, (uint8_t)170, (uint8_t)157, (uint8_t)97, (uint8_t)13, (uint8_t)121, (uint8_t)188, (uint8_t)249, (uint8_t)81, (uint8_t)247, (uint8_t)128, (uint8_t)34, (uint8_t)185, (uint8_t)91, (uint8_t)78, (uint8_t)41, (uint8_t)23, (uint8_t)8, (uint8_t)94, (uint8_t)207, (uint8_t)237, (uint8_t)123, (uint8_t)4, (uint8_t)22, (uint8_t)189, (uint8_t)16, (uint8_t)189, (uint8_t)33, (uint8_t)26, (uint8_t)182, (uint8_t)204, (uint8_t)16, (uint8_t)123, (uint8_t)224, (uint8_t)164, (uint8_t)216, (uint8_t)236, (uint8_t)152, (uint8_t)87, (uint8_t)55, (uint8_t)188, (uint8_t)100, (uint8_t)49, (uint8_t)29, (uint8_t)167, (uint8_t)178, (uint8_t)72, (uint8_t)181, (uint8_t)80, (uint8_t)9, (uint8_t)118, (uint8_t)183, (uint8_t)17, (uint8_t)230, (uint8_t)118, (uint8_t)118, (uint8_t)147, (uint8_t)175, (uint8_t)140, (uint8_t)34, (uint8_t)91, (uint8_t)77, (uint8_t)185, (uint8_t)102, (uint8_t)29, (uint8_t)2, (uint8_t)78, (uint8_t)229, (uint8_t)153, (uint8_t)211, (uint8_t)108, (uint8_t)210, (uint8_t)40, (uint8_t)56, (uint8_t)11, (uint8_t)160, (uint8_t)213, (uint8_t)109, (uint8_t)125, (uint8_t)248, (uint8_t)248, (uint8_t)164, (uint8_t)207, (uint8_t)110, (uint8_t)235, (uint8_t)185, (uint8_t)45, (uint8_t)42};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HIGH_LATENCY_234(), &PH);
        p234_temperature_SET((int8_t)(int8_t)46, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)24062, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)4114096407L, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)62110, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)94, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -725, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -5671, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -28086, PH.base.pack) ;
        p234_latitude_SET((int32_t)108767120, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -100, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
        p234_longitude_SET((int32_t)1358338555, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)58309, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -26246, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)4, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
        c_CommunicationChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)882635702220982184L, PH.base.pack) ;
        p241_vibration_x_SET((float)1.2221355E38F, PH.base.pack) ;
        p241_vibration_y_SET((float)3.0571384E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2471107131L, PH.base.pack) ;
        p241_vibration_z_SET((float)3.116239E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)512413848L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)4041231426L, PH.base.pack) ;
        c_CommunicationChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_HOME_POSITION_242(), &PH);
        p242_approach_y_SET((float)2.3535956E38F, PH.base.pack) ;
        p242_y_SET((float)1.247611E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -707791961, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)2454255099536895557L, &PH) ;
        p242_longitude_SET((int32_t)2146338447, PH.base.pack) ;
        p242_z_SET((float) -1.2219226E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.3114259E38F, PH.base.pack) ;
        {
            float q[] =  {-1.6533501E38F, -4.7287774E37F, -1.9639918E38F, -1.7300291E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_x_SET((float)2.8618478E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t)71992710, PH.base.pack) ;
        p242_x_SET((float)1.8047728E38F, PH.base.pack) ;
        c_CommunicationChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_HOME_POSITION_243(), &PH);
        {
            float q[] =  {1.4908407E38F, -2.7580852E38F, 1.0973722E38F, -2.5749864E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_x_SET((float)1.7091021E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t)163589690, PH.base.pack) ;
        p243_z_SET((float) -2.6938314E38F, PH.base.pack) ;
        p243_approach_x_SET((float) -5.394461E37F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)3354115931226010498L, &PH) ;
        p243_approach_y_SET((float)1.8215078E38F, PH.base.pack) ;
        p243_approach_z_SET((float) -3.046313E38F, PH.base.pack) ;
        p243_y_SET((float)1.8753587E37F, PH.base.pack) ;
        p243_altitude_SET((int32_t)831345242, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        p243_latitude_SET((int32_t) -2114986601, PH.base.pack) ;
        c_CommunicationChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t)1890316597, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)42821, PH.base.pack) ;
        c_CommunicationChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_ADSB_VEHICLE_246(), &PH);
        p246_hor_velocity_SET((uint16_t)(uint16_t)47304, PH.base.pack) ;
        p246_altitude_SET((int32_t) -1393685622, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -20173, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2441476185L, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)34982, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)29625, PH.base.pack) ;
        {
            char16_t* callsign = u"wz";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_lat_SET((int32_t)1226059304, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_PARACHUTE, PH.base.pack) ;
        p246_lon_SET((int32_t)405725548, PH.base.pack) ;
        c_CommunicationChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)2.9548535E37F, PH.base.pack) ;
        p247_id_SET((uint32_t)2079509726L, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)1.9880447E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)1.4039092E38F, PH.base.pack) ;
        c_CommunicationChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_V2_EXTENSION_248(), &PH);
        p248_target_network_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)60249, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)232, (uint8_t)26, (uint8_t)217, (uint8_t)233, (uint8_t)83, (uint8_t)227, (uint8_t)105, (uint8_t)106, (uint8_t)193, (uint8_t)18, (uint8_t)16, (uint8_t)238, (uint8_t)32, (uint8_t)65, (uint8_t)25, (uint8_t)179, (uint8_t)111, (uint8_t)221, (uint8_t)55, (uint8_t)23, (uint8_t)200, (uint8_t)168, (uint8_t)96, (uint8_t)148, (uint8_t)227, (uint8_t)72, (uint8_t)111, (uint8_t)45, (uint8_t)169, (uint8_t)64, (uint8_t)208, (uint8_t)207, (uint8_t)16, (uint8_t)21, (uint8_t)53, (uint8_t)182, (uint8_t)153, (uint8_t)144, (uint8_t)146, (uint8_t)207, (uint8_t)191, (uint8_t)248, (uint8_t)244, (uint8_t)242, (uint8_t)115, (uint8_t)96, (uint8_t)163, (uint8_t)59, (uint8_t)174, (uint8_t)185, (uint8_t)36, (uint8_t)167, (uint8_t)110, (uint8_t)110, (uint8_t)195, (uint8_t)32, (uint8_t)142, (uint8_t)75, (uint8_t)25, (uint8_t)234, (uint8_t)187, (uint8_t)88, (uint8_t)75, (uint8_t)171, (uint8_t)149, (uint8_t)69, (uint8_t)26, (uint8_t)183, (uint8_t)94, (uint8_t)99, (uint8_t)189, (uint8_t)27, (uint8_t)154, (uint8_t)69, (uint8_t)13, (uint8_t)47, (uint8_t)75, (uint8_t)166, (uint8_t)100, (uint8_t)196, (uint8_t)73, (uint8_t)175, (uint8_t)67, (uint8_t)109, (uint8_t)227, (uint8_t)221, (uint8_t)126, (uint8_t)219, (uint8_t)54, (uint8_t)234, (uint8_t)177, (uint8_t)122, (uint8_t)75, (uint8_t)97, (uint8_t)217, (uint8_t)112, (uint8_t)83, (uint8_t)43, (uint8_t)82, (uint8_t)44, (uint8_t)9, (uint8_t)167, (uint8_t)205, (uint8_t)5, (uint8_t)136, (uint8_t)234, (uint8_t)17, (uint8_t)159, (uint8_t)120, (uint8_t)128, (uint8_t)192, (uint8_t)72, (uint8_t)1, (uint8_t)218, (uint8_t)85, (uint8_t)77, (uint8_t)97, (uint8_t)246, (uint8_t)120, (uint8_t)215, (uint8_t)9, (uint8_t)31, (uint8_t)157, (uint8_t)68, (uint8_t)241, (uint8_t)152, (uint8_t)170, (uint8_t)47, (uint8_t)250, (uint8_t)178, (uint8_t)61, (uint8_t)180, (uint8_t)210, (uint8_t)125, (uint8_t)9, (uint8_t)201, (uint8_t)104, (uint8_t)148, (uint8_t)80, (uint8_t)60, (uint8_t)88, (uint8_t)76, (uint8_t)51, (uint8_t)38, (uint8_t)177, (uint8_t)152, (uint8_t)241, (uint8_t)58, (uint8_t)13, (uint8_t)189, (uint8_t)83, (uint8_t)161, (uint8_t)249, (uint8_t)169, (uint8_t)252, (uint8_t)178, (uint8_t)90, (uint8_t)183, (uint8_t)151, (uint8_t)46, (uint8_t)67, (uint8_t)46, (uint8_t)96, (uint8_t)131, (uint8_t)179, (uint8_t)103, (uint8_t)223, (uint8_t)93, (uint8_t)48, (uint8_t)131, (uint8_t)39, (uint8_t)56, (uint8_t)122, (uint8_t)205, (uint8_t)14, (uint8_t)118, (uint8_t)88, (uint8_t)126, (uint8_t)138, (uint8_t)86, (uint8_t)192, (uint8_t)62, (uint8_t)96, (uint8_t)220, (uint8_t)92, (uint8_t)181, (uint8_t)43, (uint8_t)123, (uint8_t)220, (uint8_t)35, (uint8_t)124, (uint8_t)178, (uint8_t)137, (uint8_t)168, (uint8_t)229, (uint8_t)21, (uint8_t)60, (uint8_t)150, (uint8_t)244, (uint8_t)41, (uint8_t)24, (uint8_t)121, (uint8_t)48, (uint8_t)188, (uint8_t)79, (uint8_t)248, (uint8_t)109, (uint8_t)66, (uint8_t)88, (uint8_t)191, (uint8_t)250, (uint8_t)110, (uint8_t)113, (uint8_t)157, (uint8_t)127, (uint8_t)229, (uint8_t)59, (uint8_t)139, (uint8_t)189, (uint8_t)184, (uint8_t)156, (uint8_t)120, (uint8_t)248, (uint8_t)32, (uint8_t)66, (uint8_t)113, (uint8_t)94, (uint8_t)108, (uint8_t)74, (uint8_t)85, (uint8_t)106, (uint8_t)39, (uint8_t)41, (uint8_t)162, (uint8_t)172, (uint8_t)11, (uint8_t)158, (uint8_t)116, (uint8_t)253, (uint8_t)244, (uint8_t)157, (uint8_t)249, (uint8_t)56, (uint8_t)144, (uint8_t)174, (uint8_t)127, (uint8_t)224, (uint8_t)203, (uint8_t)232};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        c_CommunicationChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t)2, (int8_t) -113, (int8_t)110, (int8_t) -86, (int8_t) -3, (int8_t)1, (int8_t) -103, (int8_t) -42, (int8_t)52, (int8_t)108, (int8_t)8, (int8_t) -95, (int8_t) -97, (int8_t) -42, (int8_t) -127, (int8_t) -91, (int8_t) -37, (int8_t)107, (int8_t)71, (int8_t) -56, (int8_t)75, (int8_t)125, (int8_t)5, (int8_t) -24, (int8_t) -41, (int8_t)102, (int8_t)99, (int8_t)110, (int8_t) -47, (int8_t)85, (int8_t)94, (int8_t) -43};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)41376, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        c_CommunicationChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_VECT_250(), &PH);
        {
            char16_t* name = u"oifhJakzca";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float)1.5941028E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)8159568209444630093L, PH.base.pack) ;
        p250_x_SET((float) -1.5332906E38F, PH.base.pack) ;
        p250_y_SET((float)1.2139076E38F, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"xagvlrCc";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float)3.1352003E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)2654537867L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_NAMED_VALUE_INT_252(), &PH);
        {
            char16_t* name = u"hzydiqpgpo";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t) -1035226958, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)359935504L, PH.base.pack) ;
        c_CommunicationChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"ocmnkfkwbjctalIvPsnexVcEvnuGjh";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
        c_CommunicationChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_DEBUG_254(), &PH);
        p254_value_SET((float) -2.3882503E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)2963115417L, PH.base.pack) ;
        c_CommunicationChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SETUP_SIGNING_256(), &PH);
        p256_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)2741474669856642626L, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)177, (uint8_t)108, (uint8_t)247, (uint8_t)119, (uint8_t)73, (uint8_t)225, (uint8_t)206, (uint8_t)55, (uint8_t)44, (uint8_t)177, (uint8_t)53, (uint8_t)190, (uint8_t)27, (uint8_t)85, (uint8_t)77, (uint8_t)15, (uint8_t)165, (uint8_t)99, (uint8_t)113, (uint8_t)74, (uint8_t)128, (uint8_t)168, (uint8_t)202, (uint8_t)4, (uint8_t)36, (uint8_t)75, (uint8_t)199, (uint8_t)211, (uint8_t)62, (uint8_t)57, (uint8_t)167, (uint8_t)122};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)2646368701L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)280596937L, PH.base.pack) ;
        c_CommunicationChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PLAY_TUNE_258(), &PH);
        {
            char16_t* tune = u"bgpi";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        c_CommunicationChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_INFORMATION_259(), &PH);
        p259_cam_definition_version_SET((uint16_t)(uint16_t)64635, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"csfakbquavzzl";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_sensor_size_h_SET((float) -2.2215445E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)86, (uint8_t)26, (uint8_t)212, (uint8_t)132, (uint8_t)252, (uint8_t)251, (uint8_t)104, (uint8_t)82, (uint8_t)32, (uint8_t)85, (uint8_t)152, (uint8_t)124, (uint8_t)7, (uint8_t)253, (uint8_t)220, (uint8_t)205, (uint8_t)188, (uint8_t)52, (uint8_t)210, (uint8_t)142, (uint8_t)89, (uint8_t)181, (uint8_t)103, (uint8_t)127, (uint8_t)84, (uint8_t)43, (uint8_t)223, (uint8_t)155, (uint8_t)74, (uint8_t)71, (uint8_t)8, (uint8_t)29};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float) -1.5833426E38F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)22763, PH.base.pack) ;
        p259_focal_length_SET((float)1.0712429E38F, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)21567, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)86, (uint8_t)164, (uint8_t)171, (uint8_t)128, (uint8_t)78, (uint8_t)86, (uint8_t)149, (uint8_t)142, (uint8_t)236, (uint8_t)177, (uint8_t)130, (uint8_t)191, (uint8_t)163, (uint8_t)176, (uint8_t)57, (uint8_t)102, (uint8_t)145, (uint8_t)214, (uint8_t)142, (uint8_t)32, (uint8_t)132, (uint8_t)84, (uint8_t)182, (uint8_t)229, (uint8_t)122, (uint8_t)163, (uint8_t)36, (uint8_t)144, (uint8_t)128, (uint8_t)133, (uint8_t)37, (uint8_t)30};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_time_boot_ms_SET((uint32_t)3118774207L, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)2918143373L, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)3535862960L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float) -2.879073E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -1.1085021E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)588537755L, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p261_used_capacity_SET((float) -4.0689302E37F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p261_write_speed_SET((float) -1.2239222E38F, PH.base.pack) ;
        p261_total_capacity_SET((float)3.1888506E37F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        c_CommunicationChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_time_boot_ms_SET((uint32_t)2211287829L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.0744778E38F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)4159496735L, PH.base.pack) ;
        p262_image_interval_SET((float) -1.23333E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        {
            char16_t* file_url = u"wjtokqurDuxltGbVoqjcbm";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_camera_id_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p263_lon_SET((int32_t) -713977606, PH.base.pack) ;
        p263_image_index_SET((int32_t) -431826598, PH.base.pack) ;
        {
            float q[] =  {1.1775467E38F, -3.3180424E38F, -8.0694616E37F, 5.0025793E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_time_boot_ms_SET((uint32_t)1210269162L, PH.base.pack) ;
        p263_lat_SET((int32_t) -799424672, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)6769819443334114525L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -736161159, PH.base.pack) ;
        p263_alt_SET((int32_t) -482453204, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -18, PH.base.pack) ;
        c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)3959992065160282516L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2860305402L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)6827057483460746991L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)5863134317068952273L, PH.base.pack) ;
        c_CommunicationChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)1418281958L, PH.base.pack) ;
        p265_pitch_SET((float)1.512047E38F, PH.base.pack) ;
        p265_yaw_SET((float)1.9685539E38F, PH.base.pack) ;
        p265_roll_SET((float) -5.4970527E37F, PH.base.pack) ;
        c_CommunicationChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)211, (uint8_t)140, (uint8_t)206, (uint8_t)121, (uint8_t)56, (uint8_t)56, (uint8_t)145, (uint8_t)148, (uint8_t)10, (uint8_t)154, (uint8_t)173, (uint8_t)125, (uint8_t)162, (uint8_t)163, (uint8_t)187, (uint8_t)51, (uint8_t)96, (uint8_t)179, (uint8_t)103, (uint8_t)121, (uint8_t)182, (uint8_t)178, (uint8_t)0, (uint8_t)168, (uint8_t)179, (uint8_t)18, (uint8_t)58, (uint8_t)188, (uint8_t)127, (uint8_t)155, (uint8_t)183, (uint8_t)88, (uint8_t)57, (uint8_t)40, (uint8_t)157, (uint8_t)178, (uint8_t)122, (uint8_t)180, (uint8_t)99, (uint8_t)11, (uint8_t)151, (uint8_t)73, (uint8_t)120, (uint8_t)68, (uint8_t)32, (uint8_t)227, (uint8_t)20, (uint8_t)133, (uint8_t)153, (uint8_t)76, (uint8_t)67, (uint8_t)97, (uint8_t)242, (uint8_t)6, (uint8_t)58, (uint8_t)117, (uint8_t)127, (uint8_t)122, (uint8_t)71, (uint8_t)217, (uint8_t)226, (uint8_t)195, (uint8_t)92, (uint8_t)200, (uint8_t)80, (uint8_t)66, (uint8_t)226, (uint8_t)51, (uint8_t)141, (uint8_t)159, (uint8_t)51, (uint8_t)43, (uint8_t)68, (uint8_t)224, (uint8_t)155, (uint8_t)239, (uint8_t)26, (uint8_t)187, (uint8_t)101, (uint8_t)152, (uint8_t)57, (uint8_t)14, (uint8_t)1, (uint8_t)113, (uint8_t)162, (uint8_t)71, (uint8_t)10, (uint8_t)97, (uint8_t)222, (uint8_t)187, (uint8_t)136, (uint8_t)154, (uint8_t)98, (uint8_t)234, (uint8_t)175, (uint8_t)195, (uint8_t)1, (uint8_t)58, (uint8_t)229, (uint8_t)145, (uint8_t)191, (uint8_t)74, (uint8_t)167, (uint8_t)176, (uint8_t)227, (uint8_t)214, (uint8_t)134, (uint8_t)215, (uint8_t)187, (uint8_t)42, (uint8_t)81, (uint8_t)203, (uint8_t)151, (uint8_t)52, (uint8_t)130, (uint8_t)251, (uint8_t)93, (uint8_t)174, (uint8_t)8, (uint8_t)194, (uint8_t)144, (uint8_t)217, (uint8_t)7, (uint8_t)255, (uint8_t)79, (uint8_t)195, (uint8_t)91, (uint8_t)87, (uint8_t)179, (uint8_t)174, (uint8_t)95, (uint8_t)215, (uint8_t)13, (uint8_t)81, (uint8_t)30, (uint8_t)44, (uint8_t)239, (uint8_t)69, (uint8_t)94, (uint8_t)30, (uint8_t)24, (uint8_t)155, (uint8_t)130, (uint8_t)247, (uint8_t)151, (uint8_t)113, (uint8_t)252, (uint8_t)187, (uint8_t)10, (uint8_t)56, (uint8_t)218, (uint8_t)248, (uint8_t)76, (uint8_t)247, (uint8_t)238, (uint8_t)235, (uint8_t)45, (uint8_t)12, (uint8_t)230, (uint8_t)82, (uint8_t)93, (uint8_t)39, (uint8_t)45, (uint8_t)230, (uint8_t)220, (uint8_t)137, (uint8_t)127, (uint8_t)249, (uint8_t)192, (uint8_t)143, (uint8_t)215, (uint8_t)182, (uint8_t)215, (uint8_t)125, (uint8_t)232, (uint8_t)104, (uint8_t)147, (uint8_t)60, (uint8_t)119, (uint8_t)131, (uint8_t)220, (uint8_t)17, (uint8_t)128, (uint8_t)198, (uint8_t)238, (uint8_t)77, (uint8_t)24, (uint8_t)177, (uint8_t)252, (uint8_t)183, (uint8_t)199, (uint8_t)212, (uint8_t)66, (uint8_t)131, (uint8_t)158, (uint8_t)180, (uint8_t)255, (uint8_t)0, (uint8_t)233, (uint8_t)12, (uint8_t)86, (uint8_t)18, (uint8_t)239, (uint8_t)240, (uint8_t)64, (uint8_t)194, (uint8_t)91, (uint8_t)148, (uint8_t)246, (uint8_t)236, (uint8_t)238, (uint8_t)180, (uint8_t)195, (uint8_t)2, (uint8_t)206, (uint8_t)6, (uint8_t)154, (uint8_t)103, (uint8_t)175, (uint8_t)117, (uint8_t)38, (uint8_t)204, (uint8_t)212, (uint8_t)10, (uint8_t)61, (uint8_t)224, (uint8_t)141, (uint8_t)104, (uint8_t)204, (uint8_t)22, (uint8_t)210, (uint8_t)129, (uint8_t)14, (uint8_t)200, (uint8_t)42, (uint8_t)143, (uint8_t)26, (uint8_t)5, (uint8_t)110, (uint8_t)132, (uint8_t)226, (uint8_t)182, (uint8_t)223, (uint8_t)186, (uint8_t)59, (uint8_t)87, (uint8_t)253, (uint8_t)182, (uint8_t)128};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)49581, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)24, (uint8_t)77, (uint8_t)175, (uint8_t)110, (uint8_t)56, (uint8_t)174, (uint8_t)58, (uint8_t)255, (uint8_t)248, (uint8_t)152, (uint8_t)40, (uint8_t)221, (uint8_t)115, (uint8_t)140, (uint8_t)239, (uint8_t)237, (uint8_t)167, (uint8_t)25, (uint8_t)161, (uint8_t)18, (uint8_t)189, (uint8_t)104, (uint8_t)125, (uint8_t)105, (uint8_t)181, (uint8_t)29, (uint8_t)1, (uint8_t)145, (uint8_t)94, (uint8_t)229, (uint8_t)176, (uint8_t)134, (uint8_t)134, (uint8_t)227, (uint8_t)166, (uint8_t)21, (uint8_t)128, (uint8_t)46, (uint8_t)239, (uint8_t)250, (uint8_t)161, (uint8_t)49, (uint8_t)35, (uint8_t)42, (uint8_t)121, (uint8_t)124, (uint8_t)160, (uint8_t)108, (uint8_t)162, (uint8_t)227, (uint8_t)229, (uint8_t)69, (uint8_t)81, (uint8_t)40, (uint8_t)67, (uint8_t)219, (uint8_t)233, (uint8_t)5, (uint8_t)147, (uint8_t)225, (uint8_t)218, (uint8_t)165, (uint8_t)168, (uint8_t)221, (uint8_t)107, (uint8_t)162, (uint8_t)234, (uint8_t)203, (uint8_t)35, (uint8_t)84, (uint8_t)213, (uint8_t)93, (uint8_t)174, (uint8_t)101, (uint8_t)52, (uint8_t)118, (uint8_t)8, (uint8_t)168, (uint8_t)80, (uint8_t)34, (uint8_t)191, (uint8_t)144, (uint8_t)129, (uint8_t)154, (uint8_t)225, (uint8_t)40, (uint8_t)27, (uint8_t)13, (uint8_t)46, (uint8_t)178, (uint8_t)220, (uint8_t)100, (uint8_t)223, (uint8_t)164, (uint8_t)128, (uint8_t)210, (uint8_t)86, (uint8_t)250, (uint8_t)67, (uint8_t)209, (uint8_t)190, (uint8_t)18, (uint8_t)35, (uint8_t)153, (uint8_t)23, (uint8_t)243, (uint8_t)223, (uint8_t)191, (uint8_t)12, (uint8_t)220, (uint8_t)139, (uint8_t)6, (uint8_t)193, (uint8_t)162, (uint8_t)131, (uint8_t)214, (uint8_t)90, (uint8_t)196, (uint8_t)218, (uint8_t)142, (uint8_t)153, (uint8_t)225, (uint8_t)97, (uint8_t)1, (uint8_t)203, (uint8_t)137, (uint8_t)221, (uint8_t)156, (uint8_t)188, (uint8_t)144, (uint8_t)76, (uint8_t)158, (uint8_t)167, (uint8_t)207, (uint8_t)203, (uint8_t)79, (uint8_t)143, (uint8_t)194, (uint8_t)101, (uint8_t)13, (uint8_t)30, (uint8_t)194, (uint8_t)72, (uint8_t)15, (uint8_t)169, (uint8_t)33, (uint8_t)194, (uint8_t)252, (uint8_t)21, (uint8_t)89, (uint8_t)51, (uint8_t)184, (uint8_t)132, (uint8_t)157, (uint8_t)20, (uint8_t)190, (uint8_t)163, (uint8_t)117, (uint8_t)34, (uint8_t)172, (uint8_t)107, (uint8_t)159, (uint8_t)156, (uint8_t)22, (uint8_t)142, (uint8_t)109, (uint8_t)18, (uint8_t)51, (uint8_t)51, (uint8_t)190, (uint8_t)251, (uint8_t)43, (uint8_t)194, (uint8_t)137, (uint8_t)162, (uint8_t)84, (uint8_t)25, (uint8_t)119, (uint8_t)209, (uint8_t)178, (uint8_t)137, (uint8_t)16, (uint8_t)161, (uint8_t)240, (uint8_t)103, (uint8_t)100, (uint8_t)203, (uint8_t)151, (uint8_t)190, (uint8_t)252, (uint8_t)92, (uint8_t)223, (uint8_t)193, (uint8_t)43, (uint8_t)11, (uint8_t)184, (uint8_t)226, (uint8_t)252, (uint8_t)215, (uint8_t)49, (uint8_t)54, (uint8_t)78, (uint8_t)213, (uint8_t)208, (uint8_t)80, (uint8_t)162, (uint8_t)166, (uint8_t)117, (uint8_t)30, (uint8_t)169, (uint8_t)65, (uint8_t)18, (uint8_t)249, (uint8_t)93, (uint8_t)21, (uint8_t)166, (uint8_t)19, (uint8_t)185, (uint8_t)39, (uint8_t)220, (uint8_t)111, (uint8_t)54, (uint8_t)48, (uint8_t)35, (uint8_t)241, (uint8_t)103, (uint8_t)23, (uint8_t)156, (uint8_t)107, (uint8_t)13, (uint8_t)180, (uint8_t)36, (uint8_t)38, (uint8_t)67, (uint8_t)74, (uint8_t)255, (uint8_t)148, (uint8_t)71, (uint8_t)178, (uint8_t)116, (uint8_t)214, (uint8_t)141, (uint8_t)248, (uint8_t)76, (uint8_t)39, (uint8_t)76, (uint8_t)67, (uint8_t)39, (uint8_t)0};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_sequence_SET((uint16_t)(uint16_t)7401, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)35260, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        c_CommunicationChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_status_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)31445, PH.base.pack) ;
        p269_framerate_SET((float) -2.2996314E38F, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)54015, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)30691, PH.base.pack) ;
        {
            char16_t* uri = u"eeqesmgbaslwylbzlqz";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_bitrate_SET((uint32_t)472520685L, PH.base.pack) ;
        c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_bitrate_SET((uint32_t)1438391989L, PH.base.pack) ;
        p270_framerate_SET((float) -5.5285564E37F, PH.base.pack) ;
        {
            char16_t* uri = u"dqhzmCrHDUlojctwoalGmohezmbyguoqmeslczjyhggxpqwpxartydtvtvecuixsxsuqrxymqoubdYghhsiftheSvbYfdtuxfpMwebkWOtRtqehewhhhAmghrvpsHfsgfzlcokdD";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_rotation_SET((uint16_t)(uint16_t)63578, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)11986, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)36259, PH.base.pack) ;
        c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"mhwcgksvkpcdpfnsiwebMfgqgegbaxxadhzDutsfutfgovYkoe";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"jjPhxdwafhipmcyyqelihgfex";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_CommunicationChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PROTOCOL_VERSION_300(), &PH);
        p300_min_version_SET((uint16_t)(uint16_t)48381, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)23744, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)249, (uint8_t)226, (uint8_t)30, (uint8_t)99, (uint8_t)246, (uint8_t)27, (uint8_t)205, (uint8_t)7};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)28604, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)252, (uint8_t)8, (uint8_t)12, (uint8_t)187, (uint8_t)180, (uint8_t)76, (uint8_t)158, (uint8_t)219};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)19124, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)133071442L, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_CRITICAL, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)8154517258817837868L, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_uptime_sec_SET((uint32_t)1971430405L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)2876643348953795868L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)237, (uint8_t)233, (uint8_t)76, (uint8_t)53, (uint8_t)236, (uint8_t)138, (uint8_t)190, (uint8_t)11, (uint8_t)193, (uint8_t)147, (uint8_t)220, (uint8_t)46, (uint8_t)61, (uint8_t)198, (uint8_t)60, (uint8_t)16};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        {
            char16_t* name = u"lkEajxchnADgdjymiwuihykksmulBudChmgbtkDfqotiaom";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_version_minor_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)316094876L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"vmmzjbhCfe";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t) -5115, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)34112, PH.base.pack) ;
        {
            char16_t* param_value = u"lugnLnfykmhjhMtxtnnhn";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)48508, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_id = u"galq";
            p322_param_id_SET_(param_id, &PH) ;
        }
        c_CommunicationChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_value = u"lBsmjqpqdipwllmxxdhcxqUzyauqklxQfzqzlwbEeiGegeCmLkcdagKyamRkjvotGpynzprWnd";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        {
            char16_t* param_id = u"vlpalqvw";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"iaS";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        {
            char16_t* param_id = u"qskwlDoxyaVu";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
        c_CommunicationChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_TEST_Channel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_increment_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)59158, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)13695, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)7180420783902995200L, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)49166, (uint16_t)52684, (uint16_t)2198, (uint16_t)56994, (uint16_t)13890, (uint16_t)59265, (uint16_t)65203, (uint16_t)44098, (uint16_t)9084, (uint16_t)6933, (uint16_t)35454, (uint16_t)13097, (uint16_t)64739, (uint16_t)41073, (uint16_t)18316, (uint16_t)43699, (uint16_t)39750, (uint16_t)37281, (uint16_t)51553, (uint16_t)65342, (uint16_t)39108, (uint16_t)22871, (uint16_t)3155, (uint16_t)13512, (uint16_t)13708, (uint16_t)60703, (uint16_t)54336, (uint16_t)2545, (uint16_t)42827, (uint16_t)42271, (uint16_t)51833, (uint16_t)15201, (uint16_t)60999, (uint16_t)28286, (uint16_t)40280, (uint16_t)50843, (uint16_t)40735, (uint16_t)393, (uint16_t)9844, (uint16_t)57105, (uint16_t)3758, (uint16_t)26045, (uint16_t)8787, (uint16_t)22542, (uint16_t)35023, (uint16_t)26569, (uint16_t)64225, (uint16_t)5695, (uint16_t)12930, (uint16_t)53732, (uint16_t)31017, (uint16_t)61962, (uint16_t)26745, (uint16_t)64794, (uint16_t)44448, (uint16_t)55557, (uint16_t)22871, (uint16_t)36455, (uint16_t)58459, (uint16_t)59300, (uint16_t)24537, (uint16_t)60795, (uint16_t)35984, (uint16_t)16702, (uint16_t)56165, (uint16_t)39272, (uint16_t)53206, (uint16_t)41548, (uint16_t)50761, (uint16_t)41295, (uint16_t)14727, (uint16_t)3777};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_TEST_Channel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = input_bytes_adv(&c_TEST_Channel, buff, sizeof buff));) c_CommunicationChannel_output_bytes(buff,  len);
        c_CommunicationChannel_process_received();// process received pack on receiver side
    }
}

